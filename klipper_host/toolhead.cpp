#define NOMINMAX
#include "toolhead.h"
#include "klipper_mcu.h"

#include <algorithm>
#include <cmath>
#include <iostream>

// ========== ToolHead ==========

ToolHead::ToolHead(KlipperMCU& mcu) : m_mcu(mcu) {}

void ToolHead::setMaxVelocity(double maxVel) {
    m_maxVel = maxVel;
}

void ToolHead::setMaxAccel(double maxAccel) {
    m_maxAccel = maxAccel;
}

void ToolHead::setJunctionDeviation(double jd) {
    m_junctionDeviation = jd;
}

void ToolHead::setSquareCornerVelocity(double scv) {
    m_squareCornerVelocity = scv;
    // Compute junction_deviation from square_corner_velocity:
    // jd = scv^2 / (2 * accel)  (Klipper's formula)
    m_junctionDeviation = (scv * scv) / (2.0 * m_maxAccel);
}

void ToolHead::addStepper(int axis, MCU_stepper* stepper) {
    if (axis >= 0 && axis < 3) {
        m_steppers[axis] = stepper;
    }
}

void ToolHead::moveAbsolute(const Vec3& pos, double speed) {
    speed = std::min(speed, m_maxVel);
    if (speed <= 0) speed = m_maxVel;

    Move move(m_pos, pos, speed, m_maxAccel);

    if (!move.is_kinematic_move) return;

    // Calculate junction velocity with previous move
    Move* prev = m_queue.empty() ? nullptr : &m_queue.back();
    move.calcJunctionV2(prev, m_junctionDeviation);

    // Limit max start velocity
    move.max_start_v2 = std::min(move.max_junction_v2,
                                  move.max_cruise_v2);

    m_queue.push_back(std::move(move));
    m_pos = pos;

    // Flush lookahead if queue is getting long
    if (m_queue.size() >= 16) {
        lookaheadFlush(false);
    }
}

void ToolHead::moveRelative(const Vec3& delta, double speed) {
    Vec3 target = m_pos + delta;
    moveAbsolute(target, speed);
}

void ToolHead::flush() {
    if (!m_queue.empty()) {
        lookaheadFlush(true);
    }
}

void ToolHead::lookaheadFlush(bool forceFlush) {
    if (m_queue.empty()) return;

    size_t count = m_queue.size();

    // REVERSE PASS: propagate max_start_v2 backwards
    // Last move must end at 0 (if force flush)
    double end_v2 = forceFlush ? 0.0 : m_queue.back().max_cruise_v2;

    for (int i = static_cast<int>(count) - 1; i >= 0; --i) {
        Move& move = m_queue[i];
        // The move's start velocity is limited by what end velocity the
        // braking phase can reach: v_start² = v_end² + 2*a*d
        // But it can't exceed junction limit
        double reachable = end_v2 + 2.0 * move.accel * move.move_d;
        move.max_start_v2 = std::min(move.max_start_v2,
                                      std::min(reachable, move.max_cruise_v2));
        end_v2 = move.max_start_v2;
    }

    // FORWARD PASS: propagate start_v2 forward, set junctions
    double prev_end_v2 = 0.0;

    for (size_t i = 0; i < count; ++i) {
        Move& move = m_queue[i];

        // Start velocity: limited by previous move's end velocity AND
        // what acceleration can reach over the distance
        double start_v2 = std::min(move.max_start_v2, prev_end_v2);

        // Cruise velocity: limited by max cruise speed
        double cruise_v2 = move.max_cruise_v2;

        // End velocity: limited by what we can reach from start and
        // also by the next move's max_start (or 0 if flush)
        double end_v2_limit;
        if (i + 1 < count) {
            end_v2_limit = m_queue[i + 1].max_start_v2;
        } else {
            end_v2_limit = forceFlush ? 0.0 : cruise_v2;
        }

        // Can't exceed what acceleration allows: v² = v0² + 2*a*d
        double max_end_from_start = start_v2 + 2.0 * move.accel * move.move_d;
        double end_v2_val = std::min(end_v2_limit, max_end_from_start);
        end_v2_val = std::min(end_v2_val, cruise_v2);

        // Set the trapezoid
        move.setJunction(start_v2, cruise_v2, end_v2_val);

        // Assign print time
        move.print_time = m_nextPrintTime;
        m_nextPrintTime += move.accel_t + move.cruise_t + move.decel_t;

        prev_end_v2 = end_v2_val;
    }

    // Generate TrapMoves and add to queue
    for (auto& move : m_queue) {
        auto trapMoves = move.toTrapMoves();
        m_trapq.append(trapMoves);
    }

    m_queue.clear();
}

void ToolHead::generateAxisSteps(int axis, const TrapMove& tm, bool needsReset) {
    MCU_stepper* stepper = m_steppers[axis];
    if (!stepper) return;

    // Get axis component of movement
    double axisR = 0;
    switch (axis) {
        case 0: axisR = tm.axes_r.x; break;
        case 1: axisR = tm.axes_r.y; break;
        case 2: axisR = tm.axes_r.z; break;
    }
    if (std::abs(axisR) < 0.000000001) return;

    double stepDist = stepper->getStepDist();
    if (stepDist <= 0) return;

    double mcuFreq = m_mcu.getClockSync().getMcuFreq();

    // Axis-projected kinematic parameters
    double v0 = std::abs(axisR) * tm.start_v;        // start velocity on this axis (mm/s)
    double accel = std::abs(axisR) * 2.0 * tm.half_accel; // acceleration on this axis (mm/s²)

    // Total axis distance for this phase
    double totalDist = v0 * tm.move_t + 0.5 * accel * tm.move_t * tm.move_t;
    if (totalDist < stepDist * 0.5) return;

    int numSteps = static_cast<int>(totalDist / stepDist + 0.5);
    if (numSteps <= 0) return;

    // Set direction
    bool forward = (axisR > 0);
    stepper->setNextStepDir(forward);

    // Absolute MCU clock at TrapMove start
    int64_t tmStartClock = m_mcu.getClockSync().printTimeToClock(tm.print_time);

    // === Compute exact step times using kinematic equations ===
    // Position along axis: p(t) = v0*t + 0.5*accel*t²
    // Step n occurs when p(t_n) = (n+1) * stepDist
    // Solve: t_n = (-v0 + sqrt(v0² + 2*accel*(n+1)*stepDist)) / accel
    // For constant velocity: t_n = (n+1) * stepDist / v0

    std::vector<int64_t> stepClocks(numSteps);

    if (std::abs(accel) < 1e-6) {
        // Constant velocity phase
        double invV = 1.0 / std::max(v0, 1e-6);
        for (int i = 0; i < numSteps; i++) {
            double t = (i + 1) * stepDist * invV;
            stepClocks[i] = tmStartClock + static_cast<int64_t>(t * mcuFreq + 0.5);
        }
    } else {
        // Accelerating or decelerating phase
        double v0sq = v0 * v0;
        double inv_a = 1.0 / accel;
        for (int i = 0; i < numSteps; i++) {
            double pos = (i + 1) * stepDist;
            double disc = v0sq + 2.0 * accel * pos;
            if (disc < 0) {
                // Deceleration: ran out of distance (rounding at boundary)
                numSteps = i;
                break;
            }
            double t = (-v0 + std::sqrt(disc)) * inv_a;
            stepClocks[i] = tmStartClock + static_cast<int64_t>(t * mcuFreq + 0.5);
        }
        if (numSteps <= 0) return;
    }

    // === Decide: reset_step_clock or chain from previous move ===
    // Klipper architecture: reset_step_clock is called once at print start.
    // Between moves, queue_step commands chain seamlessly — the MCU's internal
    // step clock auto-advances after each step batch completes.
    bool doReset = false;
    if (needsReset) {
        if (!stepper->isClockInitialized()) {
            // First use ever: must reset
            doReset = true;
        } else {
            // Check if we can chain from lastStepClock
            int64_t gap = stepClocks[0] - stepper->getLastStepClock();
            if (gap <= 0 || gap > 4'000'000'000LL) {
                // Gap negative (scheduling error) or > ~13s: need reset
                doReset = true;
            }
            // else: chain from lastStepClock (no reset needed)
        }
    }

    if (doReset) {
        stepper->resetStepClock(tmStartClock);
    }

    // === Compress step clocks into queue_step(interval, count, add) commands ===
    // MCU constraints: interval is uint32 (practical max ~0x3FFFFFFF),
    //                  count is uint16 (max 65535),
    //                  add is int16 (range [-32768, 32767])

    int64_t mcuClockPos = doReset ? tmStartClock : stepper->getLastStepClock();

    int pos = 0;
    while (pos < numSteps) {
        int64_t firstInterval = stepClocks[pos] - mcuClockPos;
        if (firstInterval < 1) firstInterval = 1;

        if (pos + 1 >= numSteps) {
            // Single step remaining
            stepper->queueStep(firstInterval, 1, 0);
            mcuClockPos += firstInterval;
            pos++;
            continue;
        }

        // Compute add from first two step intervals
        int64_t secondInterval = stepClocks[pos + 1] - stepClocks[pos];
        if (secondInterval < 1) secondInterval = 1;
        int64_t add64 = secondInterval - firstInterval;

        // Clamp to int16 range
        int16_t add = static_cast<int16_t>(
            std::clamp(add64, (int64_t)-32768, (int64_t)32767));

        // Extend batch: keep adding steps while model (interval + add*i) stays accurate
        int count = 1;
        int64_t modelClock = mcuClockPos + firstInterval;

        while (pos + count < numSteps && count < 65535) {
            int64_t modelInterval = firstInterval + (int64_t)add * count;
            if (modelInterval < 1) break;

            int64_t modelNext = modelClock + modelInterval;
            int64_t actualClock = stepClocks[pos + count];
            int64_t error = std::abs(modelNext - actualClock);

            // Tolerance: allow up to 1/8 of interval or 300 ticks (1µs), whichever is larger
            int64_t tolerance = std::max((int64_t)300, modelInterval / 8);
            if (error > tolerance) break;

            modelClock = modelNext;
            count++;
        }

        stepper->queueStep(firstInterval, count, add);

        // Advance MCU clock by the exact model time
        // sum of intervals = count*firstInterval + count*(count-1)/2 * add
        mcuClockPos += (int64_t)count * firstInterval
                     + (int64_t)count * (count - 1) / 2 * (int64_t)add;
        pos += count;
    }

    // Track where the stepper's clock ended up (for next TrapMove continuation)
    stepper->setLastStepClock(mcuClockPos);
}

bool ToolHead::generateSteps() {
    auto trapMoves = m_trapq.getAndClear();
    if (trapMoves.empty()) return true;

    // Track whether each axis stepper has been reset for this batch.
    // Only the first TrapMove that moves a given axis should call
    // resetStepClock; subsequent phases continue from where queue_step
    // left off (MCU auto-advances its internal step clock).
    bool axisReset[3] = {false, false, false};

    for (auto& tm : trapMoves) {
        for (int axis = 0; axis < 3; ++axis) {
            generateAxisSteps(axis, tm, !axisReset[axis]);
            // Mark axis as reset if this TrapMove actually had motion on it
            double axisR = 0;
            switch (axis) {
                case 0: axisR = tm.axes_r.x; break;
                case 1: axisR = tm.axes_r.y; break;
                case 2: axisR = tm.axes_r.z; break;
            }
            if (std::abs(axisR) > 0.000000001) {
                axisReset[axis] = true;
            }
        }
    }

    return true;
}
