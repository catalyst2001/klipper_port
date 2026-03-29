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

void ToolHead::generateAxisSteps(int axis, const TrapMove& tm) {
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

    // Calculate total axis distance for this trap move
    double totalDist = std::abs(axisR) *
        (tm.start_v * tm.move_t + tm.half_accel * tm.move_t * tm.move_t);
    int numSteps = static_cast<int>(totalDist / stepDist + 0.5);
    if (numSteps <= 0) return;

    // Set direction
    bool forward = (axisR > 0);
    stepper->setNextStepDir(forward);

    // Reset step clock to the start of this move
    int64_t startClock = m_mcu.getClockSync().printTimeToClock(tm.print_time);
    stepper->resetStepClock(startClock);

    if (std::abs(tm.half_accel) < 0.000001) {
        // Constant velocity: uniform step intervals
        double velocity = std::abs(axisR) * tm.start_v;
        if (velocity < 0.001) return;
        int64_t interval = static_cast<int64_t>(mcuFreq * stepDist / velocity);
        if (interval < 1) interval = 1;
        stepper->queueStep(interval, numSteps, 0);
    } else {
        // Accelerating/decelerating: use itersolve-like approach
        // Step times computed using the secant method, but for queue_step
        // we approximate with interval + add (linear interpolation)
        double velocity_start = std::abs(axisR) * tm.start_v;
        double velocity_end = std::abs(axisR) * tm.getVelocity(tm.move_t);

        // Avoid division by zero
        if (velocity_start < 0.001) velocity_start = 0.001;
        if (velocity_end < 0.001) velocity_end = 0.001;

        int64_t interval_start = static_cast<int64_t>(mcuFreq * stepDist / velocity_start);
        int64_t interval_end = static_cast<int64_t>(mcuFreq * stepDist / velocity_end);

        if (interval_start < 1) interval_start = 1;
        if (interval_end < 1) interval_end = 1;

        // add = (interval_end - interval_start) / (numSteps - 1)
        int64_t add = 0;
        if (numSteps > 1) {
            add = (interval_end - interval_start) / (numSteps - 1);
        }

        stepper->queueStep(interval_start, numSteps, add);
    }
}

bool ToolHead::generateSteps() {
    auto trapMoves = m_trapq.getAndClear();
    if (trapMoves.empty()) return true;

    for (auto& tm : trapMoves) {
        for (int axis = 0; axis < 3; ++axis) {
            generateAxisSteps(axis, tm);
        }
    }

    return true;
}
