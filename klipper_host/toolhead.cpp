#define NOMINMAX
#include "toolhead.h"
#include "klipper_mcu.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <climits>
#include <thread>
#include <chrono>

// ========== Step Compression (port of Klipper's stepcompress.c) ==========

namespace {

struct StepMove {
    uint32_t interval;
    uint16_t count;
    int16_t add;
};

struct MinMaxPoint {
    int32_t minp, maxp;
};

inline int32_t sc_idiv_up(int32_t n, int32_t d) {
    return (n >= 0) ? (n + d - 1) / d : (n / d);
}

inline int32_t sc_idiv_down(int32_t n, int32_t d) {
    return (n >= 0) ? (n / d) : (n - d + 1) / d;
}

// Compute the acceptable position range for a step relative to lastStepClock.
// Matches Klipper's minmax_point from stepcompress.c
MinMaxPoint sc_minmax_point(const int64_t* stepClocks, int idx, int startIdx,
                            int64_t lastStepClock, uint32_t maxError) {
    int64_t diff = stepClocks[idx] - lastStepClock;
    uint32_t point = static_cast<uint32_t>(std::max(diff, (int64_t)0));
    uint32_t prevpoint = (idx > startIdx)
        ? static_cast<uint32_t>(std::max(stepClocks[idx - 1] - lastStepClock, (int64_t)0))
        : 0;
    uint32_t halfGap = (point - prevpoint) / 2;
    if (halfGap > maxError) halfGap = maxError;
    return { static_cast<int32_t>(point - halfGap), static_cast<int32_t>(point) };
}

// The maximum add delta between two valid quadratic sequences of the
// form "add*count*(count-1)/2 + interval*count" is "(6 + 4*sqrt(2)) *
// maxerror / (count*count)".  Using 11 works well in practice.
constexpr int32_t QUADRATIC_DEV = 11;

// Find optimal (interval, count, add) for a batch of steps.
// Direct port of Klipper's compress_bisect_add from stepcompress.c
StepMove sc_compress_bisect_add(const int64_t* stepClocks, int startIdx, int endIdx,
                                int64_t lastStepClock, uint32_t maxError) {
    int qlast = endIdx;
    if (qlast > startIdx + 65535)
        qlast = startIdx + 65535;

    auto point = sc_minmax_point(stepClocks, startIdx, startIdx, lastStepClock, maxError);
    int32_t outer_mininterval = point.minp, outer_maxinterval = point.maxp;
    int32_t add = 0, minadd = -0x8000, maxadd = 0x7FFF;
    int32_t bestinterval = 0, bestcount = 1, bestadd = 1, bestreach = INT32_MIN;
    int32_t zerointerval = 0, zerocount = 0;

    for (;;) {
        // Find longest valid sequence with the given 'add'
        MinMaxPoint nextpoint{};
        int32_t nextmininterval = outer_mininterval;
        int32_t nextmaxinterval = outer_maxinterval, interval = nextmaxinterval;
        int32_t nextcount = 1;

        for (;;) {
            nextcount++;
            if (startIdx + nextcount - 1 >= qlast) {
                int32_t count = nextcount - 1;
                return { static_cast<uint32_t>(interval),
                         static_cast<uint16_t>(count),
                         static_cast<int16_t>(add) };
            }
            nextpoint = sc_minmax_point(stepClocks, startIdx + nextcount - 1,
                                        startIdx, lastStepClock, maxError);
            int32_t nextaddfactor = nextcount * (nextcount - 1) / 2;
            int32_t c = add * nextaddfactor;
            if (nextmininterval * nextcount < nextpoint.minp - c)
                nextmininterval = sc_idiv_up(nextpoint.minp - c, nextcount);
            if (nextmaxinterval * nextcount > nextpoint.maxp - c)
                nextmaxinterval = sc_idiv_down(nextpoint.maxp - c, nextcount);
            if (nextmininterval > nextmaxinterval)
                break;
            interval = nextmaxinterval;
        }

        // Check if this is the best sequence found so far
        int32_t count = nextcount - 1;
        int32_t addfactor = count * (count - 1) / 2;
        int32_t reach = add * addfactor + interval * count;
        if (reach > bestreach
            || (reach == bestreach && interval > bestinterval)) {
            bestinterval = interval;
            bestcount = count;
            bestadd = add;
            bestreach = reach;
            if (!add) {
                zerointerval = interval;
                zerocount = count;
            }
            if (count > 0x200)
                // No 'add' will improve sequence; avoid integer overflow
                break;
        }

        // Check if a greater or lesser add could extend the sequence
        int32_t nextaddfactor = nextcount * (nextcount - 1) / 2;
        int32_t nextreach = add * nextaddfactor + interval * nextcount;
        if (nextreach < nextpoint.minp) {
            minadd = add + 1;
            outer_maxinterval = nextmaxinterval;
        } else {
            maxadd = add - 1;
            outer_mininterval = nextmininterval;
        }

        // The maximum valid deviation between two quadratic sequences
        // can be calculated and used to further limit the add range.
        if (count > 1) {
            int32_t errdelta = static_cast<int32_t>(maxError) * QUADRATIC_DEV
                             / (count * count);
            if (minadd < add - errdelta)
                minadd = add - errdelta;
            if (maxadd > add + errdelta)
                maxadd = add + errdelta;
        }

        // See if next point would further limit the add range
        int32_t c2 = outer_maxinterval * nextcount;
        int32_t nextaddfactor2 = nextcount * (nextcount - 1) / 2;
        if (minadd * nextaddfactor2 < nextpoint.minp - c2)
            minadd = sc_idiv_up(nextpoint.minp - c2, nextaddfactor2);
        c2 = outer_mininterval * nextcount;
        if (maxadd * nextaddfactor2 > nextpoint.maxp - c2)
            maxadd = sc_idiv_down(nextpoint.maxp - c2, nextaddfactor2);

        // Bisect valid add range and try again with new 'add'
        if (minadd > maxadd)
            break;
        add = maxadd - (maxadd - minadd) / 4;
    }

    if (zerocount + zerocount / 16 >= bestcount)
        // Prefer add=0 if it's similar to the best found sequence
        return { static_cast<uint32_t>(zerointerval),
                 static_cast<uint16_t>(zerocount), 0 };
    return { static_cast<uint32_t>(bestinterval),
             static_cast<uint16_t>(bestcount),
             static_cast<int16_t>(bestadd) };
}

} // anonymous namespace

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
    // jd = scv^2 * (sqrt(2) - 1) / max_accel  (Klipper's formula)
    double scv2 = scv * scv;
    m_junctionDeviation = scv2 * (std::sqrt(2.0) - 1.0) / m_maxAccel;
    m_mcrPseudoAccel = m_maxAccel * (1.0 - m_minCruiseRatio);
}

void ToolHead::addStepper(int axis, MCU_stepper* stepper) {
    if (axis >= 0 && axis < 3) {
        m_steppers[axis] = stepper;
    }
}

void ToolHead::moveAbsolute(const Vec3& pos, double speed) {
    speed = std::min(speed, m_maxVel);
    if (speed <= 0) speed = m_maxVel;

    Move move(m_pos, pos, speed, m_maxAccel, m_junctionDeviation, m_mcrPseudoAccel);

    if (!move.is_kinematic_move) return;

    // Calculate junction velocity with previous move
    Move* prev = m_queue.empty() ? nullptr : &m_queue.back();
    move.calcJunction(prev);

    m_queue.push_back(std::move(move));
    m_pos = pos;

    // Time-based flush trigger (like Klipper's junction_flush countdown)
    m_junctionFlush -= m_queue.back().min_move_t;
    if (m_junctionFlush <= 0.0) {
        lookaheadFlush(true);
    }

    // Backpressure: if we've queued enough, check if we need to wait for MCU
    if (m_nextPrintTime > m_needCheckPause) {
        checkPause();
    }
}

void ToolHead::moveRelative(const Vec3& delta, double speed) {
    Vec3 target = m_pos + delta;
    moveAbsolute(target, speed);
}

void ToolHead::flush() {
    if (!m_queue.empty()) {
        lookaheadFlush(false);
    }
    // Transition to idle — next move will re-sync print_time
    m_needStartSync = true;
    m_needCheckPause = -1.0;
}

void ToolHead::lookaheadFlush(bool lazy) {
    m_junctionFlush = LOOKAHEAD_FLUSH_TIME;

    if (m_queue.empty()) return;

    bool updateFlushCount = lazy;
    size_t flushCount = m_queue.size();

    // --- REVERSE PASS ---
    // Traverse queue from last to first move and determine maximum
    // junction speed assuming the robot comes to a complete stop
    // after the last move. Port of Klipper's LookAheadQueue.flush().

    struct JunctionInfo {
        size_t idx;
        double start_v2;
        double cruise_v2;     // -1.0 means "None" (needs forward propagation)
        double next_start_v2;
    };
    std::vector<JunctionInfo> jinfo(flushCount);

    double next_start_v2 = 0.0;
    double next_mcr_start_v2 = 0.0;
    double peak_cruise_v2 = 0.0;
    int pending_cv2_assign = 0;

    for (int i = static_cast<int>(flushCount) - 1; i >= 0; --i) {
        Move& move = m_queue[i];

        double reachable_start_v2 = next_start_v2 + move.delta_v2;
        double start_v2 = std::min(move.max_start_v2, reachable_start_v2);
        double cruise_v2 = -1.0;  // None
        pending_cv2_assign += 1;

        double reach_mcr_start_v2 = next_mcr_start_v2 + move.mcr_delta_v2;
        double mcr_start_v2 = std::min(move.max_mcr_start_v2, reach_mcr_start_v2);

        if (mcr_start_v2 < reach_mcr_start_v2) {
            // It's possible for this move to accelerate
            if (mcr_start_v2 + move.mcr_delta_v2 > next_mcr_start_v2
                || pending_cv2_assign > 1) {
                // This move can both accel and decel, or this is a
                // full accel move followed by a full decel move
                if (updateFlushCount && peak_cruise_v2 > 0.0) {
                    flushCount = static_cast<size_t>(i) + pending_cv2_assign;
                    updateFlushCount = false;
                }
                peak_cruise_v2 = (mcr_start_v2 + reach_mcr_start_v2) * 0.5;
            }
            cruise_v2 = std::min({(start_v2 + reachable_start_v2) * 0.5,
                                  move.max_cruise_v2,
                                  peak_cruise_v2});
            pending_cv2_assign = 0;
        }

        jinfo[i] = {static_cast<size_t>(i), start_v2, cruise_v2, next_start_v2};
        next_start_v2 = start_v2;
        next_mcr_start_v2 = mcr_start_v2;
    }

    if (updateFlushCount || flushCount == 0) {
        // Lazy flush found no confirmed velocity peak — nothing to flush yet
        return;
    }

    // If this is the first flush from idle, sync print_time to MCU clock
    if (m_needStartSync) {
        m_needStartSync = false;
        m_needCheckPause = -1.0;
        syncPrintTime();
    }

    // --- FORWARD PASS ---
    // Propagate cruise_v2 forward for moves that couldn't accelerate
    double prev_cruise_v2 = 0.0;

    for (size_t i = 0; i < flushCount; ++i) {
        auto& ji = jinfo[i];
        Move& move = m_queue[ji.idx];

        double cruise_v2 = ji.cruise_v2;
        if (cruise_v2 < 0.0) {
            // This move can't accelerate — propagate from previous
            cruise_v2 = std::min(prev_cruise_v2, ji.start_v2);
        }

        move.setJunction(std::min(ji.start_v2, cruise_v2),
                         cruise_v2,
                         std::min(ji.next_start_v2, cruise_v2));

        // Assign print time
        move.print_time = m_nextPrintTime;
        m_nextPrintTime += move.accel_t + move.cruise_t + move.decel_t;

        prev_cruise_v2 = cruise_v2;
    }

    // Generate TrapMoves for flushed moves and add to queue
    for (size_t i = 0; i < flushCount; ++i) {
        auto trapMoves = m_queue[i].toTrapMoves();
        m_trapq.append(trapMoves);
    }

    // Generate steps immediately (converts TrapMoves to queue_step commands)
    generateSteps();

    // Remove processed moves from the queue, keep the rest
    m_queue.erase(m_queue.begin(), m_queue.begin() + flushCount);
}

// Sync print_time to MCU clock when transitioning from idle to printing.
// Port of Klipper's ToolHead._calc_print_time()
void ToolHead::syncPrintTime() {
    double est = m_mcu.getClockSync().estimatedPrintTime();
    double minPrintTime = est + BUFFER_TIME_START;
    if (minPrintTime > m_nextPrintTime) {
        m_nextPrintTime = minPrintTime;
    }
}

// Backpressure: block if host is too far ahead of MCU.
// Port of Klipper's ToolHead._check_pause()
void ToolHead::checkPause() {
    while (true) {
        double est = m_mcu.getClockSync().estimatedPrintTime();
        double pauseTime = m_nextPrintTime - est - BUFFER_TIME_HIGH;
        if (pauseTime <= 0.0) break;

        // Sleep for the calculated time, clamped to [5ms, 1s]
        int sleepMs = static_cast<int>(std::max(5.0, std::min(1000.0, pauseTime * 1000.0)));
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));

        // Process incoming MCU responses while waiting (keeps clock sync alive)
        m_mcu.processIncoming(0);
    }
    // Update check threshold
    m_needCheckPause = m_nextPrintTime;
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
    // Uses Klipper's compress_bisect_add algorithm for optimal batch sizes.
    // max_error: 25 microseconds (matches Klipper's MAX_STEPCOMPRESS_ERROR)

    int64_t lastStepClock = doReset ? tmStartClock : stepper->getLastStepClock();
    uint32_t maxError = static_cast<uint32_t>(0.000025 * mcuFreq);
    static constexpr uint32_t CLOCK_DIFF_MAX = 3U << 28;

    int pos = 0;
    while (pos < numSteps) {
        // Check for step far from lastStepClock (> ~2.7 seconds)
        int64_t clockDiff = stepClocks[pos] - lastStepClock;
        if (clockDiff <= 0) {
            pos++;
            continue;
        }
        if (static_cast<uint64_t>(clockDiff) >= CLOCK_DIFF_MAX) {
            stepper->queueStep(static_cast<uint32_t>(clockDiff), 1, 0);
            lastStepClock = stepClocks[pos];
            pos++;
            continue;
        }

        StepMove move = sc_compress_bisect_add(stepClocks.data(), pos, numSteps,
                                               lastStepClock, maxError);

        stepper->queueStep(move.interval, move.count, move.add);

        // Update lastStepClock using the model's total advance
        // last_clock = lastStepClock + interval*count + add*count*(count-1)/2
        int64_t totalTicks = (int64_t)move.interval * move.count
                           + (int64_t)move.add * ((int64_t)move.count * (move.count - 1) / 2);
        lastStepClock += totalTicks;
        pos += move.count;
    }

    // Track where the stepper's clock ended up (for next TrapMove continuation)
    stepper->setLastStepClock(lastStepClock);
}

// ========== Input Shaper: Shaped Step Generation ==========

// Get axis position at any absolute print time across a TrapMove sequence.
// Handles times before the first move or after the last move by clamping.
double ToolHead::getAxisPositionAtTime(int axis, const std::vector<TrapMove>& moves,
                                       double printTime) {
    if (moves.empty()) return 0.0;

    // Before first move: return start position of first move
    if (printTime <= moves.front().print_time) {
        switch (axis) {
            case 0: return moves.front().start_pos.x;
            case 1: return moves.front().start_pos.y;
            case 2: return moves.front().start_pos.z;
            default: return 0.0;
        }
    }

    // Find the TrapMove containing this time (linear search from end for efficiency
    // since shaped lookups are mostly near current time)
    for (int i = static_cast<int>(moves.size()) - 1; i >= 0; i--) {
        const auto& tm = moves[i];
        if (printTime >= tm.print_time) {
            double dt = printTime - tm.print_time;
            if (dt > tm.move_t) dt = tm.move_t;  // clamp to move duration
            double dist = tm.start_v * dt + tm.half_accel * dt * dt;
            double axisR = (axis == 0) ? tm.axes_r.x : (axis == 1) ? tm.axes_r.y : tm.axes_r.z;
            double startPos = (axis == 0) ? tm.start_pos.x : (axis == 1) ? tm.start_pos.y : tm.start_pos.z;
            return startPos + axisR * dist;
        }
    }

    // Fallback: return start position of first move
    switch (axis) {
        case 0: return moves.front().start_pos.x;
        case 1: return moves.front().start_pos.y;
        case 2: return moves.front().start_pos.z;
        default: return 0.0;
    }
}

// Generate steps for a shaped axis using secant/bisection method.
// Port of Klipper's itersolve_gen_steps_range with input shaper convolution.
void ToolHead::generateShapedAxisSteps(int axis, const std::vector<TrapMove>& moves) {
    MCU_stepper* stepper = m_steppers[axis];
    if (!stepper) return;

    const auto& pulses = m_inputShaper.getAxisPulses(axis);
    if (pulses.empty()) return;

    double stepDist = stepper->getStepDist();
    if (stepDist <= 0) return;
    double halfStep = 0.5 * stepDist;

    double mcuFreq = m_mcu.getClockSync().getMcuFreq();

    double moveStart = moves.front().print_time;
    double moveEnd = moves.back().print_time + moves.back().move_t;

    // Lambda: compute shaped position at a given absolute print_time
    auto shapedPos = [&](double pt) -> double {
        double sum = 0.0;
        for (const auto& p : pulses) {
            sum += p.a * getAxisPositionAtTime(axis, moves, pt + p.t);
        }
        return sum;
    };

    // Initial shaped position
    double commandedPos = shapedPos(moveStart);
    double endPos = shapedPos(moveEnd);

    // If barely any movement, skip
    if (std::abs(endPos - commandedPos) < halfStep * 0.5) return;

    // Secant/bisection iterative solver (matching Klipper's itersolve)
    static constexpr double SEEK_TIME_RESET = 0.000100;

    bool sdir = (endPos > commandedPos);
    double target = commandedPos + (sdir ? halfStep : -halfStep);

    struct TimePos { double time, pos; };
    TimePos oldGuess = {moveStart, commandedPos};
    TimePos guess = oldGuess;

    bool haveBracket = false, isDirChange = false, checkOscillate = false;
    double lastTime = moveStart;
    double lowTime = moveStart;
    double highTime = moveStart + SEEK_TIME_RESET;
    if (highTime > moveEnd) highTime = moveEnd;

    // Collect step times as (clock, direction) pairs
    struct StepEvent {
        int64_t clock;
        bool forward;
    };
    std::vector<StepEvent> stepEvents;

    for (;;) {
        double guessDist = guess.pos - target;
        double ogDist = oldGuess.pos - target;

        // Secant method to guess next time
        double nextTime;
        double denom = guessDist - ogDist;
        if (std::abs(denom) > 1e-20) {
            nextTime = (oldGuess.time * guessDist - guess.time * ogDist) / denom;
        } else {
            nextTime = highTime; // fallback
        }

        if (!(nextTime > lowTime && nextTime < highTime)) {
            // Out of bounds - use fallback
            if (haveBracket) {
                // Bisection fallback
                nextTime = (lowTime + highTime) * 0.5;
                checkOscillate = false;
            } else if (guess.time >= moveEnd) {
                // No more steps in this time range
                break;
            } else {
                // Exponential search forward
                nextTime = highTime;
                highTime = 2.0 * highTime - lastTime;
                if (highTime > moveEnd) highTime = moveEnd;
            }
        }

        // Evaluate position at nextTime
        oldGuess = guess;
        guess.time = nextTime;
        guess.pos = shapedPos(nextTime);
        guessDist = guess.pos - target;

        if (std::abs(guessDist) > 1e-9) {
            double relDist = sdir ? guessDist : -guessDist;

            if (relDist > 0.0) {
                // Found position past target — step is present
                if (haveBracket && oldGuess.time <= lowTime) {
                    if (checkOscillate)
                        oldGuess = guess;
                    checkOscillate = true;
                }
                highTime = guess.time;
                haveBracket = true;
            } else if (relDist < -(halfStep + halfStep + 1e-8)) {
                // Direction change detected
                sdir = !sdir;
                target = sdir ? target + halfStep + halfStep
                              : target - halfStep - halfStep;
                lowTime = lastTime;
                highTime = guess.time;
                isDirChange = haveBracket = true;
                checkOscillate = false;
            } else {
                lowTime = guess.time;
            }

            if (!haveBracket || highTime - lowTime > 1e-9) {
                continue;
            }
        }

        // Found a step!
        int64_t stepClock = m_mcu.getClockSync().printTimeToClock(guess.time);
        stepEvents.push_back({stepClock, sdir});

        // Advance target to next step
        target = sdir ? target + halfStep + halfStep
                      : target - halfStep - halfStep;

        // Reset bounds for next search
        double seekDelta = 1.5 * (guess.time - lastTime);
        if (seekDelta < 1e-9) seekDelta = 1e-9;
        if (isDirChange && seekDelta > SEEK_TIME_RESET)
            seekDelta = SEEK_TIME_RESET;
        lastTime = lowTime = guess.time;
        highTime = guess.time + seekDelta;
        if (highTime > moveEnd) highTime = moveEnd;
        isDirChange = haveBracket = checkOscillate = false;
    }

    if (stepEvents.empty()) return;

    // Reset step clock at start
    int64_t tmStartClock = m_mcu.getClockSync().printTimeToClock(moveStart);
    stepper->resetStepClock(tmStartClock);

    // Group consecutive same-direction steps and compress each batch
    uint32_t maxError = static_cast<uint32_t>(0.000025 * mcuFreq);
    static constexpr uint32_t CLOCK_DIFF_MAX = 3U << 28;

    size_t batchStart = 0;
    while (batchStart < stepEvents.size()) {
        bool batchDir = stepEvents[batchStart].forward;

        // Find end of same-direction batch
        size_t batchEnd = batchStart + 1;
        while (batchEnd < stepEvents.size() && stepEvents[batchEnd].forward == batchDir) {
            batchEnd++;
        }

        // Set direction for this batch
        stepper->setNextStepDir(batchDir);

        // Extract step clocks for this batch
        int numSteps = static_cast<int>(batchEnd - batchStart);
        std::vector<int64_t> batchClocks(numSteps);
        for (int i = 0; i < numSteps; i++) {
            batchClocks[i] = stepEvents[batchStart + i].clock;
        }

        // Compress and send
        int64_t lastStepClock = stepper->getLastStepClock();
        int pos = 0;
        while (pos < numSteps) {
            int64_t clockDiff = batchClocks[pos] - lastStepClock;
            if (clockDiff <= 0) {
                pos++;
                continue;
            }
            if (static_cast<uint64_t>(clockDiff) >= CLOCK_DIFF_MAX) {
                stepper->queueStep(static_cast<uint32_t>(clockDiff), 1, 0);
                lastStepClock = batchClocks[pos];
                pos++;
                continue;
            }

            StepMove move = sc_compress_bisect_add(batchClocks.data(), pos, numSteps,
                                                   lastStepClock, maxError);

            stepper->queueStep(move.interval, move.count, move.add);

            int64_t totalTicks = (int64_t)move.interval * move.count
                + (int64_t)move.add * ((int64_t)move.count * (move.count - 1) / 2);
            lastStepClock += totalTicks;
            pos += move.count;
        }

        stepper->setLastStepClock(lastStepClock);
        batchStart = batchEnd;
    }
}

bool ToolHead::generateSteps() {
    auto trapMoves = m_trapq.getAndClear();
    if (trapMoves.empty()) return true;

    // Check which axes use input shaping
    bool shaped[3] = {false, false, false};
    for (int axis = 0; axis < 3; ++axis) {
        shaped[axis] = m_inputShaper.isAxisShaped(axis);
    }

    // For shaped axes: use iterative solver across all TrapMoves at once
    for (int axis = 0; axis < 3; ++axis) {
        if (shaped[axis] && m_steppers[axis]) {
            generateShapedAxisSteps(axis, trapMoves);
        }
    }

    // For unshaped axes: collect ALL step times across all TrapMoves, then
    // compress once. This is critical for serial throughput — compressing
    // per-TrapMove produces 10-100x more queue_step commands than needed
    // because consecutive arc segments have similar step spacing that the
    // compressor can merge into single (interval, count, add) triples.
    for (int axis = 0; axis < 3; ++axis) {
        if (shaped[axis]) continue;
        MCU_stepper* stepper = m_steppers[axis];
        if (!stepper) continue;

        double stepDist = stepper->getStepDist();
        if (stepDist <= 0) continue;
        double mcuFreq = m_mcu.getClockSync().getMcuFreq();

        // Collect all step events (clock + direction) across all TrapMoves
        struct StepEvent { int64_t clock; bool forward; };
        std::vector<StepEvent> allSteps;

        for (const auto& tm : trapMoves) {
            double axisR = 0;
            switch (axis) {
                case 0: axisR = tm.axes_r.x; break;
                case 1: axisR = tm.axes_r.y; break;
                case 2: axisR = tm.axes_r.z; break;
            }
            if (std::abs(axisR) < 0.000000001) continue;

            double v0 = std::abs(axisR) * tm.start_v;
            double accel = std::abs(axisR) * 2.0 * tm.half_accel;
            double totalDist = v0 * tm.move_t + 0.5 * accel * tm.move_t * tm.move_t;
            if (totalDist < stepDist * 0.5) continue;

            int numSteps = static_cast<int>(totalDist / stepDist + 0.5);
            if (numSteps <= 0) continue;

            bool forward = (axisR > 0);
            int64_t tmStartClock = m_mcu.getClockSync().printTimeToClock(tm.print_time);

            if (std::abs(accel) < 1e-6) {
                double invV = 1.0 / std::max(v0, 1e-6);
                for (int i = 0; i < numSteps; i++) {
                    double t = (i + 1) * stepDist * invV;
                    allSteps.push_back({
                        tmStartClock + static_cast<int64_t>(t * mcuFreq + 0.5),
                        forward
                    });
                }
            } else {
                double v0sq = v0 * v0;
                double inv_a = 1.0 / accel;
                for (int i = 0; i < numSteps; i++) {
                    double pos = (i + 1) * stepDist;
                    double disc = v0sq + 2.0 * accel * pos;
                    if (disc < 0) break;
                    double t = (-v0 + std::sqrt(disc)) * inv_a;
                    allSteps.push_back({
                        tmStartClock + static_cast<int64_t>(t * mcuFreq + 0.5),
                        forward
                    });
                }
            }
        }

        if (allSteps.empty()) continue;

        // Reset step clock at start of first step batch
        int64_t firstClock = allSteps.front().clock;
        if (!stepper->isClockInitialized()) {
            stepper->resetStepClock(firstClock);
        } else {
            int64_t gap = firstClock - stepper->getLastStepClock();
            if (gap <= 0 || gap > 4'000'000'000LL) {
                stepper->resetStepClock(firstClock);
            }
        }

        // Compress and send: group by direction, then compress each batch.
        // This matches Klipper's steppersync approach of collecting steps
        // then compressing across TrapMove boundaries.
        uint32_t maxError = static_cast<uint32_t>(0.000025 * mcuFreq);
        static constexpr uint32_t CLOCK_DIFF_MAX = 3U << 28;

        size_t batchStart = 0;
        while (batchStart < allSteps.size()) {
            bool batchDir = allSteps[batchStart].forward;

            size_t batchEnd = batchStart + 1;
            while (batchEnd < allSteps.size() && allSteps[batchEnd].forward == batchDir) {
                batchEnd++;
            }

            stepper->setNextStepDir(batchDir);

            int numSteps = static_cast<int>(batchEnd - batchStart);
            std::vector<int64_t> batchClocks(numSteps);
            for (int i = 0; i < numSteps; i++) {
                batchClocks[i] = allSteps[batchStart + i].clock;
            }

            int64_t lastStepClock = stepper->getLastStepClock();
            int pos = 0;
            while (pos < numSteps) {
                int64_t clockDiff = batchClocks[pos] - lastStepClock;
                if (clockDiff <= 0) {
                    pos++;
                    continue;
                }
                if (static_cast<uint64_t>(clockDiff) >= CLOCK_DIFF_MAX) {
                    stepper->queueStep(static_cast<uint32_t>(clockDiff), 1, 0);
                    lastStepClock = batchClocks[pos];
                    pos++;
                    continue;
                }

                StepMove move = sc_compress_bisect_add(batchClocks.data(), pos, numSteps,
                                                       lastStepClock, maxError);
                stepper->queueStep(move.interval, move.count, move.add);

                int64_t totalTicks = (int64_t)move.interval * move.count
                    + (int64_t)move.add * ((int64_t)move.count * (move.count - 1) / 2);
                lastStepClock += totalTicks;
                pos += move.count;
            }

            stepper->setLastStepClock(lastStepClock);
            batchStart = batchEnd;
        }
    }

    return true;
}
