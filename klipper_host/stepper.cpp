#include "stepper.h"
#include "klipper_mcu.h"

#include <sstream>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include <climits>
#include <mutex>
#include <condition_variable>

static constexpr int64_t MAX_SCHEDULE_TICKS = (1LL << 31) - 1;

// ========== MCU_stepper ==========

MCU_stepper::MCU_stepper(KlipperMCU& mcu) : m_mcu(mcu) {}

int64_t MCU_stepper::syncPosition(int timeoutMs) {
    std::map<std::string, int64_t> outParams;
    std::map<std::string, std::vector<uint8_t>> outBuf;
    std::map<std::string, int64_t> params = {{"oid", m_oid}};
    if (!m_mcu.sendWithResponse("stepper_get_position", "stepper_position",
                                 outParams, outBuf, params, {}, timeoutMs)) {
        std::cerr << "[Stepper] syncPosition timeout oid=" << m_oid << std::endl;
        return INT64_MIN;
    }
    auto it = outParams.find("pos");
    return (it != outParams.end()) ? it->second : INT64_MIN;
}

void MCU_stepper::setupPin(const std::string& stepPin, const std::string& dirPin) {
    m_stepPin = stepPin;
    m_dirPin = dirPin;
}

void MCU_stepper::setupStepPulseTicks(int stepPulseTicks) {
    m_stepPulseTicks = stepPulseTicks;
}

void MCU_stepper::setupInvertDir(bool invert) {
    m_invertDir = invert;
}

void MCU_stepper::setupStepDist(double rotationDist, int fullSteps, int microsteps,
                                 double gearRatio) {
    if (gearRatio <= 0.0) gearRatio = 1.0;
    m_stepDist = rotationDist / (fullSteps * microsteps * gearRatio);
}

bool MCU_stepper::buildConfig() {
    m_mcu.requestMoveQueueSlot();
    m_oid = m_mcu.createOid();

    int stepPinNum = m_mcu.resolvePin(m_stepPin);
    int dirPinNum = m_mcu.resolvePin(m_dirPin);
    if (stepPinNum < 0 || dirPinNum < 0) return false;

    // step_pulse_ticks: compute from MCU clock frequency, matching Python Klipper.
    // Default pulse duration is 2µs. If both-edge stepping is available AND the
    // pulse duration is short enough, enable it for the optimized MCU path.
    double mcuFreq = static_cast<double>(m_mcu.getConstantInt("CLOCK_FREQ", 300000000));
    constexpr double DEFAULT_PULSE_DURATION = 0.000002; // 2µs
    constexpr double MIN_BOTH_EDGE_DURATION = 0.000000500; // 500ns

    if (m_stepPulseTicks <= 0) {
        m_stepPulseTicks = (std::max)(1, static_cast<int>(mcuFreq * DEFAULT_PULSE_DURATION + 0.5));
    }

    // Check both-edge support. MCU firmware reports either STEPPER_STEP_BOTH_EDGE
    // (new) or STEPPER_BOTH_EDGE (old). Python checks both.
    int ssbe = m_mcu.getConstantInt("STEPPER_STEP_BOTH_EDGE", 0);
    int sbe = m_mcu.getConstantInt("STEPPER_BOTH_EDGE", 0);

    int invertStep = 0;
    bool wantBothEdge = (ssbe || sbe) && (DEFAULT_PULSE_DURATION <= MIN_BOTH_EDGE_DURATION);
    if (wantBothEdge) {
        invertStep = -1; // SF_SINGLE_SCHED → enables edge-optimized path
        if (sbe) {
            m_stepPulseTicks = 0; // old MCUs: zero pulse ticks for both-edge
        }
    }

    std::ostringstream cfg;
    cfg << "config_stepper oid=" << m_oid
        << " step_pin=" << stepPinNum
        << " dir_pin=" << dirPinNum
        << " invert_step=" << invertStep
        << " step_pulse_ticks=" << m_stepPulseTicks;
    m_mcu.addConfigCmd(cfg.str());

    return true;
}

bool MCU_stepper::queueStep(int64_t interval, int64_t count, int64_t add) {
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"interval", interval},
        {"count", count},
        {"add", add}
    };
    return m_mcu.sendCommand("queue_step", params);
}

bool MCU_stepper::queueStepBatched(int64_t interval, int64_t count, int64_t add) {
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"interval", interval},
        {"count", count},
        {"add", add}
    };
    auto payload = m_mcu.encodeCommandPayload("queue_step", params);
    if (payload.empty()) return false;
    return m_mcu.queuePayload(payload);
}

bool MCU_stepper::setNextStepDir(bool forward) {
    bool dir = forward ^ m_invertDir;
    m_curDir = forward;
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"dir", dir ? 1 : 0}
    };
    return m_mcu.sendCommand("set_next_step_dir", params);
}

bool MCU_stepper::setNextStepDirBatched(bool forward) {
    bool dir = forward ^ m_invertDir;
    m_curDir = forward;
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"dir", dir ? 1 : 0}
    };
    auto payload = m_mcu.encodeCommandPayload("set_next_step_dir", params);
    if (payload.empty()) return false;
    return m_mcu.queuePayload(payload);
}

bool MCU_stepper::resetStepClock(int64_t clock) {
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"clock", static_cast<int64_t>(static_cast<uint32_t>(clock))}
    };
    bool ok = m_mcu.sendCommand("reset_step_clock", params);
    if (ok) {
        m_lastStepClock = clock;
        m_clockInitialized = true;
    }
    return ok;
}

bool MCU_stepper::resetStepClockBatched(int64_t clock) {
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"clock", static_cast<int64_t>(static_cast<uint32_t>(clock))}
    };
    auto payload = m_mcu.encodeCommandPayload("reset_step_clock", params);
    if (payload.empty()) return false;
    if (!m_mcu.queuePayload(payload)) return false;
    m_lastStepClock = clock;
    m_clockInitialized = true;
    return true;
}

// ---- SerialQueue timed methods ----

void MCU_stepper::queueStepTimed(int64_t interval, int64_t count, int64_t add,
                                  uint64_t min_clock, uint64_t req_clock) {
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"interval", interval},
        {"count", count},
        {"add", add}
    };
    auto payload = m_mcu.encodeCommandPayload("queue_step", params);
    if (payload.empty()) return;
    // Lazily allocate per-stepper command queue
    if (!m_cmdQueue)
        m_cmdQueue = m_mcu.getSerialQueue().allocCommandQueue();
    m_mcu.sendTimedRaw(payload.data(), static_cast<int>(payload.size()),
                       min_clock, req_clock, m_cmdQueue);
}

void MCU_stepper::setNextStepDirTimed(bool forward, uint64_t min_clock,
                                       uint64_t req_clock) {
    bool dir = forward ^ m_invertDir;
    m_curDir = forward;
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"dir", dir ? 1 : 0}
    };
    auto payload = m_mcu.encodeCommandPayload("set_next_step_dir", params);
    if (payload.empty()) return;
    if (!m_cmdQueue)
        m_cmdQueue = m_mcu.getSerialQueue().allocCommandQueue();
    m_mcu.sendTimedRaw(payload.data(), static_cast<int>(payload.size()),
                       min_clock, req_clock, m_cmdQueue);
}

void MCU_stepper::resetStepClockTimed(int64_t clock, uint64_t min_clock,
                                       uint64_t req_clock) {
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"clock", static_cast<int64_t>(static_cast<uint32_t>(clock))}
    };
    auto payload = m_mcu.encodeCommandPayload("reset_step_clock", params);
    if (payload.empty()) return;
    if (!m_cmdQueue)
        m_cmdQueue = m_mcu.getSerialQueue().allocCommandQueue();
    m_mcu.sendTimedRaw(payload.data(), static_cast<int>(payload.size()),
                       min_clock, req_clock, m_cmdQueue);
    m_lastStepClock = clock;
    m_clockInitialized = true;
}

// ========== MCU_endstop ==========

MCU_endstop::MCU_endstop(KlipperMCU& mcu) : m_mcu(mcu) {}

void MCU_endstop::setupPin(const std::string& pin, bool pullUp) {
    m_pin = pin;
    m_pullUp = pullUp;
}

bool MCU_endstop::buildConfig() {
    m_oid = m_mcu.createOid();
    m_trsyncOid = m_mcu.createOid();

    int pinNum = m_mcu.resolvePin(m_pin);
    if (pinNum < 0) return false;

    // config_endstop
    std::ostringstream cfg;
    cfg << "config_endstop oid=" << m_oid
        << " pin=" << pinNum
        << " pull_up=" << (m_pullUp ? 1 : 0)
        << " stepper_count=0";
    m_mcu.addConfigCmd(cfg.str());

    // config_trsync for synchronized homing
    std::ostringstream trCfg;
    trCfg << "config_trsync oid=" << m_trsyncOid;
    m_mcu.addConfigCmd(trCfg.str());

    return true;
}

bool MCU_endstop::home(int64_t homeClock, double sampleTime, int sampleCount,
                        double restTime, bool pinValue,
                        const std::vector<MCU_stepper*>& steppers,
                        double triggerClearTime) {
    int64_t sampleTicks = m_mcu.secondsToClock(sampleTime);
    int64_t restTicks = m_mcu.secondsToClock(restTime);

    // Choose between raw serial (pre-SerialQueue) and SQ-based command path
    const bool useSQ = m_mcu.isSerialQueueActive();

    // Helper: send an immediate command (via SQ or raw serial)
    auto sendImmediate = [&](const std::string& cmdName,
                             const std::map<std::string, int64_t>& params) -> bool {
        if (useSQ) {
            m_mcu.sendTimed(cmdName, params, 0, 0, nullptr);
            return true;
        } else {
            return m_mcu.sendCommand(cmdName, params);
        }
    };

    // 1. trsync_start: begin synchronized trigger monitoring
    double expireTimeout = 5.0;
    {
        int64_t expireTicks = m_mcu.secondsToClock(expireTimeout);
        int64_t reportTicks = m_mcu.secondsToClock(expireTimeout * 0.3);
        int64_t reportClock = homeClock + reportTicks;
        std::map<std::string, int64_t> params = {
            {"oid", m_trsyncOid},
            {"report_clock", static_cast<int64_t>(static_cast<uint32_t>(reportClock))},
            {"report_ticks", reportTicks},
            {"expire_reason", 4}  // REASON_COMMS_TIMEOUT
        };
        if (!sendImmediate("trsync_start", params)) return false;
    }

    // 2. stepper_stop_on_trigger: register each stepper to stop on trsync trigger
    for (auto* stepper : steppers) {
        std::map<std::string, int64_t> params = {
            {"oid", stepper->getOid()},
            {"trsync_oid", m_trsyncOid}
        };
        if (!sendImmediate("stepper_stop_on_trigger", params)) return false;
    }

    // 3. trsync_set_timeout: set expiration deadline
    {
        int64_t expireClock = homeClock + m_mcu.secondsToClock(expireTimeout);
        std::map<std::string, int64_t> params = {
            {"oid", m_trsyncOid},
            {"clock", static_cast<int64_t>(static_cast<uint32_t>(expireClock))}
        };
        if (!sendImmediate("trsync_set_timeout", params)) return false;
    }

    // 4. endstop_home: start endstop monitoring
    {
        std::map<std::string, int64_t> params = {
            {"oid", m_oid},
            {"clock", static_cast<int64_t>(static_cast<uint32_t>(homeClock))},
            {"sample_ticks", sampleTicks},
            {"sample_count", sampleCount},
            {"rest_ticks", restTicks},
            {"pin_value", pinValue ? 1 : 0},
            {"trsync_oid", m_trsyncOid},
            {"trigger_reason", 1}  // REASON_ENDSTOP_HIT
        };
        if (!sendImmediate("endstop_home", params)) return false;
    }

    // Wait for trsync completion
    bool triggered = false;

    if (useSQ) {
        // SerialQueue mode: register OID handler and wait via condition variable.
        // The SQ receive callback (running on the SQ thread) will dispatch
        // trsync_state to our handler, which signals the CV.
        std::mutex waitMutex;
        std::condition_variable waitCV;
        bool done = false;
        int triggerReason = 0;

        m_mcu.registerOidResponse("trsync_state", m_trsyncOid,
            [&](const KlipperMCU::ParsedResponse& resp) {
                auto canIt = resp.intParams.find("can_trigger");
                auto trIt = resp.intParams.find("trigger_reason");
                if (canIt != resp.intParams.end() && canIt->second == 0) {
                    std::lock_guard<std::mutex> lk(waitMutex);
                    if (trIt != resp.intParams.end())
                        triggerReason = static_cast<int>(trIt->second);
                    done = true;
                    waitCV.notify_one();
                }
            });

        {
            std::unique_lock<std::mutex> lk(waitMutex);
            waitCV.wait_for(lk, std::chrono::seconds(6), [&] { return done; });
        }

        // Unregister handler
        m_mcu.registerOidResponse("trsync_state", m_trsyncOid, nullptr);

        triggered = (triggerReason == 1);  // REASON_ENDSTOP_HIT
    } else {
        // Raw serial mode: poll for trsync_state via processIncoming
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            auto responses = m_mcu.processIncoming(100);
            for (auto& resp : responses) {
                if (resp.name == "trsync_state") {
                    auto oidIt = resp.intParams.find("oid");
                    if (oidIt == resp.intParams.end() || oidIt->second != m_trsyncOid)
                        continue;
                    auto canIt = resp.intParams.find("can_trigger");
                    auto trIt = resp.intParams.find("trigger_reason");
                    if (canIt != resp.intParams.end() && canIt->second == 0) {
                        if (trIt != resp.intParams.end() && trIt->second == 1) {
                            triggered = true;
                        }
                        goto done;
                    }
                }
            }
        }
    done:;
    }

    return triggered;
}

bool MCU_endstop::queryState(bool& triggered) {
    std::map<std::string, int64_t> outParams;
    std::map<std::string, std::vector<uint8_t>> bufP;
    std::map<std::string, int64_t> inParams = {{"oid", m_oid}};

    if (!m_mcu.sendWithResponse("endstop_query_state", "endstop_state",
                                 outParams, bufP, inParams)) {
        return false;
    }

    auto it = outParams.find("pin_value");
    if (it != outParams.end()) {
        triggered = (it->second != 0);
        return true;
    }
    return false;
}

// ========== PrinterRail ==========

PrinterRail::PrinterRail(MCU_stepper& stepper, MCU_endstop& endstop)
    : m_stepper(stepper), m_endstop(endstop) {}

void PrinterRail::setPositionLimits(double posMin, double posMax) {
    m_posMin = posMin;
    m_posMax = posMax;
}

void PrinterRail::setHomingSpeed(double speed) {
    m_homingSpeed = speed;
}

void PrinterRail::setHomingRetractDist(double dist) {
    m_homingRetractDist = dist;
}

void PrinterRail::setSecondHomingSpeed(double speed) {
    m_secondHomingSpeed = speed;
}

void PrinterRail::setPositionEndstop(double pos) {
    m_posEndstop = pos;
}

bool PrinterRail::homeAxis(KlipperMCU& mcu) {
    // Homing uses a 3-phase approach:
    // 1. Fast move toward endstop (with accel ramp)
    // 2. Retract (with accel ramp)
    // 3. Slow move toward endstop (with accel ramp)

    double stepDist = m_stepper.getStepDist();
    if (stepDist <= 0.0) return false;

    auto& clockSync = mcu.getClockSync();
    const int64_t settleTicks = static_cast<int64_t>(0.1 * clockSync.getMcuFreq());

    // Phase 1: Fast home
    {
        // During direct-path homing, schedule relative to the current MCU clock.
        // This avoids print_time -> clock ambiguity across 32-bit wrap windows.
        int64_t homeClock = clockSync.getClock() + settleTicks;

        // Set direction toward endstop
        bool dir = (m_posEndstop <= m_posMin);  // toward min = forward
        m_stepper.setNextStepDir(dir);
        m_stepper.resetStepClock(homeClock);

        // Max steps for the axis range
        int64_t maxSteps = static_cast<int64_t>(
            (m_posMax - m_posMin) / stepDist) + 100;

        // Accelerating ramp + cruise
        queueHomingSteps(mcu, m_homingSpeed, maxSteps, homeClock);

        // Home endstop
        std::vector<MCU_stepper*> steppers = {&m_stepper};
        bool triggered = m_endstop.home(homeClock, 0.000015, 4, 0.01, true,
                                         steppers);
        if (!triggered) {
            std::cout << "[PrinterRail] Fast home: endstop not triggered" << std::endl;
            return false;
        }
    }

    // Phase 2: Retract
    // Sync barrier: trsync already stopped the stepper (count=0, moves freed).
    // Round-trip to MCU confirms all prior state changes are settled.
    m_stepper.syncPosition();
    {
        int64_t startClock = clockSync.getClock() + settleTicks;

        // Reverse direction
        bool dir = (m_posEndstop > m_posMin);
        m_stepper.setNextStepDir(dir);
        m_stepper.resetStepClock(startClock);

        int64_t retractSteps = static_cast<int64_t>(
            m_homingRetractDist / stepDist);

        // Accelerating ramp + cruise (for retract)
        queueHomingSteps(mcu, m_homingSpeed, retractSteps, startClock);

        // Wait for retract to complete
        double retractTime = (retractSteps * stepDist) / m_homingSpeed;
        // Add extra time for the acceleration ramp
        double accelTime = m_homingSpeed / m_homingAccel;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>((retractTime + accelTime + 0.2) * 1000)));
    }

    // Phase 3: Slow home
    // Sync barrier: confirm MCU finished executing retract steps.
    m_stepper.syncPosition();
    {
        int64_t homeClock = clockSync.getClock() + settleTicks;

        bool dir = (m_posEndstop <= m_posMin);
        m_stepper.setNextStepDir(dir);
        m_stepper.resetStepClock(homeClock);

        int64_t maxSteps = static_cast<int64_t>(
            (m_homingRetractDist * 2.0) / stepDist) + 100;

        // Accelerating ramp + cruise (slow speed)
        queueHomingSteps(mcu, m_secondHomingSpeed, maxSteps, homeClock);

        std::vector<MCU_stepper*> steppers = {&m_stepper};
        bool triggered = m_endstop.home(homeClock, 0.000015, 4, 0.01, true,
                                         steppers);
        if (!triggered) {
            std::cout << "[PrinterRail] Slow home: endstop not triggered" << std::endl;
            return false;
        }
    }

    // Set position to endstop position
    m_stepper.setCommandedPosition(
        static_cast<int64_t>(m_posEndstop / stepDist));

    return true;
}

// Queue steps with acceleration ramp from rest to target speed, then cruise.
// Uses 2 queue_step commands: one for accel (with add), one for cruise.
void PrinterRail::queueHomingSteps(KlipperMCU& mcu, double speed, int64_t maxSteps,
                                    int64_t startClock) {
    double stepDist = m_stepper.getStepDist();
    double mcuFreq = mcu.getClockSync().getMcuFreq();
    double accel = m_homingAccel;

    // Acceleration distance: v^2 / (2*a)
    double accelDist = speed * speed / (2.0 * accel);
    int64_t accelSteps = static_cast<int64_t>(accelDist / stepDist);
    if (accelSteps < 2) accelSteps = 2;
    if (accelSteps > maxSteps - 1) accelSteps = maxSteps - 1;

    // First step interval (starting from near-zero velocity):
    // t1 = sqrt(2 * step_dist / accel), interval1 = t1 * mcu_freq
    double t1 = std::sqrt(2.0 * stepDist / accel);
    int64_t firstInterval = static_cast<int64_t>(t1 * mcuFreq);

    // Last step interval (at full speed):
    // interval_N = step_dist / speed * mcu_freq
    int64_t cruiseInterval = static_cast<int64_t>(mcuFreq * stepDist / speed);
    if (cruiseInterval < 1) cruiseInterval = 1;

    // Linear approximation of acceleration using add parameter:
    // interval_n = firstInterval + add * n
    // At step accelSteps-1: interval should ≈ cruiseInterval
    int64_t addVal = 0;
    if (accelSteps > 1) {
        addVal = (cruiseInterval - firstInterval) / (accelSteps - 1);
    }

    // Clamp add to int16_t range
    if (addVal < -32768) addVal = -32768;
    if (addVal > 32767) addVal = 32767;

    // Phase 1: Acceleration ramp
    m_stepper.queueStep(firstInterval, accelSteps, addVal);

    // Phase 2: Cruise at constant speed
    int64_t cruiseSteps = maxSteps - accelSteps;
    if (cruiseSteps > 0) {
        m_stepper.queueStep(cruiseInterval, cruiseSteps, 0);
    }
}
