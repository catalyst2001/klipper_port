#include "stepper.h"
#include "klipper_mcu.h"

#include <sstream>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

static constexpr int64_t MAX_SCHEDULE_TICKS = (1LL << 31) - 1;

// ========== MCU_stepper ==========

MCU_stepper::MCU_stepper(KlipperMCU& mcu) : m_mcu(mcu) {}

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

    // step_pulse_ticks: use MCU constant if not set
    if (m_stepPulseTicks <= 0) {
        m_stepPulseTicks = m_mcu.getConstantInt("STEPPER_BOTH_EDGE", 0) ? 0 : 2;
    }

    // Enable both-edge stepping if MCU supports it (STEPPER_STEP_BOTH_EDGE=1).
    // This uses the optimized stepper_event_edge() path in the MCU firmware,
    // which halves the number of timer events and avoids the
    // "Stepper too far in past" safety check in stepper_event_full().
    // Klipper Python does the same in MCU_stepper._build_config().
    int invertStep = 0;
    bool hasBothEdge = m_mcu.getConstantInt("STEPPER_STEP_BOTH_EDGE", 0) != 0;
    if (hasBothEdge && m_stepPulseTicks == 0) {
        invertStep = -1;  // SF_SINGLE_SCHED → enables optimized edge path
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

bool MCU_stepper::setNextStepDir(bool forward) {
    bool dir = forward ^ m_invertDir;
    m_curDir = forward;
    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"dir", dir ? 1 : 0}
    };
    return m_mcu.sendCommand("set_next_step_dir", params);
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
    trCfg << "trsync_start oid=" << m_trsyncOid
          << " report_clock=0 report_ticks=0 expire_reason=0";

    return true;
}

bool MCU_endstop::home(int64_t homeClock, double sampleTime, int sampleCount,
                        double restTime, bool pinValue,
                        const std::vector<MCU_stepper*>& steppers,
                        double triggerClearTime) {
    int64_t sampleTicks = m_mcu.secondsToClock(sampleTime);
    int64_t restTicks = m_mcu.secondsToClock(restTime);

    // Set up trsync: configure synchronized trigger
    // trsync_start
    {
        int64_t reportClock = homeClock;
        int64_t reportTicks = m_mcu.secondsToClock(0.1);
        std::map<std::string, int64_t> params = {
            {"oid", m_trsyncOid},
            {"report_clock", static_cast<int64_t>(static_cast<uint32_t>(reportClock))},
            {"report_ticks", reportTicks},
            {"expire_reason", 0}
        };
        if (!m_mcu.sendCommand("trsync_start", params)) return false;
    }

    // Set steppers to use trsync
    for (auto* stepper : steppers) {
        std::map<std::string, int64_t> params = {
            {"oid", m_trsyncOid},
            {"stepper_oid", stepper->getOid()},
            {"set", 1}
        };
        if (!m_mcu.sendCommand("trsync_set_timeout", params)) return false;
    }

    // endstop_home
    {
        std::map<std::string, int64_t> params = {
            {"oid", m_oid},
            {"clock", static_cast<int64_t>(static_cast<uint32_t>(homeClock))},
            {"sample_ticks", sampleTicks},
            {"sample_count", sampleCount},
            {"rest_ticks", restTicks},
            {"pin_value", pinValue ? 1 : 0},
            {"trsync_oid", m_trsyncOid},
            {"trigger_reason", 1}
        };
        if (!m_mcu.sendCommand("endstop_home", params)) return false;
    }

    // Wait for trsync completion (poll for trsync_state response)
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool triggered = false;
    while (std::chrono::steady_clock::now() < deadline) {
        auto responses = m_mcu.processIncoming(100);
        for (auto& resp : responses) {
            if (resp.name == "trsync_state") {
                auto canIt = resp.intParams.find("can_trigger");
                auto trIt = resp.intParams.find("trigger_reason");
                if (canIt != resp.intParams.end() && canIt->second == 0) {
                    // Trigger completed
                    if (trIt != resp.intParams.end() && trIt->second == 1) {
                        triggered = true;
                    }
                    goto done;
                }
            }
        }
    }
done:
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

    // Phase 1: Fast home
    {
        double printTime = clockSync.estimatedPrintTime() + 0.1;
        int64_t homeClock = clockSync.printTimeToClock(printTime);

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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        double printTime = clockSync.estimatedPrintTime() + 0.1;
        int64_t startClock = clockSync.printTimeToClock(printTime);

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
    {
        double printTime = clockSync.estimatedPrintTime() + 0.1;
        int64_t homeClock = clockSync.printTimeToClock(printTime);

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
