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

    std::ostringstream cfg;
    cfg << "config_stepper oid=" << m_oid
        << " step_pin=" << stepPinNum
        << " dir_pin=" << dirPinNum
        << " invert_step=" << (m_invertDir ? -1 : 0)
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
    return m_mcu.sendCommand("reset_step_clock", params);
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
    // 1. Fast move toward endstop
    // 2. Retract
    // 3. Slow move toward endstop

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

        // Generate steps for fast approach
        double moveSpeed = m_homingSpeed;
        int64_t intervalTicks = static_cast<int64_t>(
            clockSync.getMcuFreq() * stepDist / moveSpeed);
        if (intervalTicks < 1) intervalTicks = 1;

        // Max steps for the axis range
        int64_t maxSteps = static_cast<int64_t>(
            (m_posMax - m_posMin) / stepDist) + 100;
        m_stepper.queueStep(intervalTicks, maxSteps, 0);

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

        double moveSpeed = m_homingSpeed;
        int64_t intervalTicks = static_cast<int64_t>(
            clockSync.getMcuFreq() * stepDist / moveSpeed);
        int64_t retractSteps = static_cast<int64_t>(
            m_homingRetractDist / stepDist);

        m_stepper.queueStep(intervalTicks, retractSteps, 0);

        // Wait for retract to complete
        double retractTime = (retractSteps * stepDist) / moveSpeed;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>((retractTime + 0.2) * 1000)));
    }

    // Phase 3: Slow home
    {
        double printTime = clockSync.estimatedPrintTime() + 0.1;
        int64_t homeClock = clockSync.printTimeToClock(printTime);

        bool dir = (m_posEndstop <= m_posMin);
        m_stepper.setNextStepDir(dir);
        m_stepper.resetStepClock(homeClock);

        double moveSpeed = m_secondHomingSpeed;
        int64_t intervalTicks = static_cast<int64_t>(
            clockSync.getMcuFreq() * stepDist / moveSpeed);
        int64_t maxSteps = static_cast<int64_t>(
            (m_homingRetractDist * 2.0) / stepDist) + 100;

        m_stepper.queueStep(intervalTicks, maxSteps, 0);

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
