#include "mcu_objects.h"
#include "klipper_mcu.h"

#include <algorithm>
#include <sstream>
#include <cmath>

static constexpr int64_t MAX_SCHEDULE_TICKS = (1LL << 31) - 1;

// ========== MCU_digital_out ==========

MCU_digital_out::MCU_digital_out(KlipperMCU& mcu) : m_mcu(mcu) {}

void MCU_digital_out::setupPin(const std::string& pinName, bool invert) {
    m_pinName = pinName;
    m_invert = invert;
    m_startValue = invert;
    m_shutdownValue = invert;
}

void MCU_digital_out::setupMaxDuration(double maxDuration) {
    m_maxDuration = maxDuration;
}

void MCU_digital_out::setupStartValue(bool startValue, bool shutdownValue) {
    m_startValue = startValue ^ m_invert;
    m_shutdownValue = shutdownValue ^ m_invert;
}

bool MCU_digital_out::buildConfig() {
    // Safety check: if max_duration > 0, start_value must equal shutdown_value
    if (m_maxDuration > 0.0 && m_startValue != m_shutdownValue) {
        return false;
    }

    int64_t mdurTicks = m_mcu.secondsToClock(m_maxDuration);
    if (mdurTicks > MAX_SCHEDULE_TICKS) mdurTicks = MAX_SCHEDULE_TICKS;

    m_mcu.requestMoveQueueSlot();
    m_oid = m_mcu.createOid();

    int pinNum = m_mcu.resolvePin(m_pinName);
    if (pinNum < 0) return false;

    // Config command
    std::ostringstream cfg;
    cfg << "config_digital_out oid=" << m_oid
        << " pin=" << pinNum
        << " value=" << (m_startValue ? 1 : 0)
        << " default_value=" << (m_shutdownValue ? 1 : 0)
        << " max_duration=" << mdurTicks;
    m_mcu.addConfigCmd(cfg.str());

    // Restart command: restore to start value (not shutdown value!)
    // On restart/reconnect, the pin should return to its working state.
    // E.g. stepper enable pin must stay active after restart.
    std::ostringstream rst;
    rst << "update_digital_out oid=" << m_oid
        << " value=" << (m_startValue ? 1 : 0);
    m_mcu.addRestartCmd(rst.str());

    m_lastValue = m_startValue;
    return true;
}

bool MCU_digital_out::setDigital(double printTime, bool value) {
    bool outVal = value ^ m_invert;
    int64_t clock = m_mcu.getClockSync().printTimeToClock(printTime);

    std::map<std::string, int64_t> params = {
        {"oid", m_oid},
        {"clock", static_cast<int64_t>(static_cast<uint32_t>(clock))},
        {"on_ticks", outVal ? 1 : 0}
    };

    if (!m_mcu.sendCommand("queue_digital_out", params)) return false;

    m_lastClock = clock;
    m_lastValue = value;
    return true;
}

// ========== MCU_pwm ==========

MCU_pwm::MCU_pwm(KlipperMCU& mcu) : m_mcu(mcu) {}

void MCU_pwm::setupPin(const std::string& pinName, bool invert) {
    m_pinName = pinName;
    m_invert = invert;
    m_startValue = invert ? 1.0 : 0.0;
    m_shutdownValue = invert ? 1.0 : 0.0;
}

void MCU_pwm::setupCycleTime(double cycleTime, bool hardwarePwm) {
    m_cycleTime = cycleTime;
    m_hardwarePwm = hardwarePwm;
}

void MCU_pwm::setupMaxDuration(double maxDuration) {
    m_maxDuration = maxDuration;
}

void MCU_pwm::setupStartValue(double startValue, double shutdownValue) {
    if (m_invert) {
        startValue = 1.0 - startValue;
        shutdownValue = 1.0 - shutdownValue;
    }
    m_startValue = std::clamp(startValue, 0.0, 1.0);
    m_shutdownValue = std::clamp(shutdownValue, 0.0, 1.0);
}

bool MCU_pwm::buildConfig() {
    int64_t mdurTicks = m_mcu.secondsToClock(m_maxDuration);
    if (mdurTicks > MAX_SCHEDULE_TICKS) mdurTicks = MAX_SCHEDULE_TICKS;

    int64_t cycleTicks = m_mcu.secondsToClock(m_cycleTime);
    if (cycleTicks > MAX_SCHEDULE_TICKS) cycleTicks = MAX_SCHEDULE_TICKS;

    m_mcu.requestMoveQueueSlot();
    m_oid = m_mcu.createOid();

    int pinNum = m_mcu.resolvePin(m_pinName);
    if (pinNum < 0) return false;

    if (m_hardwarePwm) {
        m_pwmMax = m_mcu.getConstantFloat("PWM_MAX");
        if (m_pwmMax <= 0.0) m_pwmMax = 32768.0;  // fallback

        int startVal = static_cast<int>(m_startValue * m_pwmMax + 0.5);
        int shutdownVal = static_cast<int>(m_shutdownValue * m_pwmMax + 0.5);

        std::ostringstream cfg;
        cfg << "config_pwm_out oid=" << m_oid
            << " pin=" << pinNum
            << " cycle_ticks=" << cycleTicks
            << " value=" << startVal
            << " default_value=" << shutdownVal
            << " max_duration=" << mdurTicks;
        m_mcu.addConfigCmd(cfg.str());

        // Restart: queue_pwm_out to shutdown value
        std::ostringstream rst;
        rst << "queue_pwm_out oid=" << m_oid
            << " clock=0 value=" << shutdownVal;
        m_mcu.addRestartCmd(rst.str());
    } else {
        // Software PWM uses digital_out infrastructure
        m_pwmMax = static_cast<double>(cycleTicks);

        int startBool = (m_startValue >= 1.0) ? 1 : 0;
        int shutdownBool = (m_shutdownValue >= 0.5) ? 1 : 0;

        std::ostringstream cfg;
        cfg << "config_digital_out oid=" << m_oid
            << " pin=" << pinNum
            << " value=" << startBool
            << " default_value=" << shutdownBool
            << " max_duration=" << mdurTicks;
        m_mcu.addConfigCmd(cfg.str());

        // Set PWM cycle
        std::ostringstream cycleCmd;
        cycleCmd << "set_digital_out_pwm_cycle oid=" << m_oid
                 << " cycle_ticks=" << cycleTicks;
        m_mcu.addInitCmd(cycleCmd.str());

        // Init: queue_digital_out to start value
        int startTicks = static_cast<int>(m_startValue * m_pwmMax + 0.5);
        std::ostringstream initCmd;
        initCmd << "queue_digital_out oid=" << m_oid
                << " clock=0 on_ticks=" << startTicks;
        m_mcu.addInitCmd(initCmd.str());
    }

    m_lastValue = m_startValue;
    return true;
}

bool MCU_pwm::setPwm(double printTime, double value) {
    if (m_invert) value = 1.0 - value;
    value = std::clamp(value, 0.0, 1.0);

    int v = static_cast<int>(value * m_pwmMax + 0.5);
    int64_t clock = m_mcu.getClockSync().printTimeToClock(printTime);

    bool ok;
    if (m_hardwarePwm) {
        std::map<std::string, int64_t> params = {
            {"oid", m_oid},
            {"clock", static_cast<int64_t>(static_cast<uint32_t>(clock))},
            {"value", v}
        };
        ok = m_mcu.sendCommand("queue_pwm_out", params);
    } else {
        std::map<std::string, int64_t> params = {
            {"oid", m_oid},
            {"clock", static_cast<int64_t>(static_cast<uint32_t>(clock))},
            {"on_ticks", v}
        };
        ok = m_mcu.sendCommand("queue_digital_out", params);
    }

    if (ok) {
        m_lastClock = clock;
        m_lastValue = value;
    }
    return ok;
}

// ========== MCU_adc ==========

MCU_adc::MCU_adc(KlipperMCU& mcu) : m_mcu(mcu) {}

void MCU_adc::setupPin(const std::string& pinName) {
    m_pinName = pinName;
}

void MCU_adc::setupAdcSample(double reportTime, double sampleTime,
                              int sampleCount, double minVal, double maxVal,
                              int rangeCheckCount) {
    m_reportTime = reportTime;
    m_sampleTime = sampleTime;
    m_sampleCount = sampleCount;
    m_minVal = minVal;
    m_maxVal = maxVal;
    m_rangeCheckCount = rangeCheckCount;
}

void MCU_adc::setupAdcCallback(AdcCallback cb) {
    m_callback = std::move(cb);
}

bool MCU_adc::buildConfig() {
    m_oid = m_mcu.createOid();

    int pinNum = m_mcu.resolvePin(m_pinName);
    if (pinNum < 0) return false;

    // Config
    std::ostringstream cfg;
    cfg << "config_analog_in oid=" << m_oid
        << " pin=" << pinNum;
    m_mcu.addConfigCmd(cfg.str());

    // Calculate ADC parameters
    double mcuAdcMax = m_mcu.getConstantFloat("ADC_MAX", 4095.0);
    double maxAdc = static_cast<double>(m_sampleCount) * mcuAdcMax;
    m_invMaxAdc = 1.0 / maxAdc;

    int64_t sampleTicks = m_mcu.secondsToClock(m_sampleTime);
    m_reportClock = m_mcu.secondsToClock(m_reportTime);

    int minSample = static_cast<int>(m_minVal * maxAdc);
    int maxSample = static_cast<int>(std::ceil(m_maxVal * maxAdc));

    // Stagger start: OID * 0.01s from ~1.5s in the future
    int64_t startClock = m_mcu.getClockSync().getClock(
        m_mcu.getClockSync().getDebugInfo().timeAvg + 1.5 + m_oid * 0.01);

    // Query command (sent as init so it works on restart too)
    std::ostringstream qry;
    qry << "query_analog_in oid=" << m_oid
        << " clock=" << static_cast<uint32_t>(startClock)
        << " sample_ticks=" << sampleTicks
        << " sample_count=" << m_sampleCount
        << " rest_ticks=" << m_reportClock
        << " min_value=" << minSample
        << " max_value=" << maxSample
        << " range_check_count=" << m_rangeCheckCount;
    m_mcu.addInitCmd(qry.str());

    // Register response handler
    m_mcu.registerOidResponse("analog_in_state", m_oid,
        [this](const KlipperMCU::ParsedResponse& resp) {
            auto ncIt = resp.intParams.find("next_clock");
            auto vIt = resp.intParams.find("value");
            if (ncIt != resp.intParams.end() && vIt != resp.intParams.end()) {
                handleResponse(ncIt->second, vIt->second);
            }
        });

    return true;
}

void MCU_adc::handleResponse(int64_t nextClock, int64_t value) {
    double normalized = static_cast<double>(value) * m_invMaxAdc;
    int64_t readClock = nextClock - m_reportClock;
    double readTime = m_mcu.getClockSync().clockToPrintTime(readClock);

    m_lastValue = normalized;
    m_lastReadTime = readTime;

    if (m_callback) {
        m_callback(readTime, normalized);
    }
}
