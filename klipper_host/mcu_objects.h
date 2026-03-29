#pragma once

#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include <mutex>

// Forward declaration
class KlipperMCU;

// ---- MCU Digital Output ----
// OID-based digital output with scheduling and safety timeout.
// Corresponds to Klipper's MCU_digital_out (mcu.py).
class MCU_digital_out {
public:
    explicit MCU_digital_out(KlipperMCU& mcu);

    // Setup before finalize (mirrors Klipper's setup methods)
    void setupPin(const std::string& pinName, bool invert = false);
    void setupMaxDuration(double maxDuration);  // seconds, 0 = disabled
    void setupStartValue(bool startValue, bool shutdownValue);

    // Build config commands (called during config finalization)
    bool buildConfig();

    // Runtime: schedule digital output at a print time
    bool setDigital(double printTime, bool value);

    // Get state
    int getOid() const { return m_oid; }
    bool getLastValue() const { return m_lastValue; }
    const std::string& getPinName() const { return m_pinName; }

private:
    KlipperMCU& m_mcu;
    std::string m_pinName;
    int m_oid = -1;
    bool m_invert = false;
    double m_maxDuration = 2.0;
    bool m_startValue = false;
    bool m_shutdownValue = false;
    bool m_lastValue = false;
    int64_t m_lastClock = 0;
};

// ---- MCU PWM Output ----
// OID-based PWM output with hardware or software PWM.
// Corresponds to Klipper's MCU_pwm (mcu.py).
class MCU_pwm {
public:
    explicit MCU_pwm(KlipperMCU& mcu);

    // Setup before finalize
    void setupPin(const std::string& pinName, bool invert = false);
    void setupCycleTime(double cycleTime, bool hardwarePwm = false);
    void setupMaxDuration(double maxDuration);  // seconds
    void setupStartValue(double startValue, double shutdownValue);

    // Build config commands
    bool buildConfig();

    // Runtime: set PWM duty cycle at a print time (0.0 - 1.0)
    bool setPwm(double printTime, double value);

    // Get state
    int getOid() const { return m_oid; }
    double getLastValue() const { return m_lastValue; }
    const std::string& getPinName() const { return m_pinName; }
    double getPwmMax() const { return m_pwmMax; }

private:
    KlipperMCU& m_mcu;
    std::string m_pinName;
    int m_oid = -1;
    bool m_invert = false;
    bool m_hardwarePwm = false;
    double m_cycleTime = 0.100;
    double m_maxDuration = 2.0;
    double m_startValue = 0.0;
    double m_shutdownValue = 0.0;
    double m_lastValue = 0.0;
    double m_pwmMax = 0.0;
    int64_t m_lastClock = 0;
};

// ---- MCU Analog Input ----
// OID-based analog input with periodic sampling and range checking.
// Corresponds to Klipper's MCU_adc (mcu.py).
class MCU_adc {
public:
    using AdcCallback = std::function<void(double readTime, double value)>;

    explicit MCU_adc(KlipperMCU& mcu);

    // Setup before finalize
    void setupPin(const std::string& pinName);
    void setupAdcSample(double reportTime, double sampleTime = 0.0,
                        int sampleCount = 1, double minVal = 0.0,
                        double maxVal = 1.0, int rangeCheckCount = 0);
    void setupAdcCallback(AdcCallback cb);

    // Build config commands
    bool buildConfig();

    // Handle incoming analog_in_state response
    void handleResponse(int64_t nextClock, int64_t value);

    // Get state
    int getOid() const { return m_oid; }
    const std::string& getPinName() const { return m_pinName; }
    double getLastValue() const { return m_lastValue; }
    double getLastReadTime() const { return m_lastReadTime; }

private:
    KlipperMCU& m_mcu;
    std::string m_pinName;
    int m_oid = -1;
    double m_reportTime = 0.0;
    double m_sampleTime = 0.0;
    int m_sampleCount = 1;
    double m_minVal = 0.0;
    double m_maxVal = 1.0;
    int m_rangeCheckCount = 0;
    double m_invMaxAdc = 0.0;
    int64_t m_reportClock = 0;
    double m_lastValue = 0.0;
    double m_lastReadTime = 0.0;
    AdcCallback m_callback;
};
