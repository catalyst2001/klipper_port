#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <memory>

class KlipperMCU;
class MCU_SPI;

// TMC5160 register addresses
namespace TMC5160Reg {
    constexpr uint8_t GCONF        = 0x00;
    constexpr uint8_t GSTAT        = 0x01;
    constexpr uint8_t IOIN         = 0x04;
    constexpr uint8_t SHORT_CONF   = 0x09;
    constexpr uint8_t DRV_CONF     = 0x0A;
    constexpr uint8_t GLOBALSCALER = 0x0B;
    constexpr uint8_t IHOLD_IRUN   = 0x10;
    constexpr uint8_t TPOWERDOWN   = 0x11;
    constexpr uint8_t TSTEP        = 0x12;
    constexpr uint8_t TPWMTHRS     = 0x13;
    constexpr uint8_t TCOOLTHRS    = 0x14;
    constexpr uint8_t THIGH        = 0x15;
    constexpr uint8_t CHOPCONF     = 0x6C;
    constexpr uint8_t COOLCONF     = 0x6D;
    constexpr uint8_t DRV_STATUS   = 0x6F;
    constexpr uint8_t PWMCONF      = 0x70;
    constexpr uint8_t PWM_SCALE    = 0x71;
}

// TMC5160 stepper driver controller via SPI (with daisy chain support)
class TMC5160 {
public:
    TMC5160(const std::string& name);

    // Configuration (before finalize)
    void setSpi(MCU_SPI* spi, int chainPosition, int chainLength);
    void setCurrent(double runCurrent, double holdCurrent, double senseResistor);
    void setMicrosteps(int microsteps, bool interpolate);
    void setStealthChop(bool enable);

    const std::string& getName() const { return m_name; }
    double getRunCurrent() const { return m_runCurrent; }
    double getHoldCurrent() const { return m_holdCurrent; }
    int getMicrosteps() const { return m_microsteps; }

    // Initialize registers (call after MCU finalize)
    bool initRegisters();

    // Register access via SPI daisy chain
    bool writeRegister(uint8_t reg, uint32_t value);
    bool readRegister(uint8_t reg, uint32_t& value);

    // Driver status
    struct DriverStatus {
        uint32_t raw = 0;         // raw DRV_STATUS register value
        bool stst = false;        // standstill
        bool olA = false;         // open load A
        bool olB = false;         // open load B
        bool s2gA = false;        // short to ground A
        bool s2gB = false;        // short to ground B
        bool s2vsA = false;       // short to supply A
        bool s2vsB = false;       // short to supply B
        bool otpw = false;        // overtemperature pre-warning
        bool ot = false;          // overtemperature shutdown
        bool stallGuard = false;  // stallGuard flag
        bool stealthChop = false; // stealthChop active
        uint8_t csActual = 0;    // actual current scale (0-31)
        uint16_t sgResult = 0;   // stallGuard result

        bool isUnpowered() const; // all bits set = VMot off
        bool hasError() const;
        bool hasWarning() const;
    };

    DriverStatus readStatus();
    static std::string formatStatus(const DriverStatus& s);
    static std::string formatErrors(const DriverStatus& s);

private:
    std::string m_name;
    MCU_SPI* m_spi = nullptr;
    int m_chainPosition = 0;
    int m_chainLength = 0;

    // Settings
    double m_runCurrent = 1.0;
    double m_holdCurrent = 0.5;
    double m_senseResistor = 0.075;
    int m_microsteps = 256;
    bool m_interpolate = true;
    bool m_stealthChop = true;

    // Computed values
    uint8_t m_globalScaler = 0;   // 0 means 256
    uint8_t m_irun = 31;
    uint8_t m_ihold = 16;

    void computeCurrentSettings();
    uint32_t buildChopConf() const;
    uint32_t buildPwmConf() const;
    uint32_t buildGConf() const;
    uint32_t buildIholdIrun() const;

    // Microstep resolution encoding: 256->0, 128->1, 64->2, ...
    static uint8_t microstepsToMRES(int microsteps);
};
