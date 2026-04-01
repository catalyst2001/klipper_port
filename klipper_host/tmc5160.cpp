#include "tmc5160.h"
#include "bus_objects.h"
#include "klipper_mcu.h"

#include <cmath>
#include <sstream>
#include <iostream>
#include <algorithm>

static constexpr double VREF = 0.325;  // TMC5160 full-scale voltage

// ========== TMC5160 ==========

TMC5160::TMC5160(const std::string& name) : m_name(name) {}

void TMC5160::setSpi(MCU_SPI* spi, int chainPosition, int chainLength) {
    m_spi = spi;
    m_chainPosition = chainPosition;
    m_chainLength = chainLength;
}

void TMC5160::setCurrent(double runCurrent, double holdCurrent, double senseResistor) {
    m_runCurrent = runCurrent;
    m_holdCurrent = (std::min)(holdCurrent, runCurrent);
    m_senseResistor = senseResistor;
    computeCurrentSettings();
}

void TMC5160::setMicrosteps(int microsteps, bool interpolate) {
    m_microsteps = microsteps;
    m_interpolate = interpolate;
}

void TMC5160::setStealthChop(bool enable) {
    m_stealthChop = enable;
}

// Klipper's TMC5160 current calculation (from tmc5160.py)
void TMC5160::computeCurrentSettings() {
    // GLOBALSCALER: set so that IRUN=31 gives the desired run_current
    // globalscaler = run_current * 256 * sqrt(2) * R_sense / VREF
    int gs = static_cast<int>(m_runCurrent * 256.0 * std::sqrt(2.0)
                              * m_senseResistor / VREF + 0.5);
    gs = (std::max)(32, gs);
    if (gs >= 256) gs = 0;  // 0 means 256 in the register
    m_globalScaler = static_cast<uint8_t>(gs);

    int gsVal = m_globalScaler ? m_globalScaler : 256;

    // IRUN: cs = run_current * 256 * 32 * sqrt(2) * R_sense / (GLOBALSCALER * VREF) - 1
    int irun = static_cast<int>(m_runCurrent * 256.0 * 32.0 * std::sqrt(2.0)
                                * m_senseResistor / (gsVal * VREF) - 1.0 + 0.5);
    m_irun = static_cast<uint8_t>(std::clamp(irun, 0, 31));

    // IHOLD: same formula with hold_current
    int ihold = static_cast<int>(m_holdCurrent * 256.0 * 32.0 * std::sqrt(2.0)
                                 * m_senseResistor / (gsVal * VREF) - 1.0 + 0.5);
    m_ihold = static_cast<uint8_t>(std::clamp(ihold, 0, 31));

    std::cout << "[TMC5160:" << m_name << "] Current: run=" << m_runCurrent
              << "A hold=" << m_holdCurrent << "A R_sense=" << m_senseResistor
              << " → GS=" << gsVal << " IRUN=" << (int)m_irun
              << " IHOLD=" << (int)m_ihold << std::endl;
}

uint8_t TMC5160::microstepsToMRES(int microsteps) {
    // 256->0, 128->1, 64->2, 32->3, 16->4, 8->5, 4->6, 2->7, 1->8
    int mres = 0;
    int ms = 256;
    while (ms > microsteps && mres < 8) {
        ms >>= 1;
        mres++;
    }
    return static_cast<uint8_t>(mres);
}

uint32_t TMC5160::buildChopConf() const {
    // Klipper TMC5160 defaults: toff=3, hstrt=5, hend=2, tbl=2, tpfd=4
    uint32_t val = 0;
    val |= 3;                                    // toff [3:0]
    val |= (5U << 4);                            // hstrt [6:4]
    val |= (2U << 7);                            // hend [10:7]
    val |= (2U << 15);                           // tbl [16:15]
    val |= (4U << 20);                           // tpfd [23:20]
    val |= (static_cast<uint32_t>(microstepsToMRES(m_microsteps)) << 24); // mres [27:24]
    if (m_interpolate) val |= (1U << 28);        // intpol [28]
    return val;
}

uint32_t TMC5160::buildPwmConf() const {
    // Klipper TMC5160 defaults: pwm_ofs=30, pwm_grad=0, pwm_freq=0,
    // pwm_autoscale=1, pwm_autograd=1, freewheel=0, pwm_reg=4, pwm_lim=12
    uint32_t val = 0;
    val |= 30;                 // pwm_ofs [7:0]
    val |= (0U << 8);         // pwm_grad [15:8]
    val |= (0U << 16);        // pwm_freq [17:16]
    val |= (1U << 18);        // pwm_autoscale [18]
    val |= (1U << 19);        // pwm_autograd [19]
    val |= (0U << 20);        // freewheel [21:20]
    val |= (4U << 24);        // pwm_reg [27:24]
    val |= (12U << 28);       // pwm_lim [31:28]
    return val;
}

uint32_t TMC5160::buildGConf() const {
    uint32_t val = 0;
    if (m_stealthChop) val |= (1U << 2);  // en_pwm_mode
    val |= (1U << 3);                      // multistep_filt
    return val;
}

uint32_t TMC5160::buildIholdIrun() const {
    uint32_t val = 0;
    val |= m_ihold;              // ihold [4:0]
    val |= (m_irun << 8);       // irun [12:8]
    val |= (6U << 16);          // iholddelay [19:16] = 6
    return val;
}

bool TMC5160::writeRegister(uint8_t reg, uint32_t value) {
    if (!m_spi) return false;

    // Build 5-byte SPI datagram: [reg|0x80, data[31:24], data[23:16], data[15:8], data[7:0]]
    std::vector<uint8_t> regData = {
        static_cast<uint8_t>(reg | 0x80),
        static_cast<uint8_t>((value >> 24) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF)
    };

    if (m_chainLength > 0 && m_chainPosition > 0) {
        // Daisy chain: build full frame with NOP for other positions
        // SPI shifts data through: first byte sent ends up at farthest driver.
        // Position 1 (closest) = data at END of buffer, position N (farthest) = START.
        // Matches Klipper's: offset = (chain_len - chain_pos) * 5
        std::vector<uint8_t> frame(m_chainLength * 5, 0x00);
        int offset = (m_chainLength - m_chainPosition) * 5;
        std::copy(regData.begin(), regData.end(), frame.begin() + offset);
        return m_spi->spiSend(frame);
    }

    return m_spi->spiSend(regData);
}

bool TMC5160::readRegister(uint8_t reg, uint32_t& value) {
    if (!m_spi) return false;

    // TMC5160 SPI read: send read request, then send another request to get the response.
    // First transfer: send register address (read request)
    std::vector<uint8_t> regData = { reg, 0x00, 0x00, 0x00, 0x00 };

    if (m_chainLength > 0 && m_chainPosition > 0) {
        // Daisy chain read: offset = (chain_len - chain_pos) * 5
        std::vector<uint8_t> frame(m_chainLength * 5, 0x00);
        int offset = (m_chainLength - m_chainPosition) * 5;
        std::copy(regData.begin(), regData.end(), frame.begin() + offset);

        // First send: initiate read (response comes on NEXT transfer)
        m_spi->spiSend(frame);

        // Second transfer: send same request, capture response
        std::vector<uint8_t> response;
        std::fill(frame.begin(), frame.end(), 0x00);
        std::copy(regData.begin(), regData.end(), frame.begin() + offset);
        if (!m_spi->spiTransfer(frame, response)) return false;

        if (response.size() < static_cast<size_t>((m_chainLength - m_chainPosition + 1) * 5))
            return false;

        // Response for our position (same offset as send)
        int respOffset = (m_chainLength - m_chainPosition) * 5;
        value = (static_cast<uint32_t>(response[respOffset + 1]) << 24) |
                (static_cast<uint32_t>(response[respOffset + 2]) << 16) |
                (static_cast<uint32_t>(response[respOffset + 3]) << 8) |
                static_cast<uint32_t>(response[respOffset + 4]);
        return true;
    }

    // Non-chain read
    m_spi->spiSend(regData);
    std::vector<uint8_t> response;
    if (!m_spi->spiTransfer(regData, response)) return false;
    if (response.size() < 5) return false;

    value = (static_cast<uint32_t>(response[1]) << 24) |
            (static_cast<uint32_t>(response[2]) << 16) |
            (static_cast<uint32_t>(response[3]) << 8) |
            static_cast<uint32_t>(response[4]);
    return true;
}

bool TMC5160::initRegisters() {
    if (!m_spi) {
        std::cerr << "[TMC5160:" << m_name << "] No SPI configured" << std::endl;
        return false;
    }

    computeCurrentSettings();

    std::cout << "[TMC5160:" << m_name << "] Initializing registers..." << std::endl;

    // Read and clear GSTAT (write-1-to-clear: write back read value to clear latched bits)
    uint32_t gstat = 0;
    if (readRegister(TMC5160Reg::GSTAT, gstat)) {
        std::cout << "  GSTAT=0x" << std::hex << gstat << std::dec;
        if (gstat & 1) std::cout << " [RESET]";
        if (gstat & 2) std::cout << " [DRV_ERR]";
        if (gstat & 4) std::cout << " [UV_CP]";
        std::cout << std::endl;
        if (gstat & 0x07) {
            writeRegister(TMC5160Reg::GSTAT, gstat & 0x07);
            std::cout << "  GSTAT cleared" << std::endl;
        }
    }

    // 1. GLOBALSCALER
    if (!writeRegister(TMC5160Reg::GLOBALSCALER, m_globalScaler)) {
        std::cerr << "  Failed to write GLOBALSCALER" << std::endl;
        return false;
    }
    std::cout << "  GLOBALSCALER=" << (m_globalScaler ? (int)m_globalScaler : 256) << std::endl;

    // 2. IHOLD_IRUN
    uint32_t ihr = buildIholdIrun();
    if (!writeRegister(TMC5160Reg::IHOLD_IRUN, ihr)) {
        std::cerr << "  Failed to write IHOLD_IRUN" << std::endl;
        return false;
    }
    std::cout << "  IHOLD_IRUN=0x" << std::hex << ihr << std::dec
              << " (IRUN=" << (int)m_irun << " IHOLD=" << (int)m_ihold << ")" << std::endl;

    // 3. TPOWERDOWN
    if (!writeRegister(TMC5160Reg::TPOWERDOWN, 10)) {
        std::cerr << "  Failed to write TPOWERDOWN" << std::endl;
        return false;
    }

    // 4. CHOPCONF (includes microsteps + interpolation)
    uint32_t chopconf = buildChopConf();
    if (!writeRegister(TMC5160Reg::CHOPCONF, chopconf)) {
        std::cerr << "  Failed to write CHOPCONF" << std::endl;
        return false;
    }
    std::cout << "  CHOPCONF=0x" << std::hex << chopconf << std::dec
              << " (mres=" << (int)microstepsToMRES(m_microsteps)
              << " intpol=" << m_interpolate << ")" << std::endl;

    // 5. PWMCONF (stealthchop PWM parameters)
    uint32_t pwmconf = buildPwmConf();
    if (!writeRegister(TMC5160Reg::PWMCONF, pwmconf)) {
        std::cerr << "  Failed to write PWMCONF" << std::endl;
        return false;
    }
    std::cout << "  PWMCONF=0x" << std::hex << pwmconf << std::dec << std::endl;

    // 6. GCONF (enable stealthchop, multistep filter)
    uint32_t gconf = buildGConf();
    if (!writeRegister(TMC5160Reg::GCONF, gconf)) {
        std::cerr << "  Failed to write GCONF" << std::endl;
        return false;
    }
    std::cout << "  GCONF=0x" << std::hex << gconf << std::dec
              << (m_stealthChop ? " [StealthChop]" : " [SpreadCycle]") << std::endl;

    // 7. DRV_CONF (Klipper defaults: bbmclks=4)
    uint32_t drvconf = (4U << 8);  // bbmclks=4
    if (!writeRegister(TMC5160Reg::DRV_CONF, drvconf)) {
        std::cerr << "  Failed to write DRV_CONF" << std::endl;
        return false;
    }

    // 8. Read initial DRV_STATUS
    auto status = readStatus();
    std::cout << "  DRV_STATUS: " << formatStatus(status) << std::endl;
    if (status.hasError()) {
        std::cerr << "  ERRORS: " << formatErrors(status) << std::endl;
    }

    std::cout << "[TMC5160:" << m_name << "] Init complete" << std::endl;
    return true;
}

TMC5160::RegisterDump TMC5160::readAllRegisters() {
    RegisterDump d;
    if (!m_spi) return d;

    readRegister(TMC5160Reg::GCONF, d.gconf);
    readRegister(TMC5160Reg::GSTAT, d.gstat);
    // GSTAT is write-1-to-clear: write back to clear any latched error bits
    if (d.gstat & 0x07)
        writeRegister(TMC5160Reg::GSTAT, d.gstat & 0x07);
    readRegister(TMC5160Reg::IOIN, d.ioin);
    readRegister(TMC5160Reg::IHOLD_IRUN, d.ihold_irun);
    readRegister(TMC5160Reg::CHOPCONF, d.chopconf);
    readRegister(TMC5160Reg::DRV_STATUS, d.drv_status);
    readRegister(TMC5160Reg::PWMCONF, d.pwmconf);
    readRegister(TMC5160Reg::PWM_SCALE, d.pwm_scale);
    d.valid = true;
    return d;
}

TMC5160::DriverStatus TMC5160::readStatus() {
    DriverStatus s;
    uint32_t raw = 0;
    if (!readRegister(TMC5160Reg::DRV_STATUS, raw))
        return s;

    s.raw         = raw;
    s.sgResult    = raw & 0x3FF;
    s.s2vsA       = (raw >> 12) & 1;
    s.s2vsB       = (raw >> 13) & 1;
    s.stealthChop = (raw >> 14) & 1;
    s.csActual    = (raw >> 16) & 0x1F;
    s.stallGuard  = (raw >> 24) & 1;
    s.ot          = (raw >> 25) & 1;
    s.otpw        = (raw >> 26) & 1;
    s.s2gA        = (raw >> 27) & 1;
    s.s2gB        = (raw >> 28) & 1;
    s.olA         = (raw >> 29) & 1;
    s.olB         = (raw >> 30) & 1;
    s.stst        = (raw >> 31) & 1;
    return s;
}

bool TMC5160::DriverStatus::hasError() const {
    return ot || s2gA || s2gB || s2vsA || s2vsB;
}

bool TMC5160::DriverStatus::hasWarning() const {
    return otpw || olA || olB;
}

std::string TMC5160::formatStatus(const DriverStatus& s) {
    std::ostringstream ss;
    ss << "CS=" << (int)s.csActual
       << " SG=" << s.sgResult;
    if (s.stealthChop) ss << " [StChop]";
    if (s.stst) ss << " [Standstill]";
    if (s.stallGuard) ss << " [StallGuard]";
    return ss.str();
}

std::string TMC5160::formatErrors(const DriverStatus& s) {
    std::ostringstream ss;
    bool first = true;
    auto add = [&](const char* msg) {
        if (!first) ss << ", ";
        ss << msg;
        first = false;
    };

    if (s.ot)    add("OVERTEMP SHUTDOWN");
    if (s.otpw)  add("Overtemp warning");
    if (s.s2gA)  add("Short-to-GND phase A");
    if (s.s2gB)  add("Short-to-GND phase B");
    if (s.s2vsA) add("Short-to-supply phase A");
    if (s.s2vsB) add("Short-to-supply phase B");
    if (s.olA)   add("Open load phase A");
    if (s.olB)   add("Open load phase B");

    return first ? "OK" : ss.str();
}
