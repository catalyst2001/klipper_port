#include "bus_objects.h"
#include "klipper_mcu.h"

#include <sstream>
#include <algorithm>
#include <cmath>
#include <cstring>

// ========== MCU_SPI ==========

MCU_SPI::MCU_SPI(KlipperMCU& mcu) : m_mcu(mcu) {}

void MCU_SPI::setupPin(const std::string& csPin, bool csActiveHigh) {
    m_csPin = csPin;
    m_csActiveHigh = csActiveHigh;
}

void MCU_SPI::setupBus(const std::string& spiBus, int mode, int rate) {
    m_spiBus = spiBus;
    m_mode = mode;
    m_rate = rate;
}

bool MCU_SPI::buildConfig() {
    m_oid = m_mcu.createOid();

    // CS pin config
    if (!m_csPin.empty()) {
        int csNum = m_mcu.resolvePin(m_csPin);
        if (csNum < 0) return false;

        std::ostringstream cfg;
        cfg << "config_spi oid=" << m_oid
            << " pin=" << csNum
            << " cs_active_high=" << (m_csActiveHigh ? 1 : 0);
        m_mcu.addConfigCmd(cfg.str());
    } else {
        std::ostringstream cfg;
        cfg << "config_spi_without_cs oid=" << m_oid;
        m_mcu.addConfigCmd(cfg.str());
    }

    // Bus config
    int busNum = m_mcu.resolveEnum("spi_bus", m_spiBus);
    if (busNum < 0) return false;

    std::ostringstream bus;
    bus << "spi_set_bus oid=" << m_oid
        << " spi_bus=" << busNum
        << " mode=" << m_mode
        << " rate=" << m_rate;
    m_mcu.addConfigCmd(bus.str());

    return true;
}

bool MCU_SPI::spiSend(const std::vector<uint8_t>& data) {
    std::map<std::string, int64_t> intParams = {{"oid", m_oid}};
    std::map<std::string, std::vector<uint8_t>> bufParams = {{"data", data}};
    return m_mcu.sendCommand("spi_send", intParams, bufParams);
}

bool MCU_SPI::spiTransfer(const std::vector<uint8_t>& data, std::vector<uint8_t>& response) {
    std::map<std::string, int64_t> outInt;
    std::map<std::string, std::vector<uint8_t>> outBuf;
    std::map<std::string, int64_t> intParams = {{"oid", m_oid}};
    std::map<std::string, std::vector<uint8_t>> bufParams = {{"data", data}};

    if (!m_mcu.sendWithResponse("spi_transfer", "spi_transfer_response",
                                 outInt, outBuf, intParams, bufParams)) {
        return false;
    }

    auto it = outBuf.find("response");
    if (it != outBuf.end()) {
        response = it->second;
    }
    return true;
}

// ========== MCU_I2C ==========

MCU_I2C::MCU_I2C(KlipperMCU& mcu) : m_mcu(mcu) {}

void MCU_I2C::setupBus(const std::string& i2cBus, int rate, int address) {
    m_i2cBus = i2cBus;
    m_rate = rate;
    m_address = address;
}

bool MCU_I2C::buildConfig() {
    m_oid = m_mcu.createOid();

    // Base config
    std::ostringstream cfg;
    cfg << "config_i2c oid=" << m_oid;
    m_mcu.addConfigCmd(cfg.str());

    // Bus config
    int busNum = m_mcu.resolveEnum("i2c_bus", m_i2cBus);
    if (busNum < 0) return false;

    std::ostringstream bus;
    bus << "i2c_set_bus oid=" << m_oid
        << " i2c_bus=" << busNum
        << " rate=" << m_rate
        << " address=" << m_address;
    m_mcu.addConfigCmd(bus.str());

    return true;
}

bool MCU_I2C::i2cWrite(const std::vector<uint8_t>& data) {
    std::map<std::string, int64_t> intParams = {{"oid", m_oid}, {"read_len", 0}};
    std::map<std::string, std::vector<uint8_t>> bufParams = {{"write", data}};
    return m_mcu.sendCommand("i2c_transfer", intParams, bufParams);
}

bool MCU_I2C::i2cRead(const std::vector<uint8_t>& regData, int readLen,
                       std::vector<uint8_t>& response) {
    std::map<std::string, int64_t> outInt;
    std::map<std::string, std::vector<uint8_t>> outBuf;
    std::map<std::string, int64_t> intParams = {{"oid", m_oid}, {"read_len", readLen}};
    std::map<std::string, std::vector<uint8_t>> bufParams = {{"write", regData}};

    if (!m_mcu.sendWithResponse("i2c_transfer", "i2c_response",
                                 outInt, outBuf, intParams, bufParams)) {
        return false;
    }

    auto it = outBuf.find("response");
    if (it != outBuf.end()) {
        response = it->second;
    }
    return true;
}

// ========== MCU_TMC_uart ==========

MCU_TMC_uart::MCU_TMC_uart(KlipperMCU& mcu) : m_mcu(mcu) {}

void MCU_TMC_uart::setupPins(const std::string& rxPin, const std::string& txPin,
                              bool pullUp, int baudRate) {
    m_rxPin = rxPin;
    m_txPin = txPin;
    m_pullUp = pullUp;
    m_baudRate = baudRate;
}

bool MCU_TMC_uart::buildConfig() {
    m_oid = m_mcu.createOid();

    int rxNum = m_mcu.resolvePin(m_rxPin);
    int txNum = m_mcu.resolvePin(m_txPin);
    if (rxNum < 0 || txNum < 0) return false;

    int64_t bitTime = m_mcu.secondsToClock(1.0 / m_baudRate);

    std::ostringstream cfg;
    cfg << "config_tmcuart oid=" << m_oid
        << " rx_pin=" << rxNum
        << " pull_up=" << (m_pullUp ? 1 : 0)
        << " tx_pin=" << txNum
        << " bit_time=" << bitTime;
    m_mcu.addConfigCmd(cfg.str());

    return true;
}

uint8_t MCU_TMC_uart::calcCrc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t j = 0; j < len; j++) {
        uint8_t b = data[j];
        for (int i = 0; i < 8; i++) {
            if ((crc >> 7) ^ (b & 0x01))
                crc = (crc << 1) ^ 0x07;
            else
                crc = crc << 1;
            b >>= 1;
        }
    }
    return crc;
}

std::vector<uint8_t> MCU_TMC_uart::encodeBitframe(const std::vector<uint8_t>& rawBytes) {
    // Each byte becomes 10 bits: start(0) + 8 data bits (LSB first) + stop(1)
    std::vector<uint8_t> result;
    size_t totalBits = rawBytes.size() * 10;
    result.resize((totalBits + 7) / 8, 0);

    size_t bitPos = 0;
    for (uint8_t b : rawBytes) {
        uint16_t framed = (static_cast<uint16_t>(b) << 1) | 0x200; // start=0, stop=1

        for (int i = 0; i < 10; i++) {
            if (framed & (1 << i)) {
                result[bitPos / 8] |= (1 << (bitPos % 8));
            }
            bitPos++;
        }
    }
    return result;
}

std::vector<uint8_t> MCU_TMC_uart::decodeBitframe(const std::vector<uint8_t>& bitframed) {
    // Extract data bytes: skip start bit, read 8 data bits, skip stop bit
    std::vector<uint8_t> result;
    size_t totalBits = bitframed.size() * 8;
    size_t byteCount = totalBits / 10;

    for (size_t n = 0; n < byteCount; n++) {
        uint8_t val = 0;
        size_t startBit = n * 10 + 1; // skip start bit
        for (int i = 0; i < 8; i++) {
            size_t bp = startBit + i;
            if (bp / 8 < bitframed.size()) {
                if (bitframed[bp / 8] & (1 << (bp % 8))) {
                    val |= (1 << i);
                }
            }
        }
        result.push_back(val);
    }
    return result;
}

std::vector<uint8_t> MCU_TMC_uart::encodeRead(uint8_t sync, uint8_t addr, uint8_t reg) {
    uint8_t raw[4];
    raw[0] = sync;
    raw[1] = addr;
    raw[2] = reg;
    raw[3] = calcCrc8(raw, 3);
    return encodeBitframe({raw, raw + 4});
}

std::vector<uint8_t> MCU_TMC_uart::encodeWrite(uint8_t sync, uint8_t addr, uint8_t reg, uint32_t val) {
    uint8_t raw[8];
    raw[0] = sync;
    raw[1] = addr;
    raw[2] = reg;
    raw[3] = static_cast<uint8_t>((val >> 24) & 0xFF);
    raw[4] = static_cast<uint8_t>((val >> 16) & 0xFF);
    raw[5] = static_cast<uint8_t>((val >> 8) & 0xFF);
    raw[6] = static_cast<uint8_t>(val & 0xFF);
    raw[7] = calcCrc8(raw, 7);
    return encodeBitframe({raw, raw + 8});
}

bool MCU_TMC_uart::readRegister(uint8_t addr, uint8_t reg, uint32_t& value) {
    auto msg = encodeRead(0xF5, addr, reg);

    std::map<std::string, int64_t> outInt;
    std::map<std::string, std::vector<uint8_t>> outBuf;
    std::map<std::string, int64_t> intParams = {{"oid", m_oid}, {"read", 10}};
    std::map<std::string, std::vector<uint8_t>> bufParams = {{"write", msg}};

    for (int retry = 0; retry < 5; retry++) {
        if (!m_mcu.sendWithResponse("tmcuart_send", "tmcuart_response",
                                     outInt, outBuf, intParams, bufParams)) {
            continue;
        }

        auto it = outBuf.find("read");
        if (it == outBuf.end() || it->second.size() < 10) continue;

        // Decode bit-framed response to raw bytes
        auto decoded = decodeBitframe(it->second);
        if (decoded.size() < 8) continue;

        // Verify: sync=0x05, addr=0xFF, register matches, CRC OK
        if (decoded[0] != 0x05 || decoded[1] != 0xFF) continue;
        if (decoded[2] != reg) continue;
        if (calcCrc8(decoded.data(), 7) != decoded[7]) continue;

        value = (static_cast<uint32_t>(decoded[3]) << 24) |
                (static_cast<uint32_t>(decoded[4]) << 16) |
                (static_cast<uint32_t>(decoded[5]) << 8) |
                static_cast<uint32_t>(decoded[6]);
        return true;
    }
    return false;
}

bool MCU_TMC_uart::writeRegister(uint8_t addr, uint8_t reg, uint32_t value) {
    auto msg = encodeWrite(0xF5, addr, reg | 0x80, value);

    std::map<std::string, int64_t> intParams = {{"oid", m_oid}, {"read", 0}};
    std::map<std::string, std::vector<uint8_t>> bufParams = {{"write", msg}};

    for (int retry = 0; retry < 3; retry++) {
        if (m_mcu.sendCommand("tmcuart_send", intParams, bufParams)) {
            return true;
        }
    }
    return false;
}

// ========== MCU_Thermocouple ==========

MCU_Thermocouple::MCU_Thermocouple(KlipperMCU& mcu) : m_mcu(mcu) {}

void MCU_Thermocouple::setupSpi(MCU_SPI& spi) {
    m_spi = &spi;
}

void MCU_Thermocouple::setupSensor(SensorType type) {
    m_sensorType = type;
}

void MCU_Thermocouple::setupReportTime(double reportTime) {
    m_reportTime = reportTime;
}

void MCU_Thermocouple::setCallback(TempCallback cb) {
    m_callback = std::move(cb);
}

bool MCU_Thermocouple::initSensor() {
    if (!m_spi) return false;

    switch (m_sensorType) {
    case SensorType::MAX31856: {
        // Write CR0: auto-convert (0x80) | fault-clear (0x02) | 50Hz filter (0x01)
        m_spi->spiSend({0x80, 0x83});
        // Write CR1: K-type thermocouple (0x03) | average 4 samples (0x20)
        m_spi->spiSend({0x81, 0x23});
        // Write MASK: mask voltage/open faults
        m_spi->spiSend({0x82, 0xFC});
        break;
    }
    case SensorType::MAX31865: {
        // Config register: Vbias on, auto-convert, fault clear
        m_spi->spiSend({0x80, 0xC2});
        break;
    }
    default:
        // MAX31855 and MAX6675 don't need SPI init
        break;
    }
    return true;
}

bool MCU_Thermocouple::buildConfig() {
    if (!m_spi) return false;

    m_oid = m_mcu.createOid();

    // Thermocouple type enum
    int typeVal = static_cast<int>(m_sensorType);

    std::ostringstream cfg;
    cfg << "config_thermocouple oid=" << m_oid
        << " spi_oid=" << m_spi->getOid()
        << " thermocouple_type=" << typeVal;
    m_mcu.addConfigCmd(cfg.str());

    // Query: start autonomous sampling
    int64_t restTicks = m_mcu.secondsToClock(m_reportTime);
    int64_t startClock = m_mcu.getClockSync().getClock(
        m_mcu.getClockSync().getDebugInfo().timeAvg + 1.5);

    std::ostringstream qry;
    qry << "query_thermocouple oid=" << m_oid
        << " clock=" << static_cast<uint32_t>(startClock)
        << " rest_ticks=" << restTicks
        << " min_value=0 max_value=0 max_invalid_count=0";
    m_mcu.addInitCmd(qry.str());

    // Register response handler
    m_mcu.registerOidResponse("thermocouple_result", m_oid,
        [this](const KlipperMCU::ParsedResponse& resp) {
            auto ncIt = resp.intParams.find("next_clock");
            auto vIt = resp.intParams.find("value");
            auto fIt = resp.intParams.find("fault");
            if (ncIt != resp.intParams.end() && vIt != resp.intParams.end()) {
                int64_t fault = (fIt != resp.intParams.end()) ? fIt->second : 0;
                handleResponse(ncIt->second, vIt->second, fault);
            }
        });

    return true;
}

void MCU_Thermocouple::handleResponse(int64_t nextClock, int64_t rawValue, int64_t fault) {
    m_lastFault = static_cast<uint8_t>(fault);
    m_lastTemp = convertTemperature(m_sensorType, static_cast<int32_t>(rawValue));

    if (m_callback) {
        m_callback(m_lastTemp, m_lastFault);
    }
}

double MCU_Thermocouple::convertTemperature(SensorType type, int32_t rawValue) {
    switch (type) {
    case SensorType::MAX31855: {
        // 32-bit: bits [31:18] = signed 14-bit temp, LSB = 0.25C
        int32_t temp14 = rawValue >> 18;
        if (temp14 & 0x2000) temp14 |= ~0x3FFF;  // sign extend
        return temp14 * 0.25;
    }
    case SensorType::MAX31856: {
        // 24-bit: bits [23:5] = signed 19-bit temp, LSB = 0.0078125C
        int32_t temp19 = rawValue >> 5;
        if (temp19 & 0x40000) temp19 |= ~0x7FFFF;  // sign extend
        return temp19 * 0.0078125;
    }
    case SensorType::MAX6675: {
        // 16-bit: bits [15:3] = unsigned 13-bit temp, LSB = 0.25C
        int32_t temp13 = (rawValue >> 3) & 0x1FFF;
        return temp13 * 0.25;
    }
    case SensorType::MAX31865: {
        // 16-bit: bits [15:1] = 15-bit ADC, Callendar-Van Dusen for PT100/PT1000
        int32_t adc15 = (rawValue >> 1) & 0x7FFF;
        // RTD resistance ratio: R/R0 = adc / 32768
        double resistance_ratio = static_cast<double>(adc15) / 32768.0;
        // Callendar-Van Dusen: A = 3.9083e-3, B = -5.775e-7
        constexpr double A = 3.9083e-3;
        constexpr double B = -5.775e-7;
        double disc = A * A - 4.0 * B * (1.0 - resistance_ratio);
        if (disc < 0) return -999.0;
        return (-A + std::sqrt(disc)) / (2.0 * B);
    }
    default:
        return 0.0;
    }
}
