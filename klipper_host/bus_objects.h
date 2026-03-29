#pragma once

#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include <mutex>

class KlipperMCU;

// ---- MCU SPI Bus ----
// OID-based SPI communication. Corresponds to Klipper's MCU_SPI (bus.py).
class MCU_SPI {
public:
    explicit MCU_SPI(KlipperMCU& mcu);

    // Setup before finalize
    void setupPin(const std::string& csPin, bool csActiveHigh = false);
    void setupBus(const std::string& spiBus, int mode, int rate);

    // Build config commands
    bool buildConfig();

    // Runtime: send data (fire-and-forget)
    bool spiSend(const std::vector<uint8_t>& data);

    // Runtime: transfer data (sends and waits for response)
    bool spiTransfer(const std::vector<uint8_t>& data, std::vector<uint8_t>& response);

    int getOid() const { return m_oid; }

private:
    KlipperMCU& m_mcu;
    std::string m_csPin;
    bool m_csActiveHigh = false;
    std::string m_spiBus;
    int m_mode = 0;
    int m_rate = 4000000;
    int m_oid = -1;
};

// ---- MCU I2C Bus ----
// OID-based I2C communication. Corresponds to Klipper's MCU_I2C (bus.py).
class MCU_I2C {
public:
    explicit MCU_I2C(KlipperMCU& mcu);

    void setupBus(const std::string& i2cBus, int rate, int address);

    bool buildConfig();

    // Write data to I2C device
    bool i2cWrite(const std::vector<uint8_t>& data);

    // Read from I2C device: write register address bytes, read back readLen bytes
    bool i2cRead(const std::vector<uint8_t>& regData, int readLen,
                 std::vector<uint8_t>& response);

    int getOid() const { return m_oid; }

private:
    KlipperMCU& m_mcu;
    std::string m_i2cBus;
    int m_rate = 100000;
    int m_address = 0;
    int m_oid = -1;
};

// ---- TMC UART ----
// Single-wire UART communication with Trinamic stepper drivers.
// Corresponds to Klipper's MCU_TMC_uart_bitbang (tmc_uart.py).
class MCU_TMC_uart {
public:
    explicit MCU_TMC_uart(KlipperMCU& mcu);

    void setupPins(const std::string& rxPin, const std::string& txPin,
                   bool pullUp = true, int baudRate = 40000);

    bool buildConfig();

    // Read a TMC register. Returns true if successful.
    bool readRegister(uint8_t addr, uint8_t reg, uint32_t& value);

    // Write a TMC register. Returns true if write verified.
    bool writeRegister(uint8_t addr, uint8_t reg, uint32_t value);

    int getOid() const { return m_oid; }

    // TMC UART CRC8-ATM
    static uint8_t calcCrc8(const uint8_t* data, size_t len);

private:
    KlipperMCU& m_mcu;
    std::string m_rxPin;
    std::string m_txPin;
    bool m_pullUp = true;
    int m_baudRate = 40000;
    int m_oid = -1;

    // Encode raw bytes with serial bit-framing (start+data+stop bits)
    static std::vector<uint8_t> encodeBitframe(const std::vector<uint8_t>& rawBytes);

    // Decode bit-framed response back to raw bytes
    static std::vector<uint8_t> decodeBitframe(const std::vector<uint8_t>& bitframed);

    // Build a TMC read request message (bit-framed)
    static std::vector<uint8_t> encodeRead(uint8_t sync, uint8_t addr, uint8_t reg);

    // Build a TMC write request message (bit-framed)
    static std::vector<uint8_t> encodeWrite(uint8_t sync, uint8_t addr, uint8_t reg, uint32_t val);
};

// ---- Thermocouple Reader ----
// SPI thermocouple reader using firmware's autonomous sampling.
// Supports MAX31855, MAX31856, MAX6675, MAX31865.
class MCU_Thermocouple {
public:
    enum class SensorType {
        MAX31855 = 0,
        MAX31856 = 1,
        MAX6675 = 2,
        MAX31865 = 3
    };

    using TempCallback = std::function<void(double temperature, uint8_t fault)>;

    explicit MCU_Thermocouple(KlipperMCU& mcu);

    void setupSpi(MCU_SPI& spi);
    void setupSensor(SensorType type);
    void setupReportTime(double reportTime);
    void setCallback(TempCallback cb);

    // Send chip-specific init commands via SPI
    bool initSensor();

    // Build config + start autonomous sampling
    bool buildConfig();

    // Handle thermocouple_result response
    void handleResponse(int64_t nextClock, int64_t rawValue, int64_t fault);

    int getOid() const { return m_oid; }
    double getLastTemperature() const { return m_lastTemp; }
    uint8_t getLastFault() const { return m_lastFault; }

    // Convert raw ADC value to temperature for a given sensor
    static double convertTemperature(SensorType type, int32_t rawValue);

private:
    KlipperMCU& m_mcu;
    MCU_SPI* m_spi = nullptr;
    SensorType m_sensorType = SensorType::MAX31855;
    double m_reportTime = 0.300;
    int m_oid = -1;
    double m_lastTemp = 0.0;
    uint8_t m_lastFault = 0;
    TempCallback m_callback;
};
