#pragma once

#include <Windows.h>
#include <string>
#include <vector>
#include <cstdint>
#include <functional>

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string& portName, uint32_t baudRate = 250000);
    void close();
    bool isOpen() const;

    int write(const uint8_t* data, size_t length);
    int read(uint8_t* buffer, size_t maxLength, uint32_t timeoutMs = 100);

    void setDTR(bool state);
    void setRTS(bool state);
    void purge();

    std::string getPortName() const { return m_portName; }

private:
    HANDLE m_handle = INVALID_HANDLE_VALUE;
    std::string m_portName;
};
