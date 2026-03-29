#include "serial_port.h"
#include <iostream>

SerialPort::SerialPort() {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& portName, uint32_t baudRate) {
    close();

    std::string path = "\\\\.\\" + portName;
    m_handle = CreateFileA(
        path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        0,
        nullptr
    );

    if (m_handle == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to open " << portName << ", error: " << GetLastError() << std::endl;
        return false;
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(m_handle, &dcb)) {
        close();
        return false;
    }

    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;

    if (!SetCommState(m_handle, &dcb)) {
        close();
        return false;
    }

    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;
    SetCommTimeouts(m_handle, &timeouts);

    PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    m_portName = portName;
    return true;
}

void SerialPort::close() {
    if (m_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(m_handle);
        m_handle = INVALID_HANDLE_VALUE;
    }
    m_portName.clear();
}

bool SerialPort::isOpen() const {
    return m_handle != INVALID_HANDLE_VALUE;
}

int SerialPort::write(const uint8_t* data, size_t length) {
    if (!isOpen()) return -1;
    DWORD written = 0;
    if (!WriteFile(m_handle, data, static_cast<DWORD>(length), &written, nullptr)) {
        return -1;
    }
    return static_cast<int>(written);
}

int SerialPort::read(uint8_t* buffer, size_t maxLength, uint32_t timeoutMs) {
    if (!isOpen()) return -1;

    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = 10;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = timeoutMs;
    SetCommTimeouts(m_handle, &timeouts);

    DWORD bytesRead = 0;
    if (!ReadFile(m_handle, buffer, static_cast<DWORD>(maxLength), &bytesRead, nullptr)) {
        return -1;
    }
    return static_cast<int>(bytesRead);
}

void SerialPort::setDTR(bool state) {
    if (isOpen()) {
        EscapeCommFunction(m_handle, state ? SETDTR : CLRDTR);
    }
}

void SerialPort::setRTS(bool state) {
    if (isOpen()) {
        EscapeCommFunction(m_handle, state ? SETRTS : CLRRTS);
    }
}

void SerialPort::purge() {
    if (isOpen()) {
        PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    }
}
