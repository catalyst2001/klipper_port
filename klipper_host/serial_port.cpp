#include "serial_port.h"
#include <iostream>

SerialPort::SerialPort() {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& portName, uint32_t baudRate) {
    close();

    std::string path = "\\\\.\\" + portName;
    // FILE_FLAG_OVERLAPPED enables asynchronous I/O — ReadFile/WriteFile
    // return immediately and we wait on an event with a real timeout.
    // This prevents indefinite blocking when USB-CDC device disappears.
    m_handle = CreateFileA(
        path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        nullptr
    );

    if (m_handle == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to open " << portName << ", error: " << GetLastError() << std::endl;
        return false;
    }

    // Create events for overlapped I/O
    m_readEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
    m_writeEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
    if (!m_readEvent || !m_writeEvent) {
        std::cerr << "Failed to create overlapped events" << std::endl;
        close();
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

    // Default COMMTIMEOUTS — read() overrides per-call.
    // COMMTIMEOUTS provide normal timing; overlapped I/O is a safety net
    // that guarantees we can cancel reads if USB-CDC device disappears.
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 500;
    SetCommTimeouts(m_handle, &timeouts);

    PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    m_portName = portName;
    return true;
}

void SerialPort::close() {
    if (m_handle != INVALID_HANDLE_VALUE) {
        // Cancel any pending overlapped I/O before closing
        CancelIoEx(m_handle, nullptr);
        CloseHandle(m_handle);
        m_handle = INVALID_HANDLE_VALUE;
    }
    if (m_readEvent) {
        CloseHandle(m_readEvent);
        m_readEvent = nullptr;
    }
    if (m_writeEvent) {
        CloseHandle(m_writeEvent);
        m_writeEvent = nullptr;
    }
    m_portName.clear();
}

bool SerialPort::isOpen() const {
    return m_handle != INVALID_HANDLE_VALUE;
}

int SerialPort::write(const uint8_t* data, size_t length) {
    if (!isOpen()) return -1;

    OVERLAPPED ov = {};
    ov.hEvent = m_writeEvent;
    ResetEvent(m_writeEvent);

    DWORD written = 0;
    if (!WriteFile(m_handle, data, static_cast<DWORD>(length), &written, &ov)) {
        if (GetLastError() != ERROR_IO_PENDING) {
            return -1;
        }
        // Wait up to 500ms for write to complete
        DWORD waitResult = WaitForSingleObject(m_writeEvent, 500);
        if (waitResult != WAIT_OBJECT_0) {
            CancelIoEx(m_handle, &ov);
            return -1;
        }
        if (!GetOverlappedResult(m_handle, &ov, &written, FALSE)) {
            return -1;
        }
    }
    return static_cast<int>(written);
}

int SerialPort::read(uint8_t* buffer, size_t maxLength, uint32_t timeoutMs) {
    if (!isOpen()) return -1;

    // Set per-read COMMTIMEOUTS (same timing as original synchronous code).
    // The driver handles normal read completion; overlapped I/O is only
    // a safety net so we can CancelIoEx if USB device disappears.
    COMMTIMEOUTS ct = {};
    ct.ReadIntervalTimeout = 10;
    ct.ReadTotalTimeoutMultiplier = 0;
    ct.ReadTotalTimeoutConstant = timeoutMs;
    SetCommTimeouts(m_handle, &ct);

    OVERLAPPED ov = {};
    ov.hEvent = m_readEvent;
    ResetEvent(m_readEvent);

    DWORD bytesRead = 0;
    if (!ReadFile(m_handle, buffer, static_cast<DWORD>(maxLength), &bytesRead, &ov)) {
        if (GetLastError() != ERROR_IO_PENDING) {
            return -1;
        }
        // Normally COMMTIMEOUTS completes the operation within timeoutMs.
        // Safety margin catches USB-CDC disconnect where driver ignores timeouts.
        DWORD safetyMs = timeoutMs + 500;
        DWORD waitResult = WaitForSingleObject(m_readEvent, safetyMs);
        if (waitResult == WAIT_OBJECT_0) {
            if (!GetOverlappedResult(m_handle, &ov, &bytesRead, FALSE)) {
                return -1;
            }
        } else {
            // Safety timeout or WAIT_FAILED — device likely gone
            CancelIoEx(m_handle, &ov);
            GetOverlappedResult(m_handle, &ov, &bytesRead, TRUE);
            return -1;
        }
    }
    return static_cast<int>(bytesRead);
}

bool SerialPort::checkHealth() const {
    if (!isOpen()) return false;
    DWORD errors = 0;
    COMSTAT stat = {};
    if (!ClearCommError(m_handle, &errors, &stat)) {
        return false; // device gone
    }
    return true;
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
