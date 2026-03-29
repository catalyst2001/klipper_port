#pragma once

#include "serial_port.h"
#include "klipper_proto.h"

#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <functional>
#include <mutex>

// Parsed command/response format
struct MessageFormat {
    int32_t msgId = 0;
    std::string name;
    std::string formatStr;      // full format string e.g. "get_clock"
    struct Param {
        std::string name;
        char type;              // 'u','i','c','s' (s = buffer/string)
    };
    std::vector<Param> params;
};

// Callback for received responses
using ResponseCallback = std::function<void(int32_t msgId, const std::string& name,
    const std::map<std::string, int64_t>& intParams,
    const std::map<std::string, std::vector<uint8_t>>& bufParams)>;

class KlipperMCU {
public:
    KlipperMCU();
    ~KlipperMCU();

    // Connect to MCU on given COM port
    bool connect(const std::string& port, uint32_t baudRate = 250000);
    void disconnect();
    bool isConnected() const;

    // Run identify handshake — populates command dictionary
    bool identify();

    // Get the raw identify JSON data
    const std::string& getIdentifyJson() const { return m_identifyJson; }

    // Get MCU version string
    const std::string& getVersion() const { return m_version; }

    // Get build versions
    const std::string& getBuildVersions() const { return m_buildVersions; }

    // Send a command by name with parameters. Returns true if sent.
    bool sendCommand(const std::string& cmdName, const std::map<std::string, int64_t>& intParams = {},
        const std::map<std::string, std::vector<uint8_t>>& bufParams = {});

    // Send command and wait for a specific response
    bool sendWithResponse(const std::string& cmdName,
        const std::string& responseName,
        std::map<std::string, int64_t>& outIntParams,
        std::map<std::string, std::vector<uint8_t>>& outBufParams,
        const std::map<std::string, int64_t>& intParams = {},
        const std::map<std::string, std::vector<uint8_t>>& bufParams = {},
        uint32_t timeoutMs = 2000);

    // Process incoming data (call periodically or in a loop)
    // Returns list of parsed responses
    struct ParsedResponse {
        int32_t msgId;
        std::string name;
        std::map<std::string, int64_t> intParams;
        std::map<std::string, std::vector<uint8_t>> bufParams;
    };
    std::vector<ParsedResponse> processIncoming(uint32_t timeoutMs = 100);

    // Set callback for async responses
    void setResponseCallback(ResponseCallback cb) { m_responseCallback = std::move(cb); }

    // Get available commands
    const std::map<std::string, MessageFormat>& getCommands() const { return m_commands; }
    const std::map<std::string, MessageFormat>& getResponses() const { return m_responses; }

    // Enumerations: name -> (value_name -> single_value) or (value_name -> [start, count])
    struct EnumValue {
        int value = 0;           // single value or range start
        int count = 0;           // 0 for single value, >0 for range
        bool isRange() const { return count > 0; }
    };
    const std::map<std::string, std::map<std::string, EnumValue>>& getEnumerations() const { return m_enumerations; }

    // Get config values
    const std::map<std::string, int>& getConfig() const { return m_config; }
    const std::map<std::string, std::string>& getConfigStrings() const { return m_configStrings; }

    // Get last error
    const std::string& getLastError() const { return m_lastError; }

private:
    SerialPort m_serial;
    uint8_t m_sendSeq = 0;
    uint8_t m_recvSeq = 0;
    bool m_needSync = true;
    std::vector<uint8_t> m_recvBuf;
    std::string m_lastError;

    // Identified data
    std::string m_identifyJson;
    std::string m_version;
    std::string m_buildVersions;

    // Command/response dictionaries
    std::map<std::string, MessageFormat> m_commands;   // by name
    std::map<std::string, MessageFormat> m_responses;  // by name
    std::map<int32_t, MessageFormat*> m_responseById;
    std::map<int32_t, MessageFormat*> m_outputById;

    // Enumerations and config
    std::map<std::string, std::map<std::string, EnumValue>> m_enumerations;
    std::map<std::string, int> m_config;
    std::map<std::string, std::string> m_configStrings;

    ResponseCallback m_responseCallback;

    // Internal
    bool sendRawFrame(const std::vector<uint8_t>& payload);
    std::vector<uint8_t> encodeCommand(const MessageFormat& fmt,
        const std::map<std::string, int64_t>& intParams,
        const std::map<std::string, std::vector<uint8_t>>& bufParams);
    ParsedResponse decodeResponse(const uint8_t* payload, size_t len);
    void parseMessageFormat(const std::string& formatStr, MessageFormat& fmt);
    bool parseIdentifyData(const std::vector<uint8_t>& data);
};
