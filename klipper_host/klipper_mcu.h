#pragma once

#include "serial_port.h"
#include "serial_queue.h"
#include "klipper_proto.h"
#include "clock_sync.h"

#include <string>
#include <vector>
#include <map>
#include <queue>
#include <cstdint>
#include <functional>
#include <mutex>
#include <atomic>

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

    // ---- Clock Sync ----
    ClockSync& getClockSync() { return m_clockSync; }
    const ClockSync& getClockSync() const { return m_clockSync; }

    // Initialize clock synchronization (call after identify)
    bool initClockSync();

    // Send periodic get_clock for clock sync maintenance.
    // Returns false if MCU is unresponsive.
    bool clockSyncPoll();

    // Async clock sync poll via SerialQueue (non-blocking).
    // Sends get_clock through SerialQueue; response handled by receive callback.
    void clockSyncPollAsync();

    // ---- OID Management ----
    // Allocate a new OID. Returns the next sequential OID number.
    int createOid();

    // Get the current OID count
    int getOidCount() const { return m_oidCount; }

    // ---- Pin Resolution ----
    // Resolve a pin name (e.g. "PA0", "PD5") to MCU pin number using enumerations.
    // Returns -1 if not found.
    int resolvePin(const std::string& pinName) const;

    // Resolve an enumeration value by enum name and value name.
    // Returns -1 if not found.
    int resolveEnum(const std::string& enumName, const std::string& valueName) const;

    // ---- Config Finalization ----
    // Add a config command (sent only on first config, not on reconnect)
    void addConfigCmd(const std::string& cmd);

    // Add a restart command (sent on reconnect when CRC matches)
    void addRestartCmd(const std::string& cmd);

    // Add an init command (sent always, after config or restart commands)
    void addInitCmd(const std::string& cmd);

    // Finalize configuration: allocate_oids → config_cmds → finalize_config crc=X
    // Sends all commands to MCU. Returns false on failure.
    bool finalizeConfig();

    // Check if config has been finalized
    bool isConfigFinalized() const { return m_configFinalized; }

    // MCU move queue capacity (from get_config after finalization)
    int getMcuMoveCount() const { return m_mcuMoveCount; }

    // Reset config state (for reconnection)
    void resetConfig();

    // ---- Shutdown/Restart ----
    // Check if MCU is in shutdown state
    bool isShutdown() const { return m_isShutdown.load(); }

    // Get shutdown message
    std::string getShutdownMsg() const;

    // Clear shutdown state (sends clear_shutdown to MCU)
    bool clearShutdown();

    // Request firmware restart
    bool firmwareRestart();

    // Register a shutdown callback
    using ShutdownCallback = std::function<void(const std::string& reason)>;
    void setShutdownCallback(ShutdownCallback cb) { m_shutdownCallback = std::move(cb); }

    // ---- Convenience Methods ----
    // Convert seconds to MCU clock ticks using estimated frequency
    int64_t secondsToClock(double seconds) const;

    // Get an integer constant from MCU config (e.g. "ADC_MAX", "PWM_MAX")
    int getConstantInt(const std::string& name, int defaultVal = 0) const;

    // Get a float constant (reads int constant and returns as double)
    double getConstantFloat(const std::string& name, double defaultVal = 0.0) const;

    // Register a response handler for a specific OID (analog_in_state, etc.)
    using OidResponseHandler = std::function<void(const ParsedResponse& resp)>;
    void registerOidResponse(const std::string& responseName, int oid, OidResponseHandler handler);

    // Request a move queue slot (needed for scheduled commands)
    void requestMoveQueueSlot();

    // ---- Command Batching (legacy path, used only before SerialQueue starts) ----
    // Encode a command into raw bytes (without sending)
    std::vector<uint8_t> encodeCommandPayload(const std::string& cmdName,
        const std::map<std::string, int64_t>& intParams = {},
        const std::map<std::string, std::vector<uint8_t>>& bufParams = {});

    // Queue an encoded command payload for batched sending.
    // Automatically flushes when the message would exceed MESSAGE_PAYLOAD_MAX.
    bool queuePayload(const std::vector<uint8_t>& payload);

    // Flush any accumulated batch payloads as a single message.
    bool flushBatch();

    // ---- SerialQueue (background-threaded serial with clock-gating) ----
    // Start the SerialQueue background thread. Call after finalizeConfig() + initClockSync().
    bool startSerialQueue();

    // Stop the SerialQueue background thread.
    void stopSerialQueue();

    // Get the SerialQueue (for step generation to submit commands directly).
    SerialQueue& getSerialQueue() { return m_serialQueue; }
    const SerialQueue& getSerialQueue() const { return m_serialQueue; }

    // Check if SerialQueue is active (i.e., started and running).
    bool isSerialQueueActive() const { return m_serialQueue.isRunning(); }

    // Submit a timed command via SerialQueue.
    // cmdName: command name (e.g. "queue_step")
    // min_clock, req_clock: clock-gating parameters
    // cq: CommandQueue to use (nullptr = default queue)
    void sendTimed(const std::string& cmdName,
                   const std::map<std::string, int64_t>& intParams,
                   uint64_t min_clock, uint64_t req_clock,
                   CommandQueue* cq = nullptr);

    // Submit a raw encoded payload via SerialQueue.
    void sendTimedRaw(const uint8_t* payload, int len,
                      uint64_t min_clock, uint64_t req_clock,
                      CommandQueue* cq = nullptr);

    // StepperSync: move queue flow control (matches Klipper's steppersync.c).
    // Adjusts min_clock for queue_step commands so we never exceed the MCU's
    // move queue capacity. Call for each queue_step; pass the clock at which
    // the step command's last step will finish (endClock).
    uint64_t stepSyncAdjustMinClock(uint64_t minClock, uint64_t endClock);
    void stepSyncReset();
    uint64_t getStepSyncTotal() const { return m_stepSyncTotal.load(std::memory_order_relaxed); }
    uint64_t getStepSyncGated() const { return m_stepSyncGated.load(std::memory_order_relaxed); }

private:
    SerialPort m_serial;
    std::string m_portName;
    uint32_t m_baudRate = 250000;
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

    // Clock synchronization
    ClockSync m_clockSync;

    // OID management
    int m_oidCount = 0;
    int m_moveQueueSlots = 0;
    int m_mcuMoveCount = 0;
    bool m_configFinalized = false;
    std::vector<std::string> m_configCmds;
    std::vector<std::string> m_restartCmds;
    std::vector<std::string> m_initCmds;

    // OID response handlers: key = "responseName:oid"
    std::map<std::string, OidResponseHandler> m_oidHandlers;

    // Shutdown state
    std::atomic<bool> m_isShutdown{false};
    std::string m_shutdownMsg;
    mutable std::mutex m_shutdownMutex;
    ShutdownCallback m_shutdownCallback;

    // Serial write mutex — serializes sendRawFrame across threads.
    // generateSteps() sends queue_step commands without the application-level
    // mutex, while pollThread/clockSyncThread also access the serial port.
    // This mutex prevents interleaved WriteFile calls and m_sendSeq races.
    std::mutex m_sendMutex;

    // Serial read / receive buffer mutex — serializes processIncoming()
    // across threads (pollThread, stepGenThread, clockSyncThread, etc.).
    std::mutex m_recvMutex;

    // Batch accumulator for command payloads
    std::vector<uint8_t> m_batchBuf;

    // SerialQueue (background-threaded serial with clock-gating)
    SerialQueue m_serialQueue;

    // Async clock sync: sentTime for the last get_clock via SerialQueue
    std::atomic<double> m_clockSyncSentTime{0.0};

    // StepperSync: min-heap tracking move queue slot availability
    // Each entry is the clock at which one move queue slot becomes free.
    std::priority_queue<uint64_t, std::vector<uint64_t>, std::greater<uint64_t>> m_stepSyncHeap;
    std::mutex m_stepSyncMutex;
    std::atomic<uint64_t> m_stepSyncTotal{0};   // total queue_step commands
    std::atomic<uint64_t> m_stepSyncGated{0};   // commands gated (avail > 0)

    // Expanded enumerations (fully expanded ranges, e.g. PA0=0, PA1=1, ...)
    std::map<std::string, std::map<std::string, int>> m_expandedEnums;
    void expandEnumerations();

    // Resolve pin names in a command string (e.g. "config_digital_out oid=0 pin=PA3" → "... pin=3")
    std::string resolvePinsInCommand(const std::string& cmd) const;

    // Parse and send a command string like "config_digital_out oid=0 pin=96 value=0"
    bool sendCommandString(const std::string& cmdStr);

    // Internal: check for shutdown/starting responses and handle them
    void checkShutdownResponse(const ParsedResponse& resp);

    // Internal
    bool sendRawFrame(const std::vector<uint8_t>& payload);
    std::vector<uint8_t> encodeCommand(const MessageFormat& fmt,
        const std::map<std::string, int64_t>& intParams,
        const std::map<std::string, std::vector<uint8_t>>& bufParams);
    ParsedResponse decodeResponse(const uint8_t* payload, size_t len);
    void parseMessageFormat(const std::string& formatStr, MessageFormat& fmt);
    bool parseIdentifyData(const std::vector<uint8_t>& data);
};
