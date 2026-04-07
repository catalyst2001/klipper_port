#include "klipper_mcu.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <regex>

// miniz for decompressing identify data (zlib-compatible, single file)
#include "third_party/miniz.h"

// JSON parsing
#include "third_party/nlohmann/json.hpp"
using json = nlohmann::json;

KlipperMCU::KlipperMCU() {
    m_recvBuf.reserve(4096);
}

KlipperMCU::~KlipperMCU() {
    disconnect();
}

bool KlipperMCU::connect(const std::string& port, uint32_t baudRate) {
    disconnect();
    m_portName = port;
    m_baudRate = baudRate;
    if (!m_serial.open(port, baudRate)) {
        m_lastError = "Failed to open serial port: " + port;
        return false;
    }
    m_sendSeq = 1;
    m_recvSeq = 1;
    m_needSync = true;
    m_recvBuf.clear();

    // Toggle DTR to reset MCU's serial state
    m_serial.setDTR(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    m_serial.setDTR(true);
    m_serial.setRTS(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Purge any stale data
    m_serial.purge();
    uint8_t dumpBuf[4096];
    m_serial.read(dumpBuf, sizeof(dumpBuf), 200);

    return true;
}

void KlipperMCU::disconnect() {
    m_serial.close();
    m_commands.clear();
    m_responses.clear();
    m_responseById.clear();
    m_outputById.clear();
    m_enumerations.clear();
    m_expandedEnums.clear();
    m_config.clear();
    m_configStrings.clear();
    m_identifyJson.clear();
    m_version.clear();
    m_buildVersions.clear();
    m_isShutdown = false;
    m_shutdownMsg.clear();
    resetConfig();
}

bool KlipperMCU::isConnected() const {
    return m_serial.isOpen();
}

bool KlipperMCU::sendRawFrame(const std::vector<uint8_t>& payload) {
    std::lock_guard<std::mutex> lock(m_sendMutex);
    auto frame = build_message_frame(m_sendSeq, payload);
    m_sendSeq = (m_sendSeq + 1) & MESSAGE_SEQ_MASK;

    int written = m_serial.write(frame.data(), frame.size());
    if (written != static_cast<int>(frame.size())) {
        m_lastError = "Failed to write frame";
        return false;
    }
    return true;
}

bool KlipperMCU::identify() {
    if (!isConnected()) {
        m_lastError = "Not connected";
        return false;
    }

    std::cout << "[KlipperMCU] Starting identify handshake..." << std::endl;

    m_needSync = true;
    m_recvBuf.clear();

    // Purge any stale data
    m_serial.purge();
    uint8_t dumpBuf[4096];
    m_serial.read(dumpBuf, sizeof(dumpBuf), 100);

    // Phase 1: Find the correct sequence number by trying all 16 possibilities
    // MCU remembers its expected seq from previous sessions
    std::vector<uint8_t> identifyData;
    const uint8_t chunkSize = 40;
    bool seqSynced = false;

    auto payload = build_identify_cmd(0, chunkSize);

    std::cout << "[KlipperMCU] Scanning for correct sequence number..." << std::endl;
    for (int seqTry = 0; seqTry < 16 && !seqSynced; seqTry++) {
        m_sendSeq = static_cast<uint8_t>(seqTry);
        
        auto frame = build_message_frame(m_sendSeq, payload);
        m_sendSeq = (m_sendSeq + 1) & MESSAGE_SEQ_MASK;

        std::cout << "  Trying seq=" << seqTry << "..." << std::endl;

        m_serial.write(frame.data(), static_cast<int>(frame.size()));
        
        // Read response
        auto responses = processIncoming(50);
        for (auto& resp : responses) {
            if (resp.msgId == CMD_IDENTIFY_RESPONSE) {
                auto dit = resp.bufParams.find("data");
                auto oit = resp.intParams.find("offset");
                if (dit != resp.bufParams.end() && oit != resp.intParams.end()) {
                    if (oit->second == 0 && !dit->second.empty()) {
                        identifyData.insert(identifyData.end(), dit->second.begin(), dit->second.end());
                        seqSynced = true;
                        std::cout << "[KlipperMCU] Sequence synced at seq=" << seqTry 
                                  << ", got " << dit->second.size() << " bytes" << std::endl;
                    }
                }
            }
        }
    }

    if (!seqSynced) {
        m_lastError = "Failed to sync sequence with MCU after trying all 16 sequences";
        return false;
    }

    // Phase 2: Continue reading identify data chunks
    while (true) {
        uint32_t offset = static_cast<uint32_t>(identifyData.size());
        auto chunkPayload = build_identify_cmd(offset, chunkSize);
        
        bool gotResponse = false;
        for (int retry = 0; retry < 5; retry++) {
            if (!sendRawFrame(chunkPayload)) {
                m_lastError = "Failed to send identify command";
                return false;
            }

            uint32_t waitMs = 20u << retry;
            auto startTime = std::chrono::steady_clock::now();
            while (true) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - startTime).count();
                if (elapsed >= waitMs) break;

                auto responses = processIncoming(static_cast<uint32_t>(waitMs - elapsed));
                for (auto& resp : responses) {
                    if (resp.msgId == CMD_IDENTIFY_RESPONSE) {
                        auto it = resp.intParams.find("offset");
                        auto dit = resp.bufParams.find("data");
                        if (it != resp.intParams.end() && dit != resp.bufParams.end()) {
                            uint32_t respOffset = static_cast<uint32_t>(it->second);
                            if (respOffset == identifyData.size()) {
                                if (dit->second.empty()) {
                                    std::cout << "[KlipperMCU] Identify data received: " 
                                              << identifyData.size() << " bytes (compressed)" << std::endl;
                                    return parseIdentifyData(identifyData);
                                }
                                identifyData.insert(identifyData.end(), 
                                                    dit->second.begin(), dit->second.end());
                                gotResponse = true;
                            }
                        }
                    }
                }
                if (gotResponse) break;
            }
            if (gotResponse) break;
        }

        if (!gotResponse) {
            m_lastError = "Identify response timeout at offset " + std::to_string(identifyData.size());
            return false;
        }
    }
}

std::vector<KlipperMCU::ParsedResponse> KlipperMCU::processIncoming(uint32_t timeoutMs) {
    std::lock_guard<std::mutex> recvLock(m_recvMutex);
    std::vector<ParsedResponse> results;

    uint8_t tmpBuf[256];
    int bytesRead = m_serial.read(tmpBuf, sizeof(tmpBuf), timeoutMs);
    if (bytesRead > 0) {
        m_recvBuf.insert(m_recvBuf.end(), tmpBuf, tmpBuf + bytesRead);
    }

    while (!m_recvBuf.empty()) {
        int result = check_message(m_needSync, m_recvBuf.data(), m_recvBuf.size());

        if (result == 0) {
            break; // need more data
        }
        else if (result < 0) {
            // Skip bytes
            size_t skip = static_cast<size_t>(-result);
            if (skip >= m_recvBuf.size()) {
                m_recvBuf.clear();
            }
            else {
                m_recvBuf.erase(m_recvBuf.begin(), m_recvBuf.begin() + skip);
            }
        }
        else {
            // Valid message of 'result' bytes
            size_t msgLen = static_cast<size_t>(result);
            uint8_t seq = m_recvBuf[1] & MESSAGE_SEQ_MASK;

            // Extract payload
            const uint8_t* payloadStart = m_recvBuf.data() + MESSAGE_HEADER_SIZE;
            size_t payloadLen = msgLen - MESSAGE_HEADER_SIZE - MESSAGE_TRAILER_SIZE;

            if (payloadLen > 0) {
                auto resp = decodeResponse(payloadStart, payloadLen);

                // Check for shutdown/starting responses
                checkShutdownResponse(resp);

                // Dispatch to OID response handlers
                if (!resp.name.empty()) {
                    auto oidIt = resp.intParams.find("oid");
                    if (oidIt != resp.intParams.end()) {
                        std::string key = resp.name + ":" + std::to_string(oidIt->second);
                        auto handlerIt = m_oidHandlers.find(key);
                        if (handlerIt != m_oidHandlers.end()) {
                            handlerIt->second(resp);
                        }
                    }
                }

                results.push_back(std::move(resp));

                if (m_responseCallback && !results.empty()) {
                    auto& r = results.back();
                    m_responseCallback(r.msgId, r.name, r.intParams, r.bufParams);
                }
            }

            m_recvBuf.erase(m_recvBuf.begin(), m_recvBuf.begin() + msgLen);
        }
    }

    return results;
}

KlipperMCU::ParsedResponse KlipperMCU::decodeResponse(const uint8_t* payload, size_t len) {
    ParsedResponse resp;
    size_t pos = 0;

    // A message block can contain multiple commands concatenated
    // For now, parse the first one
    if (pos < len) {
        uint32_t cmdId = vlq::decode_uint32(payload, len, pos);
        resp.msgId = static_cast<int32_t>(cmdId);

        // Check if we know this response
        auto it = m_responseById.find(resp.msgId);
        if (it != m_responseById.end()) {
            resp.name = it->second->name;
            // Decode parameters based on format
            for (auto& param : it->second->params) {
                if (param.type == 's') {
                    auto buf = vlq::decode_buffer(payload, len, pos);
                    resp.bufParams[param.name] = std::move(buf);
                }
                else {
                    int64_t val = static_cast<int64_t>(vlq::decode_int32(payload, len, pos));
                    resp.intParams[param.name] = val;
                }
            }
        }
        else if (resp.msgId == CMD_IDENTIFY_RESPONSE) {
            // Hard-coded: identify_response offset=%u data=%.*s
            resp.name = "identify_response";
            resp.intParams["offset"] = static_cast<int64_t>(vlq::decode_uint32(payload, len, pos));
            resp.bufParams["data"] = vlq::decode_buffer(payload, len, pos);
        }
        else {
            resp.name = "unknown_" + std::to_string(resp.msgId);
            // Try to consume remaining bytes as raw data
            if (pos < len) {
                resp.bufParams["raw"] = std::vector<uint8_t>(payload + pos, payload + len);
            }
        }
    }

    return resp;
}

void KlipperMCU::parseMessageFormat(const std::string& formatStr, MessageFormat& fmt) {
    fmt.formatStr = formatStr;
    fmt.params.clear();

    // Parse "command_name param1=%type param2=%type ..."
    std::istringstream iss(formatStr);
    std::string token;
    iss >> fmt.name; // first token is the command name

    while (iss >> token) {
        auto eqPos = token.find('=');
        if (eqPos == std::string::npos) continue;

        MessageFormat::Param p;
        p.name = token.substr(0, eqPos);
        std::string typeStr = token.substr(eqPos + 1);

        if (typeStr == "%u" || typeStr == "%hu") {
            p.type = 'u';
        }
        else if (typeStr == "%i" || typeStr == "%hi") {
            p.type = 'i';
        }
        else if (typeStr == "%c") {
            p.type = 'c';
        }
        else if (typeStr == "%s" || typeStr == "%.*s" || typeStr == "%*s") {
            p.type = 's';
        }
        else {
            p.type = 'u'; // default
        }

        fmt.params.push_back(std::move(p));
    }
}

bool KlipperMCU::parseIdentifyData(const std::vector<uint8_t>& compressedData) {
    // Decompress with zlib
    std::vector<uint8_t> decompressed;
    decompressed.resize(compressedData.size() * 20); // generous initial size

    z_stream strm = {};
    strm.next_in = const_cast<uint8_t*>(compressedData.data());
    strm.avail_in = static_cast<uInt>(compressedData.size());
    strm.next_out = decompressed.data();
    strm.avail_out = static_cast<uInt>(decompressed.size());

    if (inflateInit(&strm) != Z_OK) {
        m_lastError = "zlib inflateInit failed";
        return false;
    }

    int ret = inflate(&strm, Z_FINISH);
    if (ret != Z_STREAM_END) {
        // Try with larger buffer
        inflateEnd(&strm);
        decompressed.resize(compressedData.size() * 100);
        strm = {};
        strm.next_in = const_cast<uint8_t*>(compressedData.data());
        strm.avail_in = static_cast<uInt>(compressedData.size());
        strm.next_out = decompressed.data();
        strm.avail_out = static_cast<uInt>(decompressed.size());
        inflateInit(&strm);
        ret = inflate(&strm, Z_FINISH);
        if (ret != Z_STREAM_END) {
            inflateEnd(&strm);
            m_lastError = "zlib inflate failed: " + std::to_string(ret);
            return false;
        }
    }

    size_t decompressedSize = strm.total_out;
    inflateEnd(&strm);
    decompressed.resize(decompressedSize);

    m_identifyJson = std::string(decompressed.begin(), decompressed.end());
    std::cout << "[KlipperMCU] Identify JSON (" << decompressedSize << " bytes):" << std::endl;
    std::cout << m_identifyJson << std::endl;

    // Parse JSON
    try {
        json j = json::parse(m_identifyJson);

        // Version
        if (j.contains("version")) {
            m_version = j["version"].get<std::string>();
        }
        if (j.contains("build_versions")) {
            m_buildVersions = j["build_versions"].get<std::string>();
        }

        // Commands
        if (j.contains("commands")) {
            for (auto& [formatStr, msgId] : j["commands"].items()) {
                MessageFormat fmt;
                fmt.msgId = msgId.get<int32_t>();
                parseMessageFormat(formatStr, fmt);
                m_commands[fmt.name] = fmt;
            }
        }

        // Responses
        if (j.contains("responses")) {
            for (auto& [formatStr, msgId] : j["responses"].items()) {
                MessageFormat fmt;
                fmt.msgId = msgId.get<int32_t>();
                parseMessageFormat(formatStr, fmt);
                m_responses[fmt.name] = fmt;
                m_responseById[fmt.msgId] = &m_responses[fmt.name];
            }
        }

        // Output messages (treated same as responses)
        if (j.contains("output")) {
            for (auto& [formatStr, msgId] : j["output"].items()) {
                MessageFormat fmt;
                fmt.msgId = msgId.get<int32_t>();
                parseMessageFormat(formatStr, fmt);
                m_responses[fmt.name] = fmt;
                m_outputById[fmt.msgId] = &m_responses[fmt.name];
                m_responseById[fmt.msgId] = &m_responses[fmt.name];
            }
        }

        // Enumerations
        if (j.contains("enumerations")) {
            for (auto& [enumName, values] : j["enumerations"].items()) {
                std::map<std::string, EnumValue> enumMap;
                for (auto& [valName, valData] : values.items()) {
                    EnumValue ev;
                    if (valData.is_array() && valData.size() == 2) {
                        ev.value = valData[0].get<int>();
                        ev.count = valData[1].get<int>();
                    }
                    else if (valData.is_number_integer()) {
                        ev.value = valData.get<int>();
                        ev.count = 0;
                    }
                    enumMap[valName] = ev;
                }
                m_enumerations[enumName] = std::move(enumMap);
            }
        }

        // Config (can be int or string)
        if (j.contains("config")) {
            for (auto& [key, val] : j["config"].items()) {
                if (val.is_number_integer()) {
                    m_config[key] = val.get<int>();
                }
                else if (val.is_string()) {
                    m_configStrings[key] = val.get<std::string>();
                }
            }
        }

        std::cout << "[KlipperMCU] Identified: " << m_version << std::endl;
        std::cout << "[KlipperMCU] Commands: " << m_commands.size() 
                  << ", Responses: " << m_responses.size() << std::endl;

        // Expand enumeration ranges for pin resolution
        expandEnumerations();

        return true;
    }
    catch (const std::exception& e) {
        m_lastError = std::string("JSON parse error: ") + e.what();
        return false;
    }
}

std::vector<uint8_t> KlipperMCU::encodeCommand(const MessageFormat& fmt,
    const std::map<std::string, int64_t>& intParams,
    const std::map<std::string, std::vector<uint8_t>>& bufParams) {
    
    std::vector<uint8_t> payload;
    vlq::encode_int32(payload, fmt.msgId);

    for (auto& param : fmt.params) {
        if (param.type == 's') {
            auto it = bufParams.find(param.name);
            if (it != bufParams.end()) {
                vlq::encode_uint32(payload, static_cast<uint32_t>(it->second.size()));
                payload.insert(payload.end(), it->second.begin(), it->second.end());
            }
            else {
                vlq::encode_uint32(payload, 0); // empty buffer
            }
        }
        else {
            auto it = intParams.find(param.name);
            int64_t val = (it != intParams.end()) ? it->second : 0;
            vlq::encode_int32(payload, static_cast<int32_t>(val));
        }
    }

    return payload;
}

bool KlipperMCU::sendCommand(const std::string& cmdName,
    const std::map<std::string, int64_t>& intParams,
    const std::map<std::string, std::vector<uint8_t>>& bufParams) {
    
    auto it = m_commands.find(cmdName);
    if (it == m_commands.end()) {
        m_lastError = "Unknown command: " + cmdName;
        return false;
    }

    auto payload = encodeCommand(it->second, intParams, bufParams);
    return sendRawFrame(payload);
}

bool KlipperMCU::sendWithResponse(const std::string& cmdName,
    const std::string& responseName,
    std::map<std::string, int64_t>& outIntParams,
    std::map<std::string, std::vector<uint8_t>>& outBufParams,
    const std::map<std::string, int64_t>& intParams,
    const std::map<std::string, std::vector<uint8_t>>& bufParams,
    uint32_t timeoutMs) {
    
    if (!sendCommand(cmdName, intParams, bufParams)) {
        return false;
    }

    auto startTime = std::chrono::steady_clock::now();
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed >= timeoutMs) {
            m_lastError = "Response timeout for: " + responseName;
            return false;
        }

        auto responses = processIncoming(100);
        for (auto& resp : responses) {
            if (resp.name == responseName) {
                outIntParams = std::move(resp.intParams);
                outBufParams = std::move(resp.bufParams);
                return true;
            }
        }
    }
}

// ============================================================
// Clock Sync
// ============================================================

bool KlipperMCU::initClockSync() {
    return m_clockSync.connect(*this);
}

bool KlipperMCU::clockSyncPoll() {
    if (!isConnected() || m_isShutdown) return false;

    m_clockSync.incrementPending();

    auto sentTime = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    std::map<std::string, int64_t> intP;
    std::map<std::string, std::vector<uint8_t>> bufP;
    if (!sendWithResponse("get_clock", "clock", intP, bufP, {}, {}, 2000)) {
        return m_clockSync.isActive();
    }

    auto recvTime = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    uint32_t clock32 = static_cast<uint32_t>(intP["clock"]);
    m_clockSync.handleClockResponse(clock32, sentTime, recvTime);

    return true;
}

void KlipperMCU::clockSyncPollAsync() {
    if (!isConnected() || m_isShutdown) return;

    m_clockSync.incrementPending();

    // Record sent time before send (used by receive callback to feed ClockSync)
    double sentTime = MonotonicClock::now();
    m_clockSyncSentTime.store(sentTime, std::memory_order_release);

    // Encode get_clock and submit via SerialQueue default queue (no clock gating)
    auto payload = encodeCommandPayload("get_clock");
    if (!payload.empty()) {
        m_serialQueue.send(m_serialQueue.getDefaultCommandQueue(),
                           payload.data(), static_cast<int>(payload.size()),
                           0, 0);
    }
}

// ============================================================
// OID Management
// ============================================================

int KlipperMCU::createOid() {
    if (m_configFinalized) {
        m_lastError = "Cannot create OID after config finalization";
        return -1;
    }
    return m_oidCount++;
}

void KlipperMCU::addConfigCmd(const std::string& cmd) {
    m_configCmds.push_back(cmd);
}

void KlipperMCU::addRestartCmd(const std::string& cmd) {
    m_restartCmds.push_back(cmd);
}

void KlipperMCU::addInitCmd(const std::string& cmd) {
    m_initCmds.push_back(cmd);
}

void KlipperMCU::resetConfig() {
    m_oidCount = 0;
    m_moveQueueSlots = 0;
    m_configFinalized = false;
    m_configCmds.clear();
    m_restartCmds.clear();
    m_initCmds.clear();
    m_oidHandlers.clear();
}

int64_t KlipperMCU::secondsToClock(double seconds) const {
    // Use nominal CLOCK_FREQ for deterministic tick values (matches Python Klipper).
    // The clock sync estimated frequency varies between runs and would cause
    // CRC mismatches when the MCU already has a stored config.
    auto it = m_config.find("CLOCK_FREQ");
    double freq = (it != m_config.end()) ? static_cast<double>(it->second)
                                         : m_clockSync.getEstimatedFreq();
    return static_cast<int64_t>(seconds * freq);
}

int KlipperMCU::getConstantInt(const std::string& name, int defaultVal) const {
    auto it = m_config.find(name);
    return (it != m_config.end()) ? it->second : defaultVal;
}

double KlipperMCU::getConstantFloat(const std::string& name, double defaultVal) const {
    auto it = m_config.find(name);
    return (it != m_config.end()) ? static_cast<double>(it->second) : defaultVal;
}

void KlipperMCU::registerOidResponse(const std::string& responseName, int oid, OidResponseHandler handler) {
    std::string key = responseName + ":" + std::to_string(oid);
    m_oidHandlers[key] = std::move(handler);
}

void KlipperMCU::requestMoveQueueSlot() {
    m_moveQueueSlots++;
}

// ---- Command Batching ----

std::vector<uint8_t> KlipperMCU::encodeCommandPayload(const std::string& cmdName,
    const std::map<std::string, int64_t>& intParams,
    const std::map<std::string, std::vector<uint8_t>>& bufParams) {
    auto it = m_commands.find(cmdName);
    if (it == m_commands.end()) return {};
    return encodeCommand(it->second, intParams, bufParams);
}

bool KlipperMCU::queuePayload(const std::vector<uint8_t>& payload) {
    // If adding this payload would exceed max, flush the current batch first
    if (!m_batchBuf.empty() &&
        m_batchBuf.size() + payload.size() > MESSAGE_PAYLOAD_MAX) {
        if (!flushBatch()) return false;
    }
    m_batchBuf.insert(m_batchBuf.end(), payload.begin(), payload.end());
    return true;
}

bool KlipperMCU::flushBatch() {
    if (m_batchBuf.empty()) return true;
    std::vector<uint8_t> buf;
    buf.swap(m_batchBuf);
    return sendRawFrame(buf);
}

bool KlipperMCU::finalizeConfig() {
    if (m_configFinalized) {
        m_lastError = "Config already finalized";
        return false;
    }

    // Step 1: Query MCU's current config state
    std::map<std::string, int64_t> configParams;
    std::map<std::string, std::vector<uint8_t>> bufP;
    if (!sendWithResponse("get_config", "config", configParams, bufP)) {
        m_lastError = "Failed to query MCU config state";
        return false;
    }

    bool isConfig = configParams["is_config"] != 0;
    uint32_t mcuCrc = static_cast<uint32_t>(configParams["crc"]);

    // Step 1.5: If MCU is in shutdown, clear it and force full reset.
    //           Stepper state (s->count) may survive clear_shutdown.
    bool needsRestart = false;
    if (configParams["is_shutdown"] != 0) {
        std::cout << "[KlipperMCU] MCU is in shutdown state, clearing..." << std::endl;
        // Send clear_shutdown and re-query config
        if (!sendCommand("clear_shutdown")) {
            m_lastError = "Failed to send clear_shutdown";
            return false;
        }
        m_isShutdown = false;
        {
            std::lock_guard<std::mutex> lock(m_shutdownMutex);
            m_shutdownMsg.clear();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Re-query config state after clearing shutdown
        configParams.clear();
        if (!sendWithResponse("get_config", "config", configParams, bufP)) {
            m_lastError = "Failed to re-query config after clear_shutdown";
            return false;
        }
        isConfig = configParams["is_config"] != 0;
        mcuCrc = static_cast<uint32_t>(configParams["crc"]);

        if (configParams["is_shutdown"] != 0) {
            m_lastError = "MCU still in shutdown after clear_shutdown";
            return false;
        }
        std::cout << "[KlipperMCU] Shutdown cleared, is_config=" << isConfig
                  << " crc=" << mcuCrc << std::endl;
        // Always do a full reset after shutdown — stepper state (s->count)
        // may be non-zero even when move_count == 0, causing
        // "Can't reset time when stepper active" during homing.
        needsRestart = true;
    }

    if (!needsRestart && configParams.count("move_count") && configParams["move_count"] > 0) {
        std::cout << "[KlipperMCU] Stale MCU state detected (move_count="
                  << configParams["move_count"] << "), resetting..." << std::endl;
        needsRestart = true;
    }

    if (needsRestart) {
        // Send reset to MCU
        sendCommand("reset");
        m_isShutdown = false;
        {
            std::lock_guard<std::mutex> lock(m_shutdownMutex);
            m_shutdownMsg.clear();
        }

        // Close serial port (USB CDC drops on MCU reset)
        m_serial.close();

        // Wait for MCU to reboot and USB to re-enumerate
        std::this_thread::sleep_for(std::chrono::milliseconds(3500));

        // Reopen serial port
        if (!m_serial.open(m_portName, m_baudRate)) {
            m_lastError = "Failed to reopen serial port after MCU reset: " + m_portName;
            return false;
        }

        // DTR toggle + purge
        m_serial.setDTR(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m_serial.setDTR(true);
        m_serial.setRTS(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m_serial.purge();
        uint8_t dumpBuf[4096];
        m_serial.read(dumpBuf, sizeof(dumpBuf), 300);

        m_needSync = true;
        m_recvBuf.clear();

        // Scan for correct sequence number (MCU may be at any seq after reboot)
        bool seqSynced = false;
        auto getCfgIt = m_commands.find("get_config");
        if (getCfgIt == m_commands.end()) {
            m_lastError = "get_config command not found after reset";
            return false;
        }

        std::vector<uint8_t> probe = encodeCommand(getCfgIt->second, {}, {});

        std::cout << "[KlipperMCU] Scanning for sequence number after reset..." << std::endl;
        for (int seqTry = 0; seqTry < 16 && !seqSynced; seqTry++) {
            m_sendSeq = static_cast<uint8_t>(seqTry);

            auto frame = build_message_frame(m_sendSeq, probe);
            m_sendSeq = (m_sendSeq + 1) & MESSAGE_SEQ_MASK;

            m_serial.write(frame.data(), static_cast<int>(frame.size()));

            auto responses = processIncoming(80);
            for (auto& resp : responses) {
                if (resp.name == "config") {
                    configParams = std::move(resp.intParams);
                    seqSynced = true;
                    std::cout << "[KlipperMCU] Sequence synced at seq=" << seqTry << std::endl;
                    break;
                }
            }
        }

        if (!seqSynced) {
            m_lastError = "Failed to re-sync with MCU after reset (tried all 16 sequences)";
            return false;
        }

        isConfig = configParams["is_config"] != 0;
        mcuCrc = static_cast<uint32_t>(configParams["crc"]);

        if (configParams["is_shutdown"] != 0) {
            m_lastError = "MCU still in shutdown after reset";
            return false;
        }
        std::cout << "[KlipperMCU] MCU reset successful, is_config=" 
                  << isConfig << std::endl;
    }

    // Step 2: Prepare config commands (work on copies to allow retry)
    std::vector<std::string> cfgCmds = m_configCmds;
    std::vector<std::string> rstCmds = m_restartCmds;
    std::vector<std::string> initCmds = m_initCmds;

    // Prepend allocate_oids as first config command
    cfgCmds.insert(cfgCmds.begin(),
        "allocate_oids count=" + std::to_string(m_oidCount));

    // Step 3: Resolve pin names in all command lists
    for (auto* cmdList : {&cfgCmds, &rstCmds, &initCmds}) {
        for (auto& cmd : *cmdList) {
            cmd = resolvePinsInCommand(cmd);
        }
    }

    // Step 4: Calculate CRC of config commands
    std::string configStr;
    for (size_t i = 0; i < cfgCmds.size(); i++) {
        if (i > 0) configStr += '\n';
        configStr += cfgCmds[i];
    }
    // Use zlib CRC32 (same as Python's zlib.crc32)
    uint32_t configCrc = static_cast<uint32_t>(
        mz_crc32(MZ_CRC32_INIT,
                  reinterpret_cast<const uint8_t*>(configStr.data()),
                  configStr.size()));

    // Step 5: Append finalize_config
    cfgCmds.push_back("finalize_config crc=" + std::to_string(configCrc));

    // Step 5.5: Update stale clock values in restart/init commands.
    // Commands like queue_digital_out, queue_pwm_out, query_analog_in contain
    // clock=<value> that was computed at buildConfig() time. By now the MCU clock
    // has advanced, so we must replace with a fresh future value.
    {
        std::map<std::string, int64_t> clkP;
        std::map<std::string, std::vector<uint8_t>> clkBufP;
        if (sendWithResponse("get_clock", "clock", clkP, clkBufP, {}, {}, 3000)) {
            uint32_t now32 = static_cast<uint32_t>(clkP["clock"]);
            // Schedule 250ms in the future, stagger each by 10ms
            uint32_t offsetTicks = static_cast<uint32_t>(secondsToClock(0.25));
            uint32_t staggerTicks = static_cast<uint32_t>(secondsToClock(0.01));
            uint32_t nextClock = now32 + offsetTicks;

            auto updateClock = [&](std::string& cmd) {
                const std::string token = "clock=";
                size_t pos = cmd.find(token);
                if (pos == std::string::npos) return;
                size_t numStart = pos + token.length();
                size_t numEnd = cmd.find_first_of(" \t\n", numStart);
                if (numEnd == std::string::npos) numEnd = cmd.length();
                cmd.replace(numStart, numEnd - numStart, std::to_string(nextClock));
                nextClock += staggerTicks;
            };
            for (auto& cmd : rstCmds)  updateClock(cmd);
            for (auto& cmd : initCmds) updateClock(cmd);
        }
    }

    // Step 6: Determine which commands to send
    std::vector<std::string> cmdsToSend;
    if (!isConfig) {
        // MCU not configured → send full config + init
        cmdsToSend.insert(cmdsToSend.end(), cfgCmds.begin(), cfgCmds.end());
        cmdsToSend.insert(cmdsToSend.end(), initCmds.begin(), initCmds.end());
        std::cout << "[KlipperMCU] Sending printer configuration (" 
                  << cmdsToSend.size() << " commands)..." << std::endl;
    }
    else {
        // MCU already configured
        if (configCrc != mcuCrc) {
            // CRC mismatch → firmware restart + retry with new config
            std::cout << "[KlipperMCU] CRC mismatch (host=" << configCrc 
                      << " mcu=" << mcuCrc << "), restarting MCU..." << std::endl;
            
            // Use the same reset + resync path as the stale state handler above
            sendCommand("reset");
            m_isShutdown = false;
            {
                std::lock_guard<std::mutex> lock(m_shutdownMutex);
                m_shutdownMsg.clear();
            }

            // Close serial port (USB CDC drops on MCU reset)
            m_serial.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(3500));

            // Reopen serial port
            if (!m_serial.open(m_portName, m_baudRate)) {
                m_lastError = "Failed to reopen serial port after CRC mismatch restart: " + m_portName;
                return false;
            }

            m_serial.setDTR(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            m_serial.setDTR(true);
            m_serial.setRTS(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            m_serial.purge();
            uint8_t dumpBuf2[4096];
            m_serial.read(dumpBuf2, sizeof(dumpBuf2), 300);

            m_needSync = true;
            m_recvBuf.clear();

            // Re-sync sequence number
            bool seqSynced2 = false;
            auto getCfgIt2 = m_commands.find("get_config");
            if (getCfgIt2 == m_commands.end()) {
                m_lastError = "get_config command not found after CRC mismatch restart";
                return false;
            }
            auto probe2 = encodeCommand(getCfgIt2->second, {}, {});
            for (int seqTry = 0; seqTry < 16 && !seqSynced2; seqTry++) {
                m_sendSeq = static_cast<uint8_t>(seqTry);
                auto frame = build_message_frame(m_sendSeq, probe2);
                m_sendSeq = (m_sendSeq + 1) & MESSAGE_SEQ_MASK;
                m_serial.write(frame.data(), static_cast<int>(frame.size()));
                auto responses2 = processIncoming(80);
                for (auto& resp : responses2) {
                    if (resp.name == "config") {
                        configParams = std::move(resp.intParams);
                        seqSynced2 = true;
                        break;
                    }
                }
            }
            if (!seqSynced2) {
                m_lastError = "Failed to re-sync after CRC mismatch restart";
                return false;
            }

            isConfig = configParams["is_config"] != 0;
            // After reset, MCU is unconfigured — fall through to send full config
            if (isConfig) {
                m_lastError = "MCU still configured after CRC mismatch restart";
                return false;
            }
            cmdsToSend.insert(cmdsToSend.end(), cfgCmds.begin(), cfgCmds.end());
            cmdsToSend.insert(cmdsToSend.end(), initCmds.begin(), initCmds.end());
            std::cout << "[KlipperMCU] Re-sending config after CRC mismatch (" 
                      << cmdsToSend.size() << " commands)..." << std::endl;
        } else {
            // CRC matches → send restart + init commands only
            cmdsToSend.insert(cmdsToSend.end(), rstCmds.begin(), rstCmds.end());
            cmdsToSend.insert(cmdsToSend.end(), initCmds.begin(), initCmds.end());
            std::cout << "[KlipperMCU] MCU already configured (CRC match), sending " 
                      << cmdsToSend.size() << " restart/init commands..." << std::endl;
        }
    }

    // Step 7: Send all commands
    for (auto& cmd : cmdsToSend) {
        if (!sendCommandString(cmd)) {
            m_lastError = "Failed to send config command: " + cmd + " (" + m_lastError + ")";
            return false;
        }
    }

    // Step 8: Verify configuration
    configParams.clear();
    bufP.clear();
    if (!sendWithResponse("get_config", "config", configParams, bufP)) {
        m_lastError = "Failed to verify MCU config";
        return false;
    }

    if (configParams["is_config"] == 0) {
        m_lastError = "MCU did not accept configuration";
        return false;
    }

    m_configFinalized = true;
    m_mcuMoveCount = static_cast<int>(configParams["move_count"]);
    stepSyncReset();
    std::cout << "[KlipperMCU] Configuration finalized (CRC=" << configCrc 
              << ", move_count=" << configParams["move_count"] << ")" << std::endl;
    return true;
}

// ============================================================
// Pin Resolution
// ============================================================

void KlipperMCU::expandEnumerations() {
    m_expandedEnums.clear();
    for (auto& [enumName, values] : m_enumerations) {
        auto& expanded = m_expandedEnums[enumName];
        for (auto& [valName, ev] : values) {
            if (ev.isRange()) {
                // Expand range: e.g. "PA0" with [0, 32] -> PA0=0, PA1=1, ..., PA31=31
                // Find the numeric suffix of the base name
                std::string root = valName;
                int startIdx = 0;
                while (!root.empty() && std::isdigit(root.back())) {
                    root.pop_back();
                }
                if (root.size() < valName.size()) {
                    startIdx = std::stoi(valName.substr(root.size()));
                }
                for (int i = 0; i < ev.count; i++) {
                    expanded[root + std::to_string(startIdx + i)] = ev.value + i;
                }
            }
            else {
                expanded[valName] = ev.value;
            }
        }
    }
}

int KlipperMCU::resolvePin(const std::string& pinName) const {
    return resolveEnum("pin", pinName);
}

int KlipperMCU::resolveEnum(const std::string& enumName, const std::string& valueName) const {
    auto enumIt = m_expandedEnums.find(enumName);
    if (enumIt == m_expandedEnums.end()) return -1;

    auto valIt = enumIt->second.find(valueName);
    if (valIt == enumIt->second.end()) {
        // Try as numeric
        try {
            return std::stoi(valueName);
        }
        catch (...) {
            return -1;
        }
    }
    return valIt->second;
}

std::string KlipperMCU::resolvePinsInCommand(const std::string& cmd) const {
    // Replace pin=NAME patterns with pin=NUMBER
    // Matches: " pin=NAME", "_pin=NAME" (space or underscore before 'pin=')
    std::string result = cmd;
    std::regex pinRegex(R"(([_ ]pin=)([^ ]+))");
    std::string output;
    std::sregex_iterator it(result.begin(), result.end(), pinRegex);
    std::sregex_iterator end;
    size_t lastPos = 0;

    while (it != end) {
        auto& match = *it;
        output += result.substr(lastPos, match.position() - lastPos);

        std::string prefix = match[1].str();
        std::string pinName = match[2].str();

        int pinNum = resolvePin(pinName);
        if (pinNum >= 0) {
            output += prefix + std::to_string(pinNum);
        }
        else {
            output += match[0].str(); // keep as-is if not found
        }

        lastPos = match.position() + match.length();
        ++it;
    }
    output += result.substr(lastPos);
    return output;
}

bool KlipperMCU::sendCommandString(const std::string& cmdStr) {
    // Parse a command string like "config_digital_out oid=0 pin=96 value=0 default_value=0 max_duration=0"
    std::istringstream iss(cmdStr);
    std::string cmdName;
    iss >> cmdName;

    auto cmdIt = m_commands.find(cmdName);
    if (cmdIt == m_commands.end()) {
        m_lastError = "Unknown command in string: " + cmdName;
        return false;
    }

    // Parse key=value pairs
    std::map<std::string, int64_t> intParams;
    std::map<std::string, std::vector<uint8_t>> bufParams;

    std::string token;
    while (iss >> token) {
        auto eqPos = token.find('=');
        if (eqPos == std::string::npos) continue;

        std::string key = token.substr(0, eqPos);
        std::string valStr = token.substr(eqPos + 1);

        // Find param type from format
        char type = 'u';
        for (auto& p : cmdIt->second.params) {
            if (p.name == key) {
                type = p.type;
                break;
            }
        }

        if (type == 's') {
            // Buffer param
            bufParams[key] = std::vector<uint8_t>(valStr.begin(), valStr.end());
        }
        else {
            // Try as enumeration first, then as number
            int64_t val = 0;
            // Check if this parameter has an enumeration
            int enumVal = -1;
            // For pin parameters, use pin enum
            if (key.find("pin") != std::string::npos) {
                enumVal = resolvePin(valStr);
            }
            if (enumVal >= 0) {
                val = enumVal;
            }
            else {
                try {
                    val = std::stoll(valStr);
                }
                catch (...) {
                    m_lastError = "Invalid parameter value: " + key + "=" + valStr;
                    return false;
                }
            }
            intParams[key] = val;
        }
    }

    return sendCommand(cmdName, intParams, bufParams);
}

// ============================================================
// Shutdown / Restart
// ============================================================

void KlipperMCU::checkShutdownResponse(const ParsedResponse& resp) {
    if (resp.name == "shutdown" || resp.name == "is_shutdown") {
        if (m_isShutdown.load()) return;
        m_isShutdown = true;

        // Resolve static_string_id to message
        std::string msg = "Unknown shutdown reason";
        auto idIt = resp.intParams.find("static_string_id");
        if (idIt != resp.intParams.end()) {
            int stringId = static_cast<int>(idIt->second);
            // Look up in static_string_id enumeration
            auto enumIt = m_expandedEnums.find("static_string_id");
            if (enumIt != m_expandedEnums.end()) {
                for (auto& [name, val] : enumIt->second) {
                    if (val == stringId) {
                        msg = name;
                        break;
                    }
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(m_shutdownMutex);
            m_shutdownMsg = msg;
        }

        // Log MCU clock from shutdown response for debugging
        auto clockIt = resp.intParams.find("clock");
        uint32_t shutdownClock32 = clockIt != resp.intParams.end()
            ? static_cast<uint32_t>(clockIt->second) : 0;
        double mcuUptimeSec = static_cast<double>(shutdownClock32) / m_clockSync.getMcuFreq();
        int64_t estClock64 = m_clockSync.getClock();
        double estPrintTime = m_clockSync.estimatedPrintTime();
        std::cerr << "[KlipperMCU] !!! SHUTDOWN: " << msg
                  << " | mcu_clock32=" << shutdownClock32
                  << " mcu_uptime=" << std::fixed << std::setprecision(3) << mcuUptimeSec << "s"
                  << " estClock64=" << estClock64
                  << " estPrintTime=" << std::setprecision(3) << estPrintTime
                  << " !!!" << std::endl;

        if (m_shutdownCallback) {
            m_shutdownCallback(msg);
        }
    }
    else if (resp.name == "starting") {
        // MCU spontaneously restarted
        if (!m_isShutdown.load()) {
            m_isShutdown = true;
            std::string msg = "MCU spontaneous restart";
            {
                std::lock_guard<std::mutex> lock(m_shutdownMutex);
                m_shutdownMsg = msg;
            }
            std::cerr << "[KlipperMCU] !!! " << msg << " !!!" << std::endl;
            if (m_shutdownCallback) {
                m_shutdownCallback(msg);
            }
        }
    }
}

std::string KlipperMCU::getShutdownMsg() const {
    std::lock_guard<std::mutex> lock(m_shutdownMutex);
    return m_shutdownMsg;
}

bool KlipperMCU::clearShutdown() {
    if (!isConnected()) {
        m_lastError = "Not connected";
        return false;
    }

    if (sendCommand("clear_shutdown")) {
        m_isShutdown = false;
        {
            std::lock_guard<std::mutex> lock(m_shutdownMutex);
            m_shutdownMsg.clear();
        }
        return true;
    }
    return false;
}

bool KlipperMCU::firmwareRestart() {
    if (!isConnected()) {
        m_lastError = "Not connected";
        return false;
    }

    // Try reset command first
    if (m_commands.count("reset")) {
        sendCommand("reset");
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        disconnect();
        return true;
    }

    m_lastError = "No reset command available";
    return false;
}

// ======================================================================
// SerialQueue integration
// ======================================================================

bool KlipperMCU::startSerialQueue() {
    if (m_serialQueue.isRunning()) return true;
    if (!m_serial.isOpen()) {
        m_lastError = "Serial port not open";
        return false;
    }

    // Setup receive callback: dispatch responses to OID handlers, shutdown detection,
    // and async clock sync updates.
    m_serialQueue.setReceiveCallback(
        [this](const uint8_t* msg, int len, double sent_time, double receive_time) {
            (void)sent_time;
            if (len <= 0) return;

            // Decode response
            auto resp = decodeResponse(msg, len);
            checkShutdownResponse(resp);

            // Handle async clock sync response
            if (resp.name == "clock") {
                double storedSentTime = m_clockSyncSentTime.load(std::memory_order_acquire);
                if (storedSentTime > 0.0) {
                    uint32_t clock32 = static_cast<uint32_t>(resp.intParams["clock"]);
                    m_clockSync.handleClockResponse(clock32, storedSentTime, receive_time);
                    m_clockSyncSentTime.store(0.0, std::memory_order_release);

                    // Feed updated clock estimate back to SerialQueue
                    auto snap = m_clockSync.getClockSnapshot();
                    SQClockEstimate ce;
                    ce.last_clock = snap.baseClock;
                    ce.conv_clock = snap.baseClock;
                    ce.conv_time = receive_time;
                    ce.est_freq = snap.estFreq;
                    m_serialQueue.updateClockEstimate(ce);
                }
            }

            // OID dispatch
            if (!resp.name.empty()) {
                auto oidIt = resp.intParams.find("oid");
                if (oidIt != resp.intParams.end()) {
                    std::string key = resp.name + ":" + std::to_string(oidIt->second);
                    auto handlerIt = m_oidHandlers.find(key);
                    if (handlerIt != m_oidHandlers.end()) {
                        handlerIt->second(resp);
                    }
                }
            }

            // General callback
            if (m_responseCallback) {
                m_responseCallback(resp.msgId, resp.name, resp.intParams, resp.bufParams);
            }
        });

    // Calculate baud adjust: time per byte at current baud rate
    // baud_adjust = 10 bits per byte / baud_rate (seconds per byte)
    double baudAdjust = 10.0 / m_baudRate;

    // Seed clock estimate from current clock sync state
    {
        auto snap = m_clockSync.getClockSnapshot();
        SQClockEstimate ce;
        ce.last_clock = snap.baseClock;
        ce.conv_clock = snap.baseClock;
        ce.conv_time = MonotonicClock::now();
        ce.est_freq = snap.estFreq;
        m_serialQueue.updateClockEstimate(ce);
    }

    // Pass current protocol sequence numbers so SerialQueue frames
    // are accepted by the MCU (which remembers its expected seq).
    if (!m_serialQueue.start(m_serial, baudAdjust, m_sendSeq, m_recvSeq)) {
        m_lastError = "Failed to start SerialQueue thread";
        return false;
    }
    return true;
}

void KlipperMCU::stopSerialQueue() {
    if (!m_serialQueue.isRunning()) return;
    // Preserve protocol sequence numbers so the direct serial path
    // (used during homing) continues where the SQ left off.
    m_sendSeq = m_serialQueue.currentSendSeq();
    m_recvSeq = m_serialQueue.currentReceiveSeq();
    m_serialQueue.stop();
}

void KlipperMCU::sendTimed(const std::string& cmdName,
                            const std::map<std::string, int64_t>& intParams,
                            uint64_t min_clock, uint64_t req_clock,
                            CommandQueue* cq) {
    auto payload = encodeCommandPayload(cmdName, intParams);
    if (payload.empty()) return;
    sendTimedRaw(payload.data(), static_cast<int>(payload.size()),
                 min_clock, req_clock, cq);
}

void KlipperMCU::sendTimedRaw(const uint8_t* payload, int len,
                               uint64_t min_clock, uint64_t req_clock,
                               CommandQueue* cq) {
    if (!cq) cq = m_serialQueue.getDefaultCommandQueue();
    m_serialQueue.send(cq, payload, len, min_clock, req_clock);
}

// ======================================================================
// StepperSync: move queue flow control
// ======================================================================

uint64_t KlipperMCU::stepSyncAdjustMinClock(uint64_t minClock, uint64_t endClock) {
    std::lock_guard<std::mutex> lk(m_stepSyncMutex);
    if (m_stepSyncHeap.empty()) return minClock;

    // Pop the earliest-available slot
    uint64_t avail = m_stepSyncHeap.top();
    m_stepSyncHeap.pop();

    // Push when this new command's slot becomes free
    m_stepSyncHeap.push(endClock);

    // Return the effective min_clock: can't send before slot is free
    return (avail > minClock) ? avail : minClock;
}

void KlipperMCU::stepSyncReset() {
    std::lock_guard<std::mutex> lk(m_stepSyncMutex);
    while (!m_stepSyncHeap.empty()) m_stepSyncHeap.pop();
    // Reserve slots for non-stepper move pool users (digital_out, PWM, etc.)
    // and a safety margin for SQ clock estimation variance.
    // Python Klipper subtracts _reserved_move_slots here.
    int reserved = 12;  // ~6 for digital_out/PWM + safety margin
    int usable = (m_mcuMoveCount - reserved) / 2;  // use half (conservative)
    if (usable < 64) usable = 64;
    for (int i = 0; i < usable; i++)
        m_stepSyncHeap.push(0);
}
