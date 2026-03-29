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
    m_config.clear();
    m_configStrings.clear();
    m_identifyJson.clear();
    m_version.clear();
    m_buildVersions.clear();
}

bool KlipperMCU::isConnected() const {
    return m_serial.isOpen();
}

bool KlipperMCU::sendRawFrame(const std::vector<uint8_t>& payload) {
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
