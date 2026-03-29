#include "klipper_proto.h"
#include <cstring>
#include <stdexcept>

// ---- CRC16-CCITT (same as Klipper) ----
uint16_t crc16_ccitt(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t data = buf[i] ^ (crc & 0xFF);
        data ^= (data << 4);
        crc = (((uint16_t)data << 8) | (crc >> 8))
            ^ ((uint8_t)(data >> 4))
            ^ ((uint16_t)data << 3);
    }
    return crc;
}

// ---- VLQ encoding ----
void vlq::encode_uint32(std::vector<uint8_t>& out, uint32_t v) {
    int32_t sv = static_cast<int32_t>(v);
    if (sv >= (3 << 5) || sv < -(1 << 5)) {
        if (sv >= (3 << 12) || sv < -(1 << 12)) {
            if (sv >= (3 << 19) || sv < -(1 << 19)) {
                if (sv >= (3 << 26) || sv < -(1 << 26)) {
                    out.push_back(((v >> 28) & 0x7F) | 0x80);
                }
                out.push_back(((v >> 21) & 0x7F) | 0x80);
            }
            out.push_back(((v >> 14) & 0x7F) | 0x80);
        }
        out.push_back(((v >> 7) & 0x7F) | 0x80);
    }
    out.push_back(v & 0x7F);
}

void vlq::encode_int32(std::vector<uint8_t>& out, int32_t v) {
    encode_uint32(out, static_cast<uint32_t>(v));
}

// ---- VLQ decoding ----
uint32_t vlq::decode_uint32(const uint8_t* data, size_t len, size_t& pos) {
    if (pos >= len) throw std::runtime_error("VLQ decode: out of data");
    uint8_t c = data[pos++];
    uint32_t v = c & 0x7F;
    if ((c & 0x60) == 0x60) {
        v |= static_cast<uint32_t>(-0x20);  // sign extension
    }
    while (c & 0x80) {
        if (pos >= len) throw std::runtime_error("VLQ decode: out of data");
        c = data[pos++];
        v = (v << 7) | (c & 0x7F);
    }
    return v;
}

int32_t vlq::decode_int32(const uint8_t* data, size_t len, size_t& pos) {
    return static_cast<int32_t>(decode_uint32(data, len, pos));
}

std::vector<uint8_t> vlq::decode_buffer(const uint8_t* data, size_t len, size_t& pos) {
    uint32_t bufLen = decode_uint32(data, len, pos);
    if (pos + bufLen > len) throw std::runtime_error("VLQ decode_buffer: out of data");
    std::vector<uint8_t> result(data + pos, data + pos + bufLen);
    pos += bufLen;
    return result;
}

// ---- Message framing ----
std::vector<uint8_t> build_message_frame(uint8_t seq, const std::vector<uint8_t>& payload) {
    size_t msgLen = MESSAGE_HEADER_SIZE + payload.size() + MESSAGE_TRAILER_SIZE;
    if (msgLen > MESSAGE_MAX) {
        throw std::runtime_error("Message too large");
    }

    std::vector<uint8_t> msg;
    msg.reserve(msgLen);

    // Header
    msg.push_back(static_cast<uint8_t>(msgLen));
    msg.push_back((seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST);

    // Payload
    msg.insert(msg.end(), payload.begin(), payload.end());

    // CRC16 over header + payload
    uint16_t crc = crc16_ccitt(msg.data(), msg.size());
    msg.push_back(static_cast<uint8_t>(crc >> 8));
    msg.push_back(static_cast<uint8_t>(crc & 0xFF));

    // Sync
    msg.push_back(MESSAGE_SYNC);

    return msg;
}

int check_message(bool& needSync, const uint8_t* buf, size_t bufLen) {
    if (bufLen < MESSAGE_MIN) return 0;

    if (needSync) {
        // Scan for sync byte
        for (size_t i = 0; i < bufLen; i++) {
            if (buf[i] == MESSAGE_SYNC) {
                needSync = false;
                return -static_cast<int>(i + 1);
            }
        }
        return -static_cast<int>(bufLen);
    }

    uint8_t msgLen = buf[0];
    if (msgLen < MESSAGE_MIN || msgLen > MESSAGE_MAX) {
        needSync = true;
        return -1;
    }

    uint8_t msgSeq = buf[1];
    if ((msgSeq & ~MESSAGE_SEQ_MASK) != MESSAGE_DEST) {
        needSync = true;
        return -1;
    }

    if (bufLen < msgLen) return 0; // need more data

    if (buf[msgLen - 1] != MESSAGE_SYNC) {
        needSync = true;
        return -1;
    }

    uint16_t expectedCrc = (buf[msgLen - 3] << 8) | buf[msgLen - 2];
    uint16_t calcCrc = crc16_ccitt(buf, msgLen - MESSAGE_TRAILER_SIZE);
    if (calcCrc != expectedCrc) {
        needSync = true;
        return -1;
    }

    return msgLen;
}

// ---- Build identify command ----
std::vector<uint8_t> build_identify_cmd(uint32_t offset, uint8_t count) {
    std::vector<uint8_t> payload;
    vlq::encode_uint32(payload, CMD_IDENTIFY);   // command id = 1
    vlq::encode_uint32(payload, offset);          // offset=%u
    vlq::encode_uint32(payload, count);           // count=%c
    return payload;
}
