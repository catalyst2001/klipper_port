#pragma once

#include <cstdint>
#include <vector>
#include <string>

// Klipper message protocol constants
constexpr uint8_t  MESSAGE_SYNC = 0x7E;
constexpr uint8_t  MESSAGE_DEST = 0x10;
constexpr uint8_t  MESSAGE_SEQ_MASK = 0x0F;
constexpr int      MESSAGE_HEADER_SIZE = 2;
constexpr int      MESSAGE_TRAILER_SIZE = 3;
constexpr int      MESSAGE_MIN = 5;
constexpr int      MESSAGE_MAX = 64;
constexpr int      MESSAGE_PAYLOAD_MAX = MESSAGE_MAX - MESSAGE_HEADER_SIZE - MESSAGE_TRAILER_SIZE;

// Hard-coded command IDs (before identify)
constexpr uint8_t  CMD_IDENTIFY_RESPONSE = 0;
constexpr uint8_t  CMD_IDENTIFY = 1;

// CRC16-CCITT
uint16_t crc16_ccitt(const uint8_t* buf, size_t len);

// VLQ encoding/decoding
namespace vlq {
    // Encode a uint32 into VLQ bytes, append to output
    void encode_uint32(std::vector<uint8_t>& out, uint32_t v);
    void encode_int32(std::vector<uint8_t>& out, int32_t v);

    // Decode a VLQ integer from buffer at position pos. Returns decoded value and advances pos.
    uint32_t decode_uint32(const uint8_t* data, size_t len, size_t& pos);
    int32_t  decode_int32(const uint8_t* data, size_t len, size_t& pos);

    // Decode a buffer/string: VLQ length + raw bytes
    std::vector<uint8_t> decode_buffer(const uint8_t* data, size_t len, size_t& pos);
}

// Build a raw message frame: length + seq + payload + crc + sync
std::vector<uint8_t> build_message_frame(uint8_t seq, const std::vector<uint8_t>& payload);

// Validate and extract a message from the buffer.
// Returns message length on success, 0 if need more data, negative to skip bytes.
int check_message(bool& needSync, const uint8_t* buf, size_t bufLen);

// Build an "identify" command payload: cmd_id=1, offset=%u, count=%c
std::vector<uint8_t> build_identify_cmd(uint32_t offset, uint8_t count);
