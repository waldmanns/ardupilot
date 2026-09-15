#include "Vektor_Protocol.h"

#include <AP_Math/crc.h>

#include <string.h>

namespace {

uint16_t bounded_strlen(const char *value, uint16_t max_len)
{
    uint16_t len = 0;
    if (value == nullptr) {
        return 0;
    }
    while (len < max_len && value[len] != '\0') {
        len++;
    }
    return len;
}

uint16_t read_u16_le(const uint8_t *data)
{
    return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
}

uint32_t read_u32_le(const uint8_t *data)
{
    return uint32_t(data[0]) |
           (uint32_t(data[1]) << 8) |
           (uint32_t(data[2]) << 16) |
           (uint32_t(data[3]) << 24);
}

void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = uint8_t(value);
    data[1] = uint8_t(value >> 8);
}

void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = uint8_t(value);
    data[1] = uint8_t(value >> 8);
    data[2] = uint8_t(value >> 16);
    data[3] = uint8_t(value >> 24);
}

} // namespace

namespace Vektor {
namespace Protocol {

uint32_t fnv1a32(const char *path)
{
    uint32_t hash = 0x811C9DC5U;
    if (path == nullptr) {
        return 0;
    }
    while (*path != '\0') {
        hash ^= uint8_t(*path++);
        hash *= 0x01000193U;
    }
    return hash;
}

uint32_t crc32_iso_hdlc(const uint8_t *data, uint32_t length)
{
    if (data == nullptr && length != 0) {
        return 0;
    }
    return ~crc_crc32(~0U, data, length);
}

uint64_t fnv1a64(const uint8_t *data, uint16_t length)
{
    if (data == nullptr && length != 0) {
        return 0;
    }
    return fnv1a64_update(0xCBF29CE484222325ULL, data, length);
}

uint64_t fnv1a64_update(uint64_t hash,
                        const uint8_t *data,
                        uint16_t length)
{
    if (data == nullptr && length != 0) {
        return 0;
    }
    for (uint16_t i = 0; i < length; i++) {
        hash ^= data[i];
        hash *= 0x100000001B3ULL;
    }
    return hash;
}

bool stable_ids_unique_nonzero(const uint32_t *ids, uint16_t count)
{
    if (ids == nullptr && count != 0) {
        return false;
    }
    for (uint16_t i = 0; i < count; i++) {
        if (ids[i] == 0) {
            return false;
        }
        for (uint16_t j = 0; j < i; j++) {
            if (ids[j] == ids[i]) {
                return false;
            }
        }
    }
    return true;
}

uint16_t primitive_size(PrimitiveType type)
{
    switch (type) {
    case PrimitiveType::BOOL:
    case PrimitiveType::U8:
    case PrimitiveType::I8:
        return 1;
    case PrimitiveType::U16:
    case PrimitiveType::I16:
        return 2;
    case PrimitiveType::U32:
    case PrimitiveType::I32:
    case PrimitiveType::FLOAT32:
    case PrimitiveType::ENUM:
        return 4;
    case PrimitiveType::U64:
    case PrimitiveType::I64:
    case PrimitiveType::FLOAT64:
    case PrimitiveType::VECTOR2F:
        return 8;
    case PrimitiveType::VECTOR3F:
        return 12;
    case PrimitiveType::QUATERNIONF:
        return 16;
    case PrimitiveType::INVALID:
    case PrimitiveType::STRING:
    case PrimitiveType::BLOB:
        return 0;
    }
    return 0;
}

bool cobs_encode(const uint8_t *decoded,
                 uint16_t decoded_len,
                 uint8_t *encoded,
                 uint16_t encoded_capacity,
                 uint16_t &encoded_len)
{
    if (encoded == nullptr || encoded_capacity == 0 ||
        (decoded == nullptr && decoded_len != 0)) {
        encoded_len = 0;
        return false;
    }

    uint16_t read_index = 0;
    uint16_t write_index = 1;
    uint16_t code_index = 0;
    uint8_t code = 1;

    while (read_index < decoded_len) {
        if (decoded[read_index] == 0) {
            if (code_index >= encoded_capacity || write_index >= encoded_capacity) {
                encoded_len = 0;
                return false;
            }
            encoded[code_index] = code;
            code_index = write_index++;
            code = 1;
            read_index++;
            continue;
        }

        if (write_index >= encoded_capacity) {
            encoded_len = 0;
            return false;
        }
        encoded[write_index++] = decoded[read_index++];
        code++;

        if (code == 0xFF) {
            if (code_index >= encoded_capacity || write_index >= encoded_capacity) {
                encoded_len = 0;
                return false;
            }
            encoded[code_index] = code;
            code_index = write_index++;
            code = 1;
        }
    }

    if (code_index >= encoded_capacity) {
        encoded_len = 0;
        return false;
    }
    encoded[code_index] = code;
    encoded_len = write_index;
    return true;
}

bool cobs_decode(const uint8_t *encoded,
                 uint16_t encoded_len,
                 uint8_t *decoded,
                 uint16_t decoded_capacity,
                 uint16_t &decoded_len)
{
    if (decoded == nullptr || (encoded == nullptr && encoded_len != 0)) {
        decoded_len = 0;
        return false;
    }
    uint16_t read_index = 0;
    uint16_t write_index = 0;

    while (read_index < encoded_len) {
        const uint8_t code = encoded[read_index++];
        if (code == 0) {
            decoded_len = 0;
            return false;
        }

        for (uint8_t i = 1; i < code; i++) {
            if (read_index >= encoded_len || write_index >= decoded_capacity) {
                decoded_len = 0;
                return false;
            }
            decoded[write_index++] = encoded[read_index++];
        }

        if (code != 0xFF && read_index < encoded_len) {
            if (write_index >= decoded_capacity) {
                decoded_len = 0;
                return false;
            }
            decoded[write_index++] = 0;
        }
    }

    decoded_len = write_index;
    return true;
}

void Parser::reset()
{
    _encoded_len = 0;
    _discard_until_delimiter = false;
}

ParseResult Parser::consume(uint8_t byte, FrameView &frame)
{
    if (byte != DELIMITER) {
        if (_discard_until_delimiter) {
            return ParseResult::NONE;
        }
        if (_encoded_len >= MAX_ENCODED_BODY_SIZE) {
            _discard_until_delimiter = true;
            _encoded_len = 0;
            return ParseResult::ENCODED_OVERFLOW;
        }
        _encoded[_encoded_len++] = byte;
        return ParseResult::NONE;
    }

    if (_discard_until_delimiter) {
        reset();
        return ParseResult::NONE;
    }

    if (_encoded_len == 0) {
        return ParseResult::EMPTY;
    }

    const ParseResult result = finish_frame(frame);
    _encoded_len = 0;
    return result;
}

ParseResult Parser::finish_frame(FrameView &frame)
{
    uint16_t decoded_len = 0;
    if (!cobs_decode(_encoded,
                     _encoded_len,
                     _decoded,
                     MAX_DECODED_FRAME_SIZE,
                     decoded_len)) {
        return ParseResult::BAD_COBS;
    }

    if (decoded_len < MIN_FRAME_SIZE) {
        return ParseResult::BAD_LENGTH;
    }

    const uint16_t payload_len = read_u16_le(&_decoded[5]);
    const uint16_t expected_len = HEADER_SIZE + payload_len + CRC_SIZE;
    if (payload_len > MAX_PAYLOAD_SIZE || decoded_len != expected_len) {
        return ParseResult::BAD_LENGTH;
    }

    if (_decoded[0] != MAJOR_VERSION) {
        return ParseResult::BAD_VERSION;
    }

    const uint32_t received_crc = read_u32_le(&_decoded[decoded_len - CRC_SIZE]);
    const uint32_t computed_crc = crc32_iso_hdlc(_decoded, decoded_len - CRC_SIZE);
    if (received_crc != computed_crc) {
        return ParseResult::BAD_CRC;
    }

    frame.version = _decoded[0];
    frame.message_type = MessageType(_decoded[1]);
    frame.flags = _decoded[2];
    frame.sequence = read_u16_le(&_decoded[3]);
    frame.payload = &_decoded[HEADER_SIZE];
    frame.payload_len = payload_len;
    return ParseResult::FRAME;
}

PayloadWriter::PayloadWriter(uint8_t *buffer, uint16_t capacity) :
    _buffer(buffer),
    _capacity(capacity)
{
    if (buffer == nullptr && capacity != 0) {
        _ok = false;
    }
}

bool PayloadWriter::reserve(uint16_t count)
{
    if (!_ok || count > _capacity || _length > _capacity - count) {
        _ok = false;
        return false;
    }
    return true;
}

bool PayloadWriter::u8(uint8_t value)
{
    if (!reserve(1)) {
        return false;
    }
    _buffer[_length++] = value;
    return true;
}

bool PayloadWriter::u16(uint16_t value)
{
    if (!reserve(2)) {
        return false;
    }
    write_u16_le(&_buffer[_length], value);
    _length += 2;
    return true;
}

bool PayloadWriter::u32(uint32_t value)
{
    if (!reserve(4)) {
        return false;
    }
    write_u32_le(&_buffer[_length], value);
    _length += 4;
    return true;
}

bool PayloadWriter::u64(uint64_t value)
{
    if (!reserve(8)) {
        return false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        _buffer[_length + i] = uint8_t(value >> (8 * i));
    }
    _length += 8;
    return true;
}

bool PayloadWriter::bytes(const uint8_t *data, uint16_t data_len)
{
    if ((data == nullptr && data_len != 0) || !reserve(data_len)) {
        _ok = false;
        return false;
    }
    if (data_len != 0 && data != nullptr) {
        memcpy(&_buffer[_length], data, data_len);
    }
    _length += data_len;
    return true;
}

bool PayloadWriter::str8(const char *value)
{
    if (value == nullptr) {
        _ok = false;
        return false;
    }
    const uint16_t len = bounded_strlen(value, 255);
    if (len > 255 || !u8(uint8_t(len))) {
        _ok = false;
        return false;
    }
    return bytes(reinterpret_cast<const uint8_t *>(value), len);
}

bool PayloadWriter::str16(const char *value)
{
    if (value == nullptr) {
        _ok = false;
        return false;
    }
    const uint16_t len = bounded_strlen(value, 65535);
    if (!u16(len)) {
        return false;
    }
    return bytes(reinterpret_cast<const uint8_t *>(value), len);
}

PayloadReader::PayloadReader(const uint8_t *buffer, uint16_t length) :
    _buffer(buffer),
    _length(length)
{
}

bool PayloadReader::reserve(uint16_t count) const
{
    return !(_buffer == nullptr && count != 0) &&
           count <= _length && _offset <= _length - count;
}

bool PayloadReader::u8(uint8_t &value)
{
    if (!reserve(1)) {
        return false;
    }
    value = _buffer[_offset++];
    return true;
}

bool PayloadReader::u16(uint16_t &value)
{
    if (!reserve(2)) {
        return false;
    }
    value = read_u16_le(&_buffer[_offset]);
    _offset += 2;
    return true;
}

bool PayloadReader::u32(uint32_t &value)
{
    if (!reserve(4)) {
        return false;
    }
    value = read_u32_le(&_buffer[_offset]);
    _offset += 4;
    return true;
}

bool PayloadReader::u64(uint64_t &value)
{
    if (!reserve(8)) {
        return false;
    }
    value = 0;
    for (uint8_t i = 0; i < 8; i++) {
        value |= uint64_t(_buffer[_offset + i]) << (8 * i);
    }
    _offset += 8;
    return true;
}

bool PayloadReader::bytes(const uint8_t *&data, uint16_t data_len)
{
    if (!reserve(data_len)) {
        return false;
    }
    data = data_len == 0 ? nullptr : &_buffer[_offset];
    _offset += data_len;
    return true;
}

uint16_t PayloadReader::remaining() const
{
    return _length - _offset;
}

bool build_frame(MessageType message_type,
                 uint8_t flags,
                 uint16_t sequence,
                 const uint8_t *payload,
                 uint16_t payload_len,
                 uint8_t *decoded_scratch,
                 uint16_t decoded_capacity,
                 uint8_t *encoded_stream,
                 uint16_t encoded_stream_capacity,
                 uint16_t &encoded_stream_len)
{
    if (payload_len > MAX_PAYLOAD_SIZE ||
        decoded_scratch == nullptr || encoded_stream == nullptr ||
        (payload == nullptr && payload_len != 0) ||
        decoded_capacity < HEADER_SIZE + payload_len + CRC_SIZE ||
        encoded_stream_capacity < 2) {
        encoded_stream_len = 0;
        return false;
    }

    decoded_scratch[0] = MAJOR_VERSION;
    decoded_scratch[1] = uint8_t(message_type);
    decoded_scratch[2] = flags;
    write_u16_le(&decoded_scratch[3], sequence);
    write_u16_le(&decoded_scratch[5], payload_len);
    if (payload_len != 0 && payload != nullptr) {
        memcpy(&decoded_scratch[HEADER_SIZE], payload, payload_len);
    }

    const uint16_t crc_offset = HEADER_SIZE + payload_len;
    const uint32_t crc = crc32_iso_hdlc(decoded_scratch, crc_offset);
    write_u32_le(&decoded_scratch[crc_offset], crc);

    uint16_t encoded_body_len = 0;
    if (!cobs_encode(decoded_scratch,
                     crc_offset + CRC_SIZE,
                     encoded_stream,
                     encoded_stream_capacity - 1,
                     encoded_body_len)) {
        encoded_stream_len = 0;
        return false;
    }

    encoded_stream[encoded_body_len] = DELIMITER;
    encoded_stream_len = encoded_body_len + 1;
    return true;
}

bool self_test()
{
    static const uint8_t crc_check[] = {
        '1', '2', '3', '4', '5', '6', '7', '8', '9'
    };
    if (crc32_iso_hdlc(crc_check, sizeof(crc_check)) != 0xCBF43926U) {
        return false;
    }

    static const uint8_t ping_payload[] = {
        0x78, 0x56, 0x34, 0x12,
        0x08, 0x07, 0x06, 0x05,
        0x04, 0x03, 0x02, 0x01
    };
    static const uint8_t expected_stream[] = {
        0x03, 0x01, 0x03, 0x04, 0x34, 0x12, 0x0C, 0x11,
        0x78, 0x56, 0x34, 0x12, 0x08, 0x07, 0x06, 0x05,
        0x04, 0x03, 0x02, 0x01, 0x56, 0xB8, 0xD9, 0x95,
        0x00
    };

    uint8_t decoded[MAX_DECODED_FRAME_SIZE];
    uint8_t encoded[MAX_ENCODED_STREAM_SIZE];
    uint16_t encoded_len = 0;
    if (!build_frame(MessageType::PING,
                     0,
                     0x1234,
                     ping_payload,
                     sizeof(ping_payload),
                     decoded,
                     sizeof(decoded),
                     encoded,
                     sizeof(encoded),
                     encoded_len)) {
        return false;
    }
    return encoded_len == sizeof(expected_stream) &&
           memcmp(encoded, expected_stream, sizeof(expected_stream)) == 0;
}

bool is_known_message_type(uint8_t message_type)
{
    switch (MessageType(message_type)) {
    case MessageType::HELLO:
    case MessageType::DESCRIBE:
    case MessageType::PING:
    case MessageType::ERROR:
    case MessageType::GET:
    case MessageType::SET:
    case MessageType::VALUE:
    case MessageType::GET_MANY:
    case MessageType::SET_MANY:
    case MessageType::VALUES:
    case MessageType::GET_ALL_PARAMS:
    case MessageType::SUBSCRIBE:
    case MessageType::UNSUBSCRIBE:
    case MessageType::TELEMETRY:
    case MessageType::ACTION:
    case MessageType::ACTION_ACK:
    case MessageType::EVENT:
    case MessageType::ROUTE_LIST:
    case MessageType::ROUTE_SET:
    case MessageType::ROUTE_DELETE:
    case MessageType::ROUTES:
    case MessageType::FILE_LIST:
    case MessageType::FILE_STAT:
    case MessageType::FILE_OPEN:
    case MessageType::FILE_READ:
    case MessageType::FILE_WRITE:
    case MessageType::FILE_CLOSE:
    case MessageType::FILE_DELETE:
    case MessageType::FILE_RENAME:
        return true;
    }
    return false;
}

} // namespace Protocol
} // namespace Vektor
