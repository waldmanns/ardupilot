#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Vektor {
namespace Protocol {

static constexpr uint8_t MAJOR_VERSION = 1;
static constexpr uint8_t HEADER_SIZE = 7;
static constexpr uint8_t CRC_SIZE = 4;
static constexpr uint8_t MIN_FRAME_SIZE = HEADER_SIZE + CRC_SIZE;
static constexpr uint8_t DELIMITER = 0x00;
static constexpr uint16_t MAX_PAYLOAD_SIZE = 256;
static constexpr uint16_t MAX_DECODED_FRAME_SIZE =
    HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;
static constexpr uint16_t MAX_ENCODED_BODY_SIZE =
    MAX_DECODED_FRAME_SIZE + (MAX_DECODED_FRAME_SIZE / 254) + 2;
static constexpr uint16_t MAX_ENCODED_STREAM_SIZE =
    MAX_ENCODED_BODY_SIZE + 1;

enum Flag : uint8_t {
    FLAG_RESPONSE = 0x01,
    FLAG_MORE = 0x02,
    FLAG_URGENT = 0x04,
    FLAG_VOLATILE = 0x08,
};

enum class MessageType : uint8_t {
    HELLO = 0x01,
    DESCRIBE = 0x02,
    PING = 0x03,
    ERROR = 0x04,

    GET = 0x10,
    SET = 0x11,
    VALUE = 0x12,
    GET_MANY = 0x13,
    SET_MANY = 0x14,
    VALUES = 0x15,
    GET_ALL_PARAMS = 0x16,

    SUBSCRIBE = 0x20,
    UNSUBSCRIBE = 0x21,
    TELEMETRY = 0x22,

    ACTION = 0x30,
    ACTION_ACK = 0x31,
    EVENT = 0x32,

    ROUTE_LIST = 0x40,
    ROUTE_SET = 0x41,
    ROUTE_DELETE = 0x42,
    ROUTES = 0x43,

    FILE_LIST = 0x50,
    FILE_STAT = 0x51,
    FILE_OPEN = 0x52,
    FILE_READ = 0x53,
    FILE_WRITE = 0x54,
    FILE_CLOSE = 0x55,
    FILE_DELETE = 0x56,
    FILE_RENAME = 0x57,
};

enum class ErrorCode : uint16_t {
    UNKNOWN_MESSAGE = 0x0001,
    BAD_VERSION = 0x0002,
    BAD_LENGTH = 0x0003,
    BAD_ID = 0x0004,
    BAD_TYPE = 0x0005,
    OUT_OF_RANGE = 0x0006,
    READ_ONLY = 0x0007,
    WRITE_ONLY = 0x0008,
    BUSY = 0x0009,
    NOT_AVAILABLE = 0x000A,
    REQUIRES_RECONFIGURE = 0x000B,
    REQUIRES_REBOOT = 0x000C,
    INVALID_STATE = 0x000D,
    INVALID_ROUTE = 0x000E,
    TYPE_MISMATCH = 0x000F,
    TOO_MANY_ITEMS = 0x0010,
    PAYLOAD_TOO_LARGE = 0x0011,
    SEQUENCE_CONFLICT = 0x0012,
    STORAGE_ERROR = 0x0013,
    FILE_NOT_FOUND = 0x0014,
    FILE_ACCESS = 0x0015,
    FILE_RANGE = 0x0016,
    FILE_STORAGE_FULL = 0x0017,
    FILE_BAD_HANDLE = 0x0018,
    FILE_VERIFY_FAILED = 0x0019,
    ACTION_REJECTED = 0x001A,
    SUBSCRIPTION_LIMIT = 0x001B,
    UNSUPPORTED_RATE = 0x001C,
    DESCRIPTOR_VERSION = 0x001D,
    NOT_READY = 0x001E,
    INTERNAL_ERROR = 0x001F,
};

enum class DescriptorDomain : uint8_t {
    BOARD = 0x01,
    COMPONENT = 0x02,
    FIELD = 0x03,
    ENDPOINT = 0x04,
    TIMER_GROUP = 0x05,
    STORAGE_AREA = 0x06,
    RUNTIME_LIMIT = 0x07,
    ENUM_TABLE = 0x08,
    EVENT_TYPE = 0x09,
    ACTION_SCHEMA = 0x0A,
};

enum class EndpointKind : uint8_t {
    PWM_BANK = 0x01,
    PWM_CHANNEL = 0x02,
    FLEX_PORT = 0x03,
    UART = 0x04,
    CAN = 0x05,
    ADC = 0x06,
    RPM_CAPTURE = 0x07,
    DIGITAL_IO = 0x08,
    USB = 0x09,
    SENSOR = 0x0A,
    STORAGE = 0x0B,
    GNSS = 0x0C,
};

enum class PrimitiveType : uint8_t {
    INVALID = 0x00,
    BOOL = 0x01,
    U8 = 0x02,
    I8 = 0x03,
    U16 = 0x04,
    I16 = 0x05,
    U32 = 0x06,
    I32 = 0x07,
    U64 = 0x08,
    I64 = 0x09,
    FLOAT32 = 0x0A,
    FLOAT64 = 0x0B,
    ENUM = 0x0C,
    STRING = 0x0D,
    BLOB = 0x0E,
    VECTOR2F = 0x0F,
    VECTOR3F = 0x10,
    QUATERNIONF = 0x11,
};

enum class FieldKind : uint8_t {
    INPUT = 0x01,
    OUTPUT = 0x02,
    PARAMETER = 0x03,
    OBSERVABLE = 0x04,
};

struct FrameView {
    uint8_t version;
    MessageType message_type;
    uint8_t flags;
    uint16_t sequence;
    const uint8_t *payload;
    uint16_t payload_len;
};

enum class ParseResult : uint8_t {
    NONE,
    FRAME,
    EMPTY,
    ENCODED_OVERFLOW,
    BAD_COBS,
    BAD_LENGTH,
    BAD_VERSION,
    BAD_CRC,
};

class Parser {
public:
    ParseResult consume(uint8_t byte, FrameView &frame);
    void reset();

private:
    ParseResult finish_frame(FrameView &frame);

    uint8_t _encoded[MAX_ENCODED_BODY_SIZE];
    uint8_t _decoded[MAX_DECODED_FRAME_SIZE];
    uint16_t _encoded_len = 0;
    bool _discard_until_delimiter = false;
};

class PayloadWriter {
public:
    PayloadWriter(uint8_t *buffer, uint16_t capacity);

    bool u8(uint8_t value);
    bool u16(uint16_t value);
    bool u32(uint32_t value);
    bool u64(uint64_t value);
    bool bytes(const uint8_t *data, uint16_t data_len);
    bool str8(const char *value);
    bool str16(const char *value);

    const uint8_t *data() const { return _buffer; }
    uint16_t length() const { return _length; }
    uint16_t remaining() const { return _capacity - _length; }
    bool ok() const { return _ok; }

private:
    bool reserve(uint16_t count);

    uint8_t *_buffer;
    uint16_t _capacity;
    uint16_t _length = 0;
    bool _ok = true;
};

class PayloadReader {
public:
    PayloadReader(const uint8_t *buffer, uint16_t length);

    bool u8(uint8_t &value);
    bool u16(uint16_t &value);
    bool u32(uint32_t &value);
    bool u64(uint64_t &value);
    bool bytes(const uint8_t *&data, uint16_t data_len);
    uint16_t remaining() const;

private:
    bool reserve(uint16_t count) const;

    const uint8_t *_buffer;
    uint16_t _length;
    uint16_t _offset = 0;
};

uint32_t crc32_iso_hdlc(const uint8_t *data, uint32_t length);
namespace detail {

constexpr uint32_t fnv1a32_update(const char *path, uint32_t hash)
{
    return *path == '\0' ? hash :
        fnv1a32_update(path + 1,
                       (hash ^ uint8_t(*path)) * 0x01000193U);
}

} // namespace detail

constexpr uint32_t fnv1a32_constexpr(const char *path)
{
    return path == nullptr ? 0U :
        detail::fnv1a32_update(path, 0x811C9DC5U);
}
uint32_t fnv1a32(const char *path);
uint64_t fnv1a64_update(uint64_t hash,
                        const uint8_t *data,
                        uint16_t length);
uint64_t fnv1a64(const uint8_t *data, uint16_t length);
bool stable_ids_unique_nonzero(const uint32_t *ids, uint16_t count);
uint16_t primitive_size(PrimitiveType type);

bool cobs_encode(const uint8_t *decoded,
                 uint16_t decoded_len,
                 uint8_t *encoded,
                 uint16_t encoded_capacity,
                 uint16_t &encoded_len);
bool cobs_decode(const uint8_t *encoded,
                 uint16_t encoded_len,
                 uint8_t *decoded,
                 uint16_t decoded_capacity,
                 uint16_t &decoded_len);
bool build_frame(MessageType message_type,
                 uint8_t flags,
                 uint16_t sequence,
                 const uint8_t *payload,
                 uint16_t payload_len,
                 uint8_t *decoded_scratch,
                 uint16_t decoded_capacity,
                 uint8_t *encoded_stream,
                 uint16_t encoded_stream_capacity,
                 uint16_t &encoded_stream_len);
bool self_test();
bool is_known_message_type(uint8_t message_type);

} // namespace Protocol
} // namespace Vektor
