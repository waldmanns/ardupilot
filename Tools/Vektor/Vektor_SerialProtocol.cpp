#include "Vektor_SerialProtocol.h"

#include "Config.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>

#include <string.h>

extern const AP_HAL::HAL &hal;

namespace {

namespace Protocol = Vektor::Protocol;

static constexpr uint16_t max_rx_bytes_per_update = 256;
static constexpr uint8_t flex_pwm_input = 1U << 0;
static constexpr uint8_t flex_pwm_output = 1U << 1;
static constexpr uint8_t flex_rpm_capture = 1U << 2;
static constexpr uint8_t flex_digital_input = 1U << 4;
static constexpr uint32_t field_readable = 1U << 0;
static constexpr uint32_t field_writable = 1U << 1;
static constexpr uint32_t field_persistent = 1U << 2;
static constexpr uint32_t field_has_min = 1U << 4;
static constexpr uint32_t field_has_max = 1U << 5;
static constexpr uint32_t field_has_default = 1U << 6;
static constexpr uint32_t field_apply_live = 1U << 7;
static constexpr uint32_t field_apply_reboot = 1U << 9;
static constexpr uint32_t param_live_flags =
    field_readable | field_writable | field_persistent |
    field_has_min | field_has_max | field_has_default |
    field_apply_live;
static constexpr const char *protocol_component_path = "component/system/0";
static constexpr const char *runtime_component_path = "component/system/1";

struct ComponentDescriptor {
    const char *path;
    const char *type_path;
    const char *name;
    const char *display_name;
    uint16_t instance;
    uint32_t flags;
};

const ComponentDescriptor component_descriptors[] = {
    {
        protocol_component_path,
        "component_type/system/protocol",
        "protocol",
        "Protocol",
        0,
        0,
    },
    {
        runtime_component_path,
        "component_type/system/runtime",
        "runtime",
        "Runtime",
        1,
        0,
    },
};

static constexpr uint16_t component_descriptor_count =
    sizeof(component_descriptors) / sizeof(component_descriptors[0]);

enum class FieldSlot : uint8_t {
    RX_FRAMES,
    RX_DROPS,
    TX_DROPS,
    UPTIME_MS,
    LOOP_COUNT,
    LOOP_DT_US,
    LOOP_WORK_US,
    LOOP_MAX_WORK_US,
    SERVICE_RATE_HZ,
    SYS_OPTIONS,
    SYS_DESC_PAGE,
    SYS_PROTOCOL_BAUD,
};

struct FieldDescriptor {
    const char *path;
    const char *owner_component_path;
    const char *name;
    const char *display_name;
    const char *units;
    Protocol::FieldKind kind;
    Protocol::PrimitiveType type;
    uint32_t flags;
    int32_t min_value;
    int32_t max_value;
    int32_t default_value;
    FieldSlot slot;
};

const FieldDescriptor field_descriptors[] = {
    {
        "component/system/0/observable/rx_frames",
        protocol_component_path,
        "rx_frames",
        "RX Frames",
        "",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        field_readable,
        0,
        0,
        0,
        FieldSlot::RX_FRAMES,
    },
    {
        "component/system/0/observable/rx_drops",
        protocol_component_path,
        "rx_drops",
        "RX Drops",
        "",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        field_readable,
        0,
        0,
        0,
        FieldSlot::RX_DROPS,
    },
    {
        "component/system/0/observable/tx_drops",
        protocol_component_path,
        "tx_drops",
        "TX Drops",
        "",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        field_readable,
        0,
        0,
        0,
        FieldSlot::TX_DROPS,
    },
    {
        "component/system/1/observable/uptime_ms",
        runtime_component_path,
        "uptime_ms",
        "Uptime",
        "ms",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        field_readable,
        0,
        0,
        0,
        FieldSlot::UPTIME_MS,
    },
    {
        "component/system/1/observable/loop_count",
        runtime_component_path,
        "loop_count",
        "Loop Count",
        "",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        field_readable,
        0,
        0,
        0,
        FieldSlot::LOOP_COUNT,
    },
    {
        "component/system/1/observable/loop_dt_us",
        runtime_component_path,
        "loop_dt_us",
        "Loop Delta",
        "us",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        field_readable,
        0,
        0,
        0,
        FieldSlot::LOOP_DT_US,
    },
    {
        "component/system/1/observable/loop_work_us",
        runtime_component_path,
        "loop_work_us",
        "Loop Work",
        "us",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        field_readable,
        0,
        0,
        0,
        FieldSlot::LOOP_WORK_US,
    },
    {
        "component/system/1/observable/loop_max_work_us",
        runtime_component_path,
        "loop_max_work_us",
        "Max Loop Work",
        "us",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        field_readable,
        0,
        0,
        0,
        FieldSlot::LOOP_MAX_WORK_US,
    },
    {
        "component/system/1/observable/service_rate_hz",
        runtime_component_path,
        "service_rate_hz",
        "Service Rate",
        "Hz",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U16,
        field_readable,
        0,
        0,
        0,
        FieldSlot::SERVICE_RATE_HZ,
    },
    {
        "component/system/0/parameter/sys_options",
        protocol_component_path,
        "sys_options",
        "System Options",
        "",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I32,
        param_live_flags,
        0,
        INT32_MAX,
        0,
        FieldSlot::SYS_OPTIONS,
    },
    {
        "component/system/0/parameter/sys_desc_page",
        protocol_component_path,
        "sys_desc_page",
        "Descriptor Page Size",
        "records",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        param_live_flags,
        Vektor::min_describe_page_records,
        Vektor::max_describe_page_records,
        Vektor::default_describe_page_records,
        FieldSlot::SYS_DESC_PAGE,
    },
    {
        "component/system/0/parameter/sys_protocol_baud",
        protocol_component_path,
        "sys_protocol_baud",
        "Protocol Baud",
        "baud",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I32,
        field_readable | field_writable | field_persistent |
        field_has_min | field_has_max | field_has_default |
        field_apply_reboot,
        Vektor::protocol_baud_min,
        Vektor::protocol_baud_max,
        Vektor::protocol_baud,
        FieldSlot::SYS_PROTOCOL_BAUD,
    },
};

static constexpr uint16_t field_descriptor_count =
    sizeof(field_descriptors) / sizeof(field_descriptors[0]);

uint32_t id_for_path(const char *path)
{
    return Vektor::Protocol::fnv1a32(path);
}

const FieldDescriptor *field_descriptor_by_index(uint16_t index)
{
    if (index >= field_descriptor_count) {
        return nullptr;
    }
    return &field_descriptors[index];
}

const ComponentDescriptor *component_descriptor_by_index(uint16_t index)
{
    if (index >= component_descriptor_count) {
        return nullptr;
    }
    return &component_descriptors[index];
}

const FieldDescriptor *field_descriptor_by_id(uint32_t field_id)
{
    for (uint16_t i = 0; i < field_descriptor_count; i++) {
        if (id_for_path(field_descriptors[i].path) == field_id) {
            return &field_descriptors[i];
        }
    }
    return nullptr;
}

bool field_is_parameter(const FieldDescriptor &field)
{
    return field.kind == Protocol::FieldKind::PARAMETER;
}

int32_t signed_raw_value(Protocol::PrimitiveType type, uint32_t raw)
{
    if (type == Protocol::PrimitiveType::I16) {
        return int16_t(uint16_t(raw));
    }
    return int32_t(raw);
}

bool write_typed_payload(Protocol::PayloadWriter &writer,
                         Protocol::PrimitiveType type,
                         uint32_t raw)
{
    switch (type) {
    case Protocol::PrimitiveType::U16:
    case Protocol::PrimitiveType::I16:
        return writer.u16(uint16_t(raw));
    case Protocol::PrimitiveType::U32:
    case Protocol::PrimitiveType::I32:
        return writer.u32(raw);
    default:
        return false;
    }
}

void pwm_channel_path(uint8_t channel, char *path, uint8_t path_len)
{
    hal.util->snprintf(path,
                       path_len,
                       "hw/pwm_bank/0/channel/%u",
                       unsigned(channel));
}

uint32_t pwm_channel_id(uint8_t channel)
{
    char path[32];
    pwm_channel_path(channel, path, sizeof(path));
    return id_for_path(path);
}

void flex_path(uint8_t port, char *path, uint8_t path_len)
{
    hal.util->snprintf(path, path_len, "hw/flex/%u", unsigned(port));
}

uint32_t flex_id(uint8_t port)
{
    char path[20];
    flex_path(port, path, sizeof(path));
    return id_for_path(path);
}

void append_firmware_version(Vektor::Protocol::PayloadWriter &writer)
{
    char version[16];
    hal.util->snprintf(version,
                       sizeof(version),
                       "%u.%u.%u",
                       unsigned(Vektor::firmware_major),
                       unsigned(Vektor::firmware_minor),
                       unsigned(Vektor::firmware_patch));
    writer.str8(version);
}

} // namespace

namespace Vektor {

void SerialProtocol::init(AP_HAL::UARTDriver *uart,
                          const BoardCapability &capability,
                          Parameters &parameters,
                          const RuntimeState &runtime)
{
    _uart = uart;
    _capability = &capability;
    _parameters = &parameters;
    _runtime = &runtime;
    _server_nonce = uint32_t(AP_HAL::micros64()) ^ uint32_t(device_id());
    _ready = (_uart != nullptr) && Protocol::self_test();
    _hello_seen = false;
    _capture_response = false;
    _request_cache.reset();

    if (!_ready) {
        return;
    }

    int32_t baud = _parameters->sys_protocol_baud.get();
    if (baud < protocol_baud_min || baud > protocol_baud_max) {
        baud = protocol_baud;
    }
    _uart->begin(uint32_t(baud), protocol_rx_space, protocol_tx_space);
    _uart->write(uint8_t(Protocol::DELIMITER));
}

void SerialProtocol::update()
{
    if (!_ready || _uart == nullptr) {
        return;
    }

    Protocol::FrameView frame {};
    uint16_t processed = 0;
    while (_uart->available() > 0 && processed < max_rx_bytes_per_update) {
        const int16_t value = _uart->read();
        if (value < 0) {
            break;
        }
        processed++;

        const Protocol::ParseResult result =
            _parser.consume(uint8_t(value), frame);
        switch (result) {
        case Protocol::ParseResult::FRAME:
            _rx_frames++;
            handle_frame(frame);
            break;
        case Protocol::ParseResult::NONE:
        case Protocol::ParseResult::EMPTY:
            break;
        case Protocol::ParseResult::ENCODED_OVERFLOW:
        case Protocol::ParseResult::BAD_COBS:
        case Protocol::ParseResult::BAD_LENGTH:
        case Protocol::ParseResult::BAD_VERSION:
        case Protocol::ParseResult::BAD_CRC:
            _rx_drops++;
            break;
        }
    }
}

void SerialProtocol::handle_frame(const Protocol::FrameView &frame)
{
    if ((frame.flags & Protocol::FLAG_RESPONSE) != 0) {
        return;
    }

    if (replay_cached_response_or_reject(frame)) {
        return;
    }

    begin_request_cache_capture(frame);
    switch (frame.message_type) {
    case Protocol::MessageType::HELLO:
        handle_hello(frame);
        break;
    case Protocol::MessageType::PING:
        handle_ping(frame);
        break;
    case Protocol::MessageType::DESCRIBE:
        handle_describe(frame);
        break;
    case Protocol::MessageType::GET:
        handle_get(frame);
        break;
    case Protocol::MessageType::SET:
        handle_set(frame);
        break;
    case Protocol::MessageType::GET_MANY:
        handle_get_many(frame);
        break;
    case Protocol::MessageType::SET_MANY:
        handle_set_many(frame);
        break;
    case Protocol::MessageType::GET_ALL_PARAMS:
        handle_get_all_params(frame);
        break;
    default:
        if (Protocol::is_known_message_type(uint8_t(frame.message_type))) {
            send_error(frame,
                       Protocol::ErrorCode::NOT_AVAILABLE,
                       0,
                       "message not implemented");
        } else {
            send_error(frame,
                       Protocol::ErrorCode::UNKNOWN_MESSAGE,
                       0,
                       "unknown message");
        }
        break;
    }
    end_request_cache_capture();
}

void SerialProtocol::handle_hello(const Protocol::FrameView &frame)
{
    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint8_t client_min_major = 0;
    uint8_t client_max_major = 0;
    uint16_t client_proto_minor = 0;
    uint32_t client_feature_flags = 0;
    uint32_t client_nonce = 0;
    if (!reader.u8(client_min_major) ||
        !reader.u8(client_max_major) ||
        !reader.u16(client_proto_minor) ||
        !reader.u32(client_feature_flags) ||
        !reader.u32(client_nonce) ||
        reader.remaining() != 0) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad HELLO");
        return;
    }

    (void)client_proto_minor;
    (void)client_feature_flags;
    _server_nonce ^= client_nonce;

    if (client_min_major > Protocol::MAJOR_VERSION ||
        client_max_major < Protocol::MAJOR_VERSION) {
        send_error(frame,
                   Protocol::ErrorCode::BAD_VERSION,
                   0,
                   "protocol major mismatch");
        return;
    }

    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u8(Protocol::MAJOR_VERSION);
    writer.u16(0); // server protocol minor
    append_firmware_version(writer);
    writer.str8(_capability->product_name);
    writer.str8(_capability->hardware_revision);
    writer.u32(_capability->board_id);
    writer.u64(0); // schema_hash: not stable yet
    writer.u64(0); // capability_hash: not stable yet
    writer.u64(device_id());
    writer.u64(coarse_capability_flags());
    writer.u16(Protocol::MAX_PAYLOAD_SIZE);
    writer.u16(0); // max_file_chunk: file service not implemented yet
    writer.u16(0); // max_subscriptions
    writer.u16(0); // max_realtime_hz
    writer.u16(0); // control_update_hz
    writer.u16(0); // attitude_update_hz
    writer.u32(_server_nonce);

    if (!writer.ok()) {
        send_error(frame,
                   Protocol::ErrorCode::INTERNAL_ERROR,
                   0,
                   "HELLO payload overflow");
        return;
    }

    _hello_seen = send_payload(Protocol::MessageType::HELLO,
                               Protocol::FLAG_RESPONSE,
                               frame.sequence,
                               writer.data(),
                               writer.length());
}

void SerialProtocol::handle_ping(const Protocol::FrameView &frame)
{
    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint32_t cookie = 0;
    uint64_t host_time_us = 0;
    if (!reader.u32(cookie) ||
        !reader.u64(host_time_us) ||
        reader.remaining() != 0) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad PING");
        return;
    }

    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u32(cookie);
    writer.u64(host_time_us);
    writer.u64(AP_HAL::micros64());

    send_payload(Protocol::MessageType::PING,
                 Protocol::FLAG_RESPONSE,
                 frame.sequence,
                 writer.data(),
                 writer.length());
}

void SerialProtocol::handle_describe(const Protocol::FrameView &frame)
{
    if (!_hello_seen) {
        send_error(frame,
                   Protocol::ErrorCode::INVALID_STATE,
                   0,
                   "HELLO required");
        return;
    }

    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint8_t domain = 0;
    uint32_t cursor = 0;
    uint16_t max_records = 0;
    if (!reader.u8(domain) ||
        !reader.u32(cursor) ||
        !reader.u16(max_records) ||
        reader.remaining() != 0) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad DESCRIBE");
        return;
    }

    if (domain == uint8_t(Protocol::DescriptorDomain::BOARD)) {
        send_board_description(frame.sequence, domain, cursor, max_records);
        return;
    }

    switch (Protocol::DescriptorDomain(domain)) {
    case Protocol::DescriptorDomain::COMPONENT:
        send_component_descriptions(frame.sequence, domain, cursor, max_records);
        return;
    case Protocol::DescriptorDomain::FIELD:
        send_field_descriptions(frame.sequence, domain, cursor, max_records);
        return;
    case Protocol::DescriptorDomain::ENDPOINT:
        send_endpoint_descriptions(frame.sequence, domain, cursor, max_records);
        return;
    case Protocol::DescriptorDomain::TIMER_GROUP:
        send_timer_group_descriptions(frame.sequence, domain, cursor, max_records);
        return;
    case Protocol::DescriptorDomain::STORAGE_AREA:
    case Protocol::DescriptorDomain::RUNTIME_LIMIT:
    case Protocol::DescriptorDomain::ENUM_TABLE:
    case Protocol::DescriptorDomain::EVENT_TYPE:
    case Protocol::DescriptorDomain::ACTION_SCHEMA:
        send_empty_description(frame.sequence, domain);
        return;
    case Protocol::DescriptorDomain::BOARD:
        break;
    }

    send_error(frame, Protocol::ErrorCode::BAD_ID, 0, "unknown domain");
}

void SerialProtocol::handle_get(const Protocol::FrameView &frame)
{
    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint32_t field_id = 0;
    if (!reader.u32(field_id) || reader.remaining() != 0) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad GET");
        return;
    }

    const FieldDescriptor *field = field_descriptor_by_id(field_id);
    if (field == nullptr) {
        send_error(frame, Protocol::ErrorCode::BAD_ID, field_id, "unknown field");
        return;
    }
    if ((field->flags & field_readable) == 0) {
        send_error(frame, Protocol::ErrorCode::WRITE_ONLY, field_id, "write-only field");
        return;
    }

    if (!send_value(frame.sequence, field_id)) {
        send_error(frame,
                   Protocol::ErrorCode::INTERNAL_ERROR,
                   field_id,
                   "failed GET");
    }
}

void SerialProtocol::handle_set(const Protocol::FrameView &frame)
{
    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint32_t field_id = 0;
    uint8_t type_id = 0;
    uint32_t raw = 0;
    Protocol::ErrorCode code = Protocol::ErrorCode::INTERNAL_ERROR;
    const char *detail = "bad SET";

    if (!reader.u32(field_id) || !reader.u8(type_id)) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad SET");
        return;
    }

    const FieldDescriptor *field = field_descriptor_by_id(field_id);
    if (field == nullptr) {
        send_error(frame, Protocol::ErrorCode::BAD_ID, field_id, "unknown field");
        return;
    }
    if (type_id != uint8_t(field->type)) {
        send_error(frame,
                   Protocol::ErrorCode::TYPE_MISMATCH,
                   field_id,
                   "type mismatch");
        return;
    }

    if (!read_typed_value(reader, field->type, raw) ||
        reader.remaining() != 0) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, field_id, "bad SET");
        return;
    }

    if (!validate_parameter_value(field_id, type_id, raw, code, detail)) {
        send_error(frame, code, field_id, detail);
        return;
    }

    if (!apply_parameter_value(field_id, type_id, raw)) {
        send_error(frame,
                   Protocol::ErrorCode::STORAGE_ERROR,
                   field_id,
                   "failed to save parameter");
        return;
    }

    send_value(frame.sequence, field_id);
}

void SerialProtocol::handle_get_many(const Protocol::FrameView &frame)
{
    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint16_t count = 0;
    if (!reader.u16(count)) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad GET_MANY");
        return;
    }
    if (count > field_count()) {
        send_error(frame, Protocol::ErrorCode::TOO_MANY_ITEMS, 0, "too many fields");
        return;
    }

    uint32_t field_ids[8];
    if (count > sizeof(field_ids) / sizeof(field_ids[0])) {
        send_error(frame, Protocol::ErrorCode::TOO_MANY_ITEMS, 0, "too many fields");
        return;
    }

    for (uint16_t i = 0; i < count; i++) {
        if (!reader.u32(field_ids[i])) {
            send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad GET_MANY");
            return;
        }
        const FieldDescriptor *field = field_descriptor_by_id(field_ids[i]);
        if (field == nullptr) {
            send_error(frame,
                       Protocol::ErrorCode::BAD_ID,
                       field_ids[i],
                       "unknown field");
            return;
        }
        if ((field->flags & field_readable) == 0) {
            send_error(frame,
                       Protocol::ErrorCode::WRITE_ONLY,
                       field_ids[i],
                       "write-only field");
            return;
        }
    }

    if (reader.remaining() != 0) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad GET_MANY");
        return;
    }

    send_values(frame.sequence, field_ids, count);
}

void SerialProtocol::handle_set_many(const Protocol::FrameView &frame)
{
    struct PendingSet {
        uint32_t field_id;
        uint8_t type_id;
        uint32_t raw;
    };

    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint16_t count = 0;
    if (!reader.u16(count)) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad SET_MANY");
        return;
    }
    if (count == 0 || count > parameter_field_count()) {
        send_error(frame,
                   Protocol::ErrorCode::TOO_MANY_ITEMS,
                   0,
                   "invalid parameter count");
        return;
    }

    PendingSet pending[8];
    if (count > sizeof(pending) / sizeof(pending[0])) {
        send_error(frame,
                   Protocol::ErrorCode::TOO_MANY_ITEMS,
                   0,
                   "too many parameters");
        return;
    }

    for (uint16_t i = 0; i < count; i++) {
        if (!reader.u32(pending[i].field_id) ||
            !reader.u8(pending[i].type_id)) {
            send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad SET_MANY");
            return;
        }

        for (uint16_t j = 0; j < i; j++) {
            if (pending[j].field_id == pending[i].field_id) {
                send_error(frame,
                           Protocol::ErrorCode::SEQUENCE_CONFLICT,
                           pending[i].field_id,
                           "duplicate field");
                return;
            }
        }

        const FieldDescriptor *field =
            field_descriptor_by_id(pending[i].field_id);
        if (field == nullptr) {
            send_error(frame,
                       Protocol::ErrorCode::BAD_ID,
                       pending[i].field_id,
                       "unknown field");
            return;
        }
        if (pending[i].type_id != uint8_t(field->type)) {
            send_error(frame,
                       Protocol::ErrorCode::TYPE_MISMATCH,
                       pending[i].field_id,
                       "type mismatch");
            return;
        }

        if (!read_typed_value(reader, field->type, pending[i].raw)) {
            send_error(frame,
                       Protocol::ErrorCode::BAD_LENGTH,
                       pending[i].field_id,
                       "bad SET_MANY");
            return;
        }
    }

    if (reader.remaining() != 0) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad SET_MANY");
        return;
    }

    for (uint16_t i = 0; i < count; i++) {
        Protocol::ErrorCode code = Protocol::ErrorCode::INTERNAL_ERROR;
        const char *detail = "bad SET_MANY";
        if (!validate_parameter_value(pending[i].field_id,
                                      pending[i].type_id,
                                      pending[i].raw,
                                      code,
                                      detail)) {
            send_error(frame, code, pending[i].field_id, detail);
            return;
        }
    }

    for (uint16_t i = 0; i < count; i++) {
        if (!apply_parameter_value(pending[i].field_id,
                                   pending[i].type_id,
                                   pending[i].raw)) {
            send_error(frame,
                       Protocol::ErrorCode::STORAGE_ERROR,
                       pending[i].field_id,
                       "failed to save parameter");
            return;
        }
    }

    uint32_t field_ids[8];
    for (uint16_t i = 0; i < count; i++) {
        field_ids[i] = pending[i].field_id;
    }
    send_values(frame.sequence, field_ids, count);
}

void SerialProtocol::handle_get_all_params(const Protocol::FrameView &frame)
{
    if (frame.payload_len != 0) {
        send_error(frame,
                   Protocol::ErrorCode::BAD_LENGTH,
                   0,
                   "bad GET_ALL_PARAMS");
        return;
    }
    send_all_parameters(frame.sequence);
}

bool SerialProtocol::replay_cached_response_or_reject(
    const Protocol::FrameView &frame)
{
    const uint32_t payload_crc =
        Protocol::crc32_iso_hdlc(frame.payload, frame.payload_len);

    RequestReplayCache::CachedResponse response {};
    const RequestReplayCache::LookupResult result =
        _request_cache.lookup(frame.message_type,
                              frame.sequence,
                              payload_crc,
                              response);

    if (result == RequestReplayCache::LookupResult::CONFLICT) {
        send_error(frame,
                   Protocol::ErrorCode::SEQUENCE_CONFLICT,
                   0,
                   "sequence conflict");
        return true;
    }

    if (result == RequestReplayCache::LookupResult::REPLAY) {
        if (_uart == nullptr) {
            return true;
        }

        const size_t written =
            _uart->write(response.encoded, response.encoded_len);
        if (written != response.encoded_len) {
            _tx_drops++;
        }
        return true;
    }

    return false;
}

void SerialProtocol::begin_request_cache_capture(
    const Protocol::FrameView &frame)
{
    _capture_response = true;
    _capture_request_type = frame.message_type;
    _capture_sequence = frame.sequence;
    _capture_payload_crc =
        Protocol::crc32_iso_hdlc(frame.payload, frame.payload_len);
}

void SerialProtocol::end_request_cache_capture()
{
    _capture_response = false;
}

void SerialProtocol::remember_cached_response(const uint8_t *encoded,
                                              uint16_t encoded_len)
{
    if (!_capture_response ||
        encoded == nullptr ||
        encoded_len == 0 ||
        encoded_len > Protocol::MAX_ENCODED_STREAM_SIZE) {
        return;
    }

    _request_cache.remember(_capture_request_type,
                            _capture_sequence,
                            _capture_payload_crc,
                            encoded,
                            encoded_len);
}

bool SerialProtocol::send_board_description(uint16_t sequence,
                                            uint8_t domain,
                                            uint32_t cursor,
                                            uint16_t max_records)
{
    (void)max_records;
    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u8(domain);

    if (cursor != 0) {
        writer.u32(0);
        writer.u16(0);
        return send_payload(Protocol::MessageType::DESCRIBE,
                            Protocol::FLAG_RESPONSE,
                            sequence,
                            writer.data(),
                            writer.length());
    }

    uint8_t record[96];
    Protocol::PayloadWriter record_writer(record, sizeof(record));
    record_writer.u8(1); // record_version
    record_writer.u32(_capability->board_id);
    record_writer.str8(_capability->product_name);
    record_writer.str8(_capability->hardware_revision);
    record_writer.str8(_capability->product_name);
    if (!record_writer.ok()) {
        _tx_drops++;
        return false;
    }

    writer.u32(0); // next_cursor
    writer.u16(1); // record_count
    writer.u16(record_writer.length());
    writer.bytes(record_writer.data(), record_writer.length());
    if (!writer.ok()) {
        _tx_drops++;
        return false;
    }

    return send_payload(Protocol::MessageType::DESCRIBE,
                        Protocol::FLAG_RESPONSE,
                        sequence,
                        writer.data(),
                        writer.length());
}

bool SerialProtocol::send_endpoint_descriptions(uint16_t sequence,
                                                uint8_t domain,
                                                uint32_t cursor,
                                                uint16_t max_records)
{
    return send_description_page(sequence,
                                 domain,
                                 cursor,
                                 max_records,
                                 endpoint_count());
}

bool SerialProtocol::send_component_descriptions(uint16_t sequence,
                                                 uint8_t domain,
                                                 uint32_t cursor,
                                                 uint16_t max_records)
{
    return send_description_page(sequence,
                                 domain,
                                 cursor,
                                 max_records,
                                 component_count());
}

bool SerialProtocol::send_field_descriptions(uint16_t sequence,
                                             uint8_t domain,
                                             uint32_t cursor,
                                             uint16_t max_records)
{
    return send_description_page(sequence,
                                 domain,
                                 cursor,
                                 max_records,
                                 field_count());
}

bool SerialProtocol::send_timer_group_descriptions(uint16_t sequence,
                                                   uint8_t domain,
                                                   uint32_t cursor,
                                                   uint16_t max_records)
{
    return send_description_page(sequence,
                                 domain,
                                 cursor,
                                 max_records,
                                 timer_group_count());
}

bool SerialProtocol::send_empty_description(uint16_t sequence, uint8_t domain)
{
    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u8(domain);
    writer.u32(0);
    writer.u16(0);
    return send_payload(Protocol::MessageType::DESCRIBE,
                        Protocol::FLAG_RESPONSE,
                        sequence,
                        writer.data(),
                        writer.length());
}

bool SerialProtocol::send_description_page(uint16_t sequence,
                                           uint8_t domain,
                                           uint32_t cursor,
                                           uint16_t max_records,
                                           uint16_t total_records)
{
    if (cursor >= total_records) {
        return send_empty_description(sequence, domain);
    }

    const uint16_t configured_limit = descriptor_page_limit();
    const uint16_t page_limit =
        (max_records == 0 || max_records > configured_limit) ?
        configured_limit :
        max_records;

    uint8_t records[Protocol::MAX_PAYLOAD_SIZE - 7];
    Protocol::PayloadWriter records_writer(records, sizeof(records));
    uint16_t record_count = 0;
    uint32_t index = cursor;

    while (index < total_records && record_count < page_limit) {
        uint8_t record[128];
        uint16_t record_len = 0;
        if (!build_descriptor_record(domain,
                                     uint16_t(index),
                                     record,
                                     sizeof(record),
                                     record_len)) {
            _tx_drops++;
            return false;
        }

        const uint16_t envelope_len = 2 + record_len;
        if (records_writer.remaining() < envelope_len) {
            break;
        }

        records_writer.u16(record_len);
        records_writer.bytes(record, record_len);
        record_count++;
        index++;
    }

    const uint32_t next_cursor = (index < total_records) ? index : 0;

    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u8(domain);
    writer.u32(next_cursor);
    writer.u16(record_count);
    writer.bytes(records_writer.data(), records_writer.length());
    if (!writer.ok() || record_count == 0) {
        _tx_drops++;
        return false;
    }

    uint8_t flags = Protocol::FLAG_RESPONSE;
    if (next_cursor != 0) {
        flags |= Protocol::FLAG_MORE;
    }
    return send_payload(Protocol::MessageType::DESCRIBE,
                        flags,
                        sequence,
                        writer.data(),
                        writer.length());
}

bool SerialProtocol::build_descriptor_record(uint8_t domain,
                                             uint16_t index,
                                             uint8_t *record,
                                             uint16_t record_capacity,
                                             uint16_t &record_len)
{
    if (domain == uint8_t(Protocol::DescriptorDomain::ENDPOINT)) {
        return build_endpoint_record(index, record, record_capacity, record_len);
    }
    if (domain == uint8_t(Protocol::DescriptorDomain::COMPONENT)) {
        return build_component_record(index, record, record_capacity, record_len);
    }
    if (domain == uint8_t(Protocol::DescriptorDomain::FIELD)) {
        return build_field_record(index, record, record_capacity, record_len);
    }
    if (domain == uint8_t(Protocol::DescriptorDomain::TIMER_GROUP)) {
        return build_timer_group_record(index, record, record_capacity, record_len);
    }

    record_len = 0;
    return false;
}

bool SerialProtocol::build_component_record(uint16_t index,
                                            uint8_t *record,
                                            uint16_t record_capacity,
                                            uint16_t &record_len)
{
    const ComponentDescriptor *component = component_descriptor_by_index(index);
    if (component == nullptr) {
        record_len = 0;
        return false;
    }

    Protocol::PayloadWriter writer(record, record_capacity);
    writer.u8(1); // record_version
    writer.u32(id_for_path(component->path));
    writer.u32(id_for_path(component->type_path));
    writer.u16(component->instance);
    writer.u32(component->flags);
    writer.str8(component->name);
    writer.str8(component->display_name);
    record_len = writer.length();
    return writer.ok();
}

bool SerialProtocol::build_field_record(uint16_t index,
                                        uint8_t *record,
                                        uint16_t record_capacity,
                                        uint16_t &record_len)
{
    if (index >= field_count()) {
        record_len = 0;
        return false;
    }

    const FieldDescriptor *field = field_descriptor_by_index(index);
    if (field == nullptr) {
        record_len = 0;
        return false;
    }

    Protocol::PayloadWriter writer(record, record_capacity);
    writer.u8(1); // record_version
    writer.u32(protocol_field_id(index));
    writer.u32(id_for_path(field->owner_component_path));
    writer.u8(uint8_t(field->kind));
    writer.u8(uint8_t(field->type));
    writer.u32(field->flags);
    writer.str8(field->name);
    writer.str8(field->display_name);
    writer.str8(field->units);
    if ((field->flags & field_has_min) != 0 &&
        !write_typed_payload(writer, field->type, uint32_t(field->min_value))) {
        record_len = 0;
        return false;
    }
    if ((field->flags & field_has_max) != 0 &&
        !write_typed_payload(writer, field->type, uint32_t(field->max_value))) {
        record_len = 0;
        return false;
    }
    if ((field->flags & field_has_default) != 0 &&
        !write_typed_payload(writer, field->type, uint32_t(field->default_value))) {
        record_len = 0;
        return false;
    }
    record_len = writer.length();
    return writer.ok();
}

bool SerialProtocol::build_endpoint_record(uint16_t index,
                                           uint8_t *record,
                                           uint16_t record_capacity,
                                           uint16_t &record_len)
{
    auto write_endpoint = [&](Protocol::EndpointKind kind,
                              uint32_t endpoint_id,
                              uint32_t parent_id,
                              uint64_t flags,
                              const char *name,
                              const char *display_name) {
        Protocol::PayloadWriter writer(record, record_capacity);
        writer.u8(1); // record_version
        writer.u32(endpoint_id);
        writer.u8(uint8_t(kind));
        writer.u32(parent_id);
        writer.u64(flags);
        writer.str8(name);
        writer.str8(display_name);
        record_len = writer.length();
        return writer.ok();
    };

    uint16_t remaining = index;

    if (remaining == 0) {
        return write_endpoint(Protocol::EndpointKind::USB,
                              id_for_path("hw/usb/0"),
                              0,
                              0,
                              "usb0",
                              "USB");
    }
    remaining--;

    if (_capability->pwm_outputs != 0) {
        const uint32_t pwm_bank_id = id_for_path("hw/pwm_bank/0");
        if (remaining == 0) {
            return write_endpoint(Protocol::EndpointKind::PWM_BANK,
                                  pwm_bank_id,
                                  0,
                                  0,
                                  "pwm_bank0",
                                  "PWM Bank");
        }
        remaining--;

        if (remaining < _capability->pwm_outputs) {
            const uint8_t channel = uint8_t(remaining + 1);
            char path[32];
            char name[16];
            char display[16];
            pwm_channel_path(channel, path, sizeof(path));
            hal.util->snprintf(name, sizeof(name), "pwm%u", unsigned(channel));
            hal.util->snprintf(display, sizeof(display), "PWM %u", unsigned(channel));
            return write_endpoint(Protocol::EndpointKind::PWM_CHANNEL,
                                  id_for_path(path),
                                  pwm_bank_id,
                                  0,
                                  name,
                                  display);
        }
        remaining -= _capability->pwm_outputs;
    }

    if (remaining < _capability->flex_timer_channels) {
        const uint8_t port = uint8_t(remaining);
        char path[20];
        char name[16];
        char display[16];
        flex_path(port, path, sizeof(path));
        hal.util->snprintf(name, sizeof(name), "flex%u", unsigned(port + 1));
        hal.util->snprintf(display, sizeof(display), "Flex %u", unsigned(port + 1));
        return write_endpoint(Protocol::EndpointKind::FLEX_PORT,
                              id_for_path(path),
                              0,
                              flex_pwm_input |
                              flex_pwm_output |
                              flex_rpm_capture |
                              flex_digital_input,
                              name,
                              display);
    }
    remaining -= _capability->flex_timer_channels;

    if (remaining < _capability->dedicated_receiver_rows) {
        return write_endpoint(Protocol::EndpointKind::UART,
                              id_for_path("hw/uart/receiver/0"),
                              0,
                              0,
                              "receiver0",
                              "Dedicated Receiver RX");
    }
    remaining -= _capability->dedicated_receiver_rows;

    if (remaining < _capability->uart_endpoints) {
        const uint8_t uart = uint8_t(remaining);
        char path[20];
        char name[16];
        char display[16];
        hal.util->snprintf(path, sizeof(path), "hw/uart/%u", unsigned(uart));
        hal.util->snprintf(name, sizeof(name), "uart%u", unsigned(uart + 1));
        hal.util->snprintf(display, sizeof(display), "UART %u", unsigned(uart + 1));
        return write_endpoint(Protocol::EndpointKind::UART,
                              id_for_path(path),
                              0,
                              0,
                              name,
                              display);
    }
    remaining -= _capability->uart_endpoints;

    if (remaining < _capability->can_ports) {
        const uint8_t can = uint8_t(remaining);
        char path[16];
        char name[16];
        char display[16];
        hal.util->snprintf(path, sizeof(path), "hw/can/%u", unsigned(can));
        hal.util->snprintf(name, sizeof(name), "can%u", unsigned(can + 1));
        hal.util->snprintf(display, sizeof(display), "CAN %u", unsigned(can + 1));
        return write_endpoint(Protocol::EndpointKind::CAN,
                              id_for_path(path),
                              0,
                              0,
                              name,
                              display);
    }
    remaining -= _capability->can_ports;

    if (remaining < _capability->adc_observables) {
        const uint8_t adc = uint8_t(remaining);
        char path[16];
        char name[16];
        char display[16];
        hal.util->snprintf(path, sizeof(path), "hw/adc/%u", unsigned(adc));
        hal.util->snprintf(name, sizeof(name), "adc%u", unsigned(adc + 1));
        hal.util->snprintf(display, sizeof(display), "ADC %u", unsigned(adc + 1));
        return write_endpoint(Protocol::EndpointKind::ADC,
                              id_for_path(path),
                              0,
                              0,
                              name,
                              display);
    }
    remaining -= _capability->adc_observables;

    if (capability_has(*_capability, CAP_ONBOARD_IMU)) {
        if (remaining == 0) {
            return write_endpoint(Protocol::EndpointKind::SENSOR,
                                  id_for_path("hw/sensor/imu/0"),
                                  0,
                                  0,
                                  "imu0",
                                  _capability->imu_name);
        }
        remaining--;
    }

    if (capability_has(*_capability, CAP_ONBOARD_COMPASS)) {
        if (remaining == 0) {
            return write_endpoint(Protocol::EndpointKind::SENSOR,
                                  id_for_path("hw/sensor/compass/0"),
                                  0,
                                  0,
                                  "compass0",
                                  _capability->compass_name);
        }
        remaining--;
    }

    if (capability_has(*_capability, CAP_STORAGE) && remaining == 0) {
        return write_endpoint(Protocol::EndpointKind::STORAGE,
                              id_for_path("hw/storage/0"),
                              0,
                              0,
                              "storage0",
                              "Storage Hardware");
    }

    record_len = 0;
    return false;
}

bool SerialProtocol::build_timer_group_record(uint16_t index,
                                              uint8_t *record,
                                              uint16_t record_capacity,
                                              uint16_t &record_len)
{
    static const uint16_t supported_rates[] = { 50, 100, 200, 330 };

    const TimerGroup *group = nullptr;
    uint8_t member_start = 0;
    bool flex_group = false;
    uint16_t remaining = index;

    if (remaining < _capability->pwm_timer_group_count) {
        group = &_capability->pwm_timer_groups[remaining];
        for (uint16_t i = 0; i < remaining; i++) {
            member_start += _capability->pwm_timer_groups[i].channel_count;
        }
    } else {
        remaining -= _capability->pwm_timer_group_count;
        if (remaining >= _capability->flex_timer_group_count) {
            record_len = 0;
            return false;
        }
        flex_group = true;
        group = &_capability->flex_timer_groups[remaining];
        for (uint16_t i = 0; i < remaining; i++) {
            member_start += _capability->flex_timer_groups[i].channel_count;
        }
    }

    char path[32];
    if (flex_group) {
        hal.util->snprintf(path,
                           sizeof(path),
                           "hw/timer_group/flex/%u",
                           unsigned(remaining));
    } else {
        hal.util->snprintf(path,
                           sizeof(path),
                           "hw/timer_group/pwm/%u",
                           unsigned(index));
    }

    Protocol::PayloadWriter writer(record, record_capacity);
    writer.u8(1); // record_version
    writer.u32(id_for_path(path));
    writer.u8(group->channel_count);
    for (uint8_t i = 0; i < group->channel_count; i++) {
        const uint8_t member = member_start + i;
        writer.u32(flex_group ?
                   flex_id(member) :
                   pwm_channel_id(member + 1));
    }
    writer.u8(uint8_t(sizeof(supported_rates) / sizeof(supported_rates[0])));
    for (uint8_t i = 0; i < sizeof(supported_rates) / sizeof(supported_rates[0]); i++) {
        writer.u16(supported_rates[i]);
    }
    writer.u16(supported_rates[0]);

    record_len = writer.length();
    return writer.ok();
}

bool SerialProtocol::send_payload(Protocol::MessageType message_type,
                                  uint8_t flags,
                                  uint16_t sequence,
                                  const uint8_t *payload,
                                  uint16_t payload_len)
{
    if (_uart == nullptr) {
        return false;
    }

    uint16_t encoded_len = 0;
    if (!Protocol::build_frame(message_type,
                               flags,
                               sequence,
                               payload,
                               payload_len,
                               _tx_decoded,
                               sizeof(_tx_decoded),
                               _tx_encoded,
                               sizeof(_tx_encoded),
                               encoded_len)) {
        _tx_drops++;
        return false;
    }

    const size_t written = _uart->write(_tx_encoded, encoded_len);
    if (written != encoded_len) {
        _tx_drops++;
        return false;
    }
    if ((flags & Protocol::FLAG_RESPONSE) != 0) {
        remember_cached_response(_tx_encoded, encoded_len);
    }
    return true;
}

bool SerialProtocol::send_value(uint16_t sequence, uint32_t field_id)
{
    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    if (!write_field_value(writer, field_id)) {
        _tx_drops++;
        return false;
    }
    if (!writer.ok()) {
        _tx_drops++;
        return false;
    }

    return send_payload(Protocol::MessageType::VALUE,
                        Protocol::FLAG_RESPONSE,
                        sequence,
                        writer.data(),
                        writer.length());
}

bool SerialProtocol::send_values(uint16_t sequence,
                                 const uint32_t *field_ids,
                                 uint16_t fields)
{
    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u16(fields);
    for (uint16_t i = 0; i < fields; i++) {
        if (!write_field_value(writer, field_ids[i])) {
            _tx_drops++;
            return false;
        }
    }

    if (!writer.ok()) {
        _tx_drops++;
        return false;
    }

    return send_payload(Protocol::MessageType::VALUES,
                        Protocol::FLAG_RESPONSE,
                        sequence,
                        writer.data(),
                        writer.length());
}

bool SerialProtocol::send_all_parameters(uint16_t sequence)
{
    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u16(parameter_field_count());
    for (uint16_t i = 0; i < field_count(); i++) {
        const FieldDescriptor *field = field_descriptor_by_index(i);
        if (field == nullptr ||
            !field_is_parameter(*field) ||
            (field->flags & field_readable) == 0) {
            continue;
        }
        if (!write_field_value(writer, protocol_field_id(i))) {
            _tx_drops++;
            return false;
        }
    }

    if (!writer.ok()) {
        _tx_drops++;
        return false;
    }

    return send_payload(Protocol::MessageType::VALUES,
                        Protocol::FLAG_RESPONSE,
                        sequence,
                        writer.data(),
                        writer.length());
}

bool SerialProtocol::write_field_value(Protocol::PayloadWriter &writer,
                                       uint32_t field_id) const
{
    const FieldDescriptor *field = field_descriptor_by_id(field_id);
    if (field == nullptr) {
        return false;
    }

    uint32_t raw = 0;
    switch (field->slot) {
    case FieldSlot::RX_FRAMES:
        raw = _rx_frames;
        break;
    case FieldSlot::RX_DROPS:
        raw = _rx_drops;
        break;
    case FieldSlot::TX_DROPS:
        raw = _tx_drops;
        break;
    case FieldSlot::UPTIME_MS:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->uptime_ms(AP_HAL::micros64());
        break;
    case FieldSlot::LOOP_COUNT:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->loop_count();
        break;
    case FieldSlot::LOOP_DT_US:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->last_loop_dt_us();
        break;
    case FieldSlot::LOOP_WORK_US:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->last_loop_work_us();
        break;
    case FieldSlot::LOOP_MAX_WORK_US:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->max_loop_work_us();
        break;
    case FieldSlot::SERVICE_RATE_HZ:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->service_rate_hz();
        break;
    case FieldSlot::SYS_OPTIONS:
        if (_parameters == nullptr) {
            return false;
        }
        raw = uint32_t(_parameters->sys_options.get());
        break;
    case FieldSlot::SYS_DESC_PAGE:
        if (_parameters == nullptr) {
            return false;
        }
        raw = uint16_t(_parameters->sys_desc_page.get());
        break;
    case FieldSlot::SYS_PROTOCOL_BAUD:
        if (_parameters == nullptr) {
            return false;
        }
        raw = uint32_t(_parameters->sys_protocol_baud.get());
        break;
    }

    writer.u32(field_id);
    writer.u8(uint8_t(field->type));
    return write_typed_payload(writer, field->type, raw);
}

bool SerialProtocol::validate_parameter_value(uint32_t field_id,
                                              uint8_t type_id,
                                              uint32_t raw,
                                              Protocol::ErrorCode &code,
                                              const char *&detail) const
{
    const FieldDescriptor *field = field_descriptor_by_id(field_id);
    if (field == nullptr) {
        code = Protocol::ErrorCode::BAD_ID;
        detail = "unknown field";
        return false;
    }
    if (!field_is_parameter(*field)) {
        code = Protocol::ErrorCode::READ_ONLY;
        detail = "field is not a parameter";
        return false;
    }
    if ((field->flags & field_writable) == 0) {
        code = Protocol::ErrorCode::READ_ONLY;
        detail = "parameter is read-only";
        return false;
    }
    if (type_id != uint8_t(field->type)) {
        code = Protocol::ErrorCode::TYPE_MISMATCH;
        detail = "type mismatch";
        return false;
    }

    const int32_t value = signed_raw_value(field->type, raw);
    if ((field->flags & field_has_min) != 0 && value < field->min_value) {
        code = Protocol::ErrorCode::OUT_OF_RANGE;
        detail = "below minimum";
        return false;
    }
    if ((field->flags & field_has_max) != 0 && value > field->max_value) {
        code = Protocol::ErrorCode::OUT_OF_RANGE;
        detail = "above maximum";
        return false;
    }

    code = Protocol::ErrorCode::UNKNOWN_MESSAGE;
    detail = "";
    return true;
}

bool SerialProtocol::apply_parameter_value(uint32_t field_id,
                                           uint8_t type_id,
                                           uint32_t raw)
{
    const FieldDescriptor *field = field_descriptor_by_id(field_id);
    if (_parameters == nullptr ||
        field == nullptr ||
        type_id != uint8_t(field->type) ||
        !field_is_parameter(*field)) {
        return false;
    }

    switch (field->slot) {
    case FieldSlot::SYS_OPTIONS:
        _parameters->sys_options.set_and_save_ifchanged(int32_t(raw));
        return true;
    case FieldSlot::SYS_DESC_PAGE:
        _parameters->sys_desc_page.set_and_save_ifchanged(
            int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::SYS_PROTOCOL_BAUD:
        _parameters->sys_protocol_baud.set_and_save_ifchanged(int32_t(raw));
        return true;
    case FieldSlot::RX_FRAMES:
    case FieldSlot::RX_DROPS:
    case FieldSlot::TX_DROPS:
    case FieldSlot::UPTIME_MS:
    case FieldSlot::LOOP_COUNT:
    case FieldSlot::LOOP_DT_US:
    case FieldSlot::LOOP_WORK_US:
    case FieldSlot::LOOP_MAX_WORK_US:
    case FieldSlot::SERVICE_RATE_HZ:
        break;
    }
    return false;
}

bool SerialProtocol::read_typed_value(Protocol::PayloadReader &reader,
                                      Protocol::PrimitiveType type,
                                      uint32_t &raw) const
{
    uint16_t raw16 = 0;
    switch (type) {
    case Protocol::PrimitiveType::U16:
    case Protocol::PrimitiveType::I16:
        if (!reader.u16(raw16)) {
            return false;
        }
        raw = raw16;
        return true;
    case Protocol::PrimitiveType::U32:
    case Protocol::PrimitiveType::I32:
        return reader.u32(raw);
    default:
        return false;
    }
}

bool SerialProtocol::send_error(const Protocol::FrameView &request,
                                Protocol::ErrorCode code,
                                uint32_t object_id,
                                const char *detail)
{
    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u8(uint8_t(request.message_type));
    writer.u16(uint16_t(code));
    writer.u32(object_id);
    writer.str16(detail);
    if (!writer.ok()) {
        _tx_drops++;
        return false;
    }

    return send_payload(Protocol::MessageType::ERROR,
                        Protocol::FLAG_RESPONSE,
                        request.sequence,
                        writer.data(),
                        writer.length());
}

uint64_t SerialProtocol::coarse_capability_flags() const
{
    uint64_t flags = 0;
    if (capability_has(*_capability, CAP_NATIVE_USB)) {
        flags |= 1ULL << 0; // CAP_USB
    }
    flags |= 1ULL << 1; // CAP_UART_VEKTOR: this adapter is active
    if (capability_has(*_capability, CAP_PWM_OUTPUTS)) {
        flags |= 1ULL << 2;
    }
    if (capability_has(*_capability, CAP_FLEX_TIMER_CHANNELS)) {
        flags |= 1ULL << 3;
    }
    if (capability_has(*_capability, CAP_CLASSIC_CAN)) {
        flags |= 1ULL << 4;
    }
    if (capability_has(*_capability, CAP_ONBOARD_IMU)) {
        flags |= 1ULL << 5;
    }
    if (capability_has(*_capability, CAP_ONBOARD_COMPASS)) {
        flags |= 1ULL << 6;
    }
    if (capability_has(*_capability, CAP_ADC_OBSERVABLES)) {
        flags |= 1ULL << 7;
    }
    if (capability_has(*_capability, CAP_STORAGE)) {
        flags |= 1ULL << 8;
        if (_capability->profile == BoardProfile::CORE_REDUCED_F405) {
            flags |= 1ULL << 14; // CAP_SD
            flags |= 1ULL << 15; // CAP_DATAFLASH
        }
    }
    return flags;
}

uint16_t SerialProtocol::endpoint_count() const
{
    uint16_t count = 1; // USB
    if (_capability->pwm_outputs != 0) {
        count += 1; // PWM bank
        count += _capability->pwm_outputs;
    }
    count += _capability->flex_timer_channels;
    count += _capability->dedicated_receiver_rows;
    count += _capability->uart_endpoints;
    count += _capability->can_ports;
    count += _capability->adc_observables;
    if (capability_has(*_capability, CAP_ONBOARD_IMU)) {
        count++;
    }
    if (capability_has(*_capability, CAP_ONBOARD_COMPASS)) {
        count++;
    }
    if (capability_has(*_capability, CAP_STORAGE)) {
        count++;
    }
    return count;
}

uint16_t SerialProtocol::timer_group_count() const
{
    return _capability->pwm_timer_group_count +
           _capability->flex_timer_group_count;
}

uint64_t SerialProtocol::device_id() const
{
    uint8_t raw_id[16] {};
    uint8_t raw_len = sizeof(raw_id);
    if (hal.util->get_system_id_unformatted(raw_id, raw_len) && raw_len != 0) {
        return Protocol::fnv1a64(raw_id, raw_len);
    }
    return 0;
}

uint32_t SerialProtocol::protocol_field_id(uint16_t index) const
{
    const FieldDescriptor *field = field_descriptor_by_index(index);
    if (field == nullptr) {
        return 0;
    }
    return id_for_path(field->path);
}

uint16_t SerialProtocol::parameter_field_count() const
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < field_count(); i++) {
        const FieldDescriptor *field = field_descriptor_by_index(i);
        if (field != nullptr &&
            field_is_parameter(*field) &&
            (field->flags & field_readable) != 0) {
            count++;
        }
    }
    return count;
}

uint16_t SerialProtocol::descriptor_page_limit() const
{
    if (_parameters == nullptr) {
        return default_describe_page_records;
    }

    const int16_t configured = _parameters->sys_desc_page.get();
    if (configured < min_describe_page_records) {
        return min_describe_page_records;
    }
    if (configured > max_describe_page_records) {
        return max_describe_page_records;
    }
    return configured;
}

uint16_t SerialProtocol::component_count() const
{
    return component_descriptor_count;
}

uint16_t SerialProtocol::field_count() const
{
    return field_descriptor_count;
}

} // namespace Vektor
