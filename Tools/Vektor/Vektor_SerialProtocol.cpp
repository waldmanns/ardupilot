#include "Vektor_SerialProtocol.h"

#include "Config.h"
#include "Vektor_Schema.h"

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
static constexpr const char *protocol_runtime_limit_path =
    "runtime_limit/protocol/0";

uint32_t id_for_path(const char *path)
{
    return Vektor::Protocol::fnv1a32(path);
}

int32_t signed_raw_value(Protocol::PrimitiveType type, uint32_t raw)
{
    if (type == Protocol::PrimitiveType::I16) {
        return int16_t(uint16_t(raw));
    }
    return int32_t(raw);
}

uint32_t float_raw_value(float value)
{
    static_assert(sizeof(value) == sizeof(uint32_t),
                  "Vektor FLOAT32 requires a 32-bit float");
    uint32_t raw = 0;
    memcpy(&raw, &value, sizeof(raw));
    return raw;
}

template <typename Parameter, typename Value>
void save_parameter_without_gcs(Parameter &parameter, Value value)
{
    if (parameter.get() == value) {
        return;
    }
    parameter.set(value);
    parameter.save_sync(true, false);
}

AP_Int32 *rcin_protocols_parameter()
{
    enum ap_var_type type = AP_PARAM_NONE;
    AP_Param *parameter = AP_Param::find("RC_PROTOCOLS", &type);
    return type == AP_PARAM_INT32 ? static_cast<AP_Int32 *>(parameter) :
                                    nullptr;
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
                          const RuntimeState &runtime,
                          const VspComponent &vsp,
                          const RcinSource &rcin,
                          AssignmentMatrix &assignments)
{
    _uart = uart;
    _capability = &capability;
    _parameters = &parameters;
    _runtime = &runtime;
    _vsp = &vsp;
    _rcin = &rcin;
    _assignments = &assignments;
    _server_nonce = uint32_t(AP_HAL::micros64()) ^ uint32_t(device_id());
    _ready = (_uart != nullptr) &&
             Protocol::self_test() &&
             initialize_descriptor_identity();
    _hello_seen = false;
    _capture_response = false;
    _request_cache.reset();
    _subscriptions.reset(1000000U / max_realtime_rate_hz);
    _telemetry_sequence = 0;

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

    service_telemetry(AP_HAL::micros64());
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
    case Protocol::MessageType::SUBSCRIBE:
        handle_subscribe(frame);
        break;
    case Protocol::MessageType::UNSUBSCRIBE:
        handle_unsubscribe(frame);
        break;
    case Protocol::MessageType::ROUTE_LIST:
        handle_route_list(frame);
        break;
    case Protocol::MessageType::ROUTE_SET:
        handle_route_set(frame);
        break;
    case Protocol::MessageType::ROUTE_DELETE:
        handle_route_delete(frame);
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
    if (client_min_major > Protocol::MAJOR_VERSION ||
        client_max_major < Protocol::MAJOR_VERSION) {
        send_error(frame,
                   Protocol::ErrorCode::BAD_VERSION,
                   0,
                   "protocol major mismatch");
        return;
    }

    _server_nonce ^= client_nonce;
    _hello_seen = false;
    _request_cache.reset();
    _subscriptions.reset(1000000U / max_realtime_rate_hz);
    _telemetry_sequence = 0;

    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u8(Protocol::MAJOR_VERSION);
    writer.u16(0); // server protocol minor
    append_firmware_version(writer);
    writer.str8(_capability->product_name);
    writer.str8(_capability->hardware_revision);
    writer.u32(_capability->board_id);
    writer.u64(_schema_hash);
    writer.u64(_capability_hash);
    writer.u64(device_id());
    writer.u64(coarse_capability_flags());
    writer.u16(Protocol::MAX_PAYLOAD_SIZE);
    writer.u16(0); // max_file_chunk: file service not implemented yet
    writer.u16(SubscriptionTable::max_subscriptions);
    writer.u16(max_realtime_rate_hz);
    writer.u16(default_service_rate_hz);
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
    case Protocol::DescriptorDomain::RUNTIME_LIMIT:
        send_runtime_limit_descriptions(frame.sequence,
                                        domain,
                                        cursor,
                                        max_records);
        return;
    case Protocol::DescriptorDomain::STORAGE_AREA:
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

    const FieldDescriptor *field = schema_registry().field_by_id(field_id);
    if (field == nullptr) {
        send_error(frame, Protocol::ErrorCode::BAD_ID, field_id, "unknown field");
        return;
    }
    if ((field->flags & FIELD_READABLE) == 0) {
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

    const FieldDescriptor *field = schema_registry().field_by_id(field_id);
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
        const FieldDescriptor *field = schema_registry().field_by_id(field_ids[i]);
        if (field == nullptr) {
            send_error(frame,
                       Protocol::ErrorCode::BAD_ID,
                       field_ids[i],
                       "unknown field");
            return;
        }
        if ((field->flags & FIELD_READABLE) == 0) {
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
            schema_registry().field_by_id(pending[i].field_id);
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

void SerialProtocol::handle_subscribe(const Protocol::FrameView &frame)
{
    if (!_hello_seen) {
        send_error(frame,
                   Protocol::ErrorCode::INVALID_STATE,
                   0,
                   "HELLO required");
        return;
    }

    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint32_t requested_period_us = 0;
    uint16_t count = 0;
    if (!reader.u32(requested_period_us) || !reader.u16(count)) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad SUBSCRIBE");
        return;
    }
    if (count == 0 || count > SubscriptionTable::max_fields) {
        send_error(frame,
                   Protocol::ErrorCode::TOO_MANY_ITEMS,
                   0,
                   "invalid subscription field count");
        return;
    }

    uint32_t field_ids[SubscriptionTable::max_fields];
    for (uint16_t i = 0; i < count; i++) {
        if (!reader.u32(field_ids[i])) {
            send_error(frame,
                       Protocol::ErrorCode::BAD_LENGTH,
                       0,
                       "bad SUBSCRIBE");
            return;
        }

        const FieldDescriptor *field = schema_registry().field_by_id(field_ids[i]);
        if (field == nullptr) {
            send_error(frame,
                       Protocol::ErrorCode::BAD_ID,
                       field_ids[i],
                       "unknown field");
            return;
        }
        if ((field->flags & FIELD_READABLE) == 0 ||
            (field->flags & FIELD_REALTIME) == 0) {
            send_error(frame,
                       Protocol::ErrorCode::NOT_AVAILABLE,
                       field_ids[i],
                       "field is not realtime");
            return;
        }
        for (uint16_t j = 0; j < i; j++) {
            if (field_ids[j] == field_ids[i]) {
                send_error(frame,
                           Protocol::ErrorCode::BAD_ID,
                           field_ids[i],
                           "duplicate field");
                return;
            }
        }
    }
    if (reader.remaining() != 0) {
        send_error(frame, Protocol::ErrorCode::BAD_LENGTH, 0, "bad SUBSCRIBE");
        return;
    }

    const SubscriptionTable::Entry *subscription = nullptr;
    const SubscriptionTable::AddResult result =
        _subscriptions.add(requested_period_us,
                           field_ids,
                           count,
                           AP_HAL::micros64(),
                           subscription);
    if (result == SubscriptionTable::AddResult::FULL) {
        send_error(frame,
                   Protocol::ErrorCode::SUBSCRIPTION_LIMIT,
                   0,
                   "subscription limit reached");
        return;
    }
    if (result != SubscriptionTable::AddResult::OK || subscription == nullptr) {
        send_error(frame,
                   Protocol::ErrorCode::INTERNAL_ERROR,
                   0,
                   "failed SUBSCRIBE");
        return;
    }

    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u16(subscription->id);
    writer.u32(subscription->period_us);
    writer.u16(subscription->field_count);
    for (uint16_t i = 0; i < subscription->field_count; i++) {
        writer.u32(subscription->field_ids[i]);
    }
    if (!writer.ok()) {
        _subscriptions.remove(subscription->id);
        send_error(frame,
                   Protocol::ErrorCode::INTERNAL_ERROR,
                   0,
                   "SUBSCRIBE payload overflow");
        return;
    }

    send_payload(Protocol::MessageType::SUBSCRIBE,
                 Protocol::FLAG_RESPONSE,
                 frame.sequence,
                 writer.data(),
                 writer.length());
}

void SerialProtocol::handle_unsubscribe(const Protocol::FrameView &frame)
{
    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint16_t subscription_id = 0;
    if (!reader.u16(subscription_id) || reader.remaining() != 0) {
        send_error(frame,
                   Protocol::ErrorCode::BAD_LENGTH,
                   0,
                   "bad UNSUBSCRIBE");
        return;
    }
    if (subscription_id == 0 || !_subscriptions.remove(subscription_id)) {
        send_error(frame,
                   Protocol::ErrorCode::BAD_ID,
                   subscription_id,
                   "unknown subscription");
        return;
    }

    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u16(subscription_id);
    send_payload(Protocol::MessageType::UNSUBSCRIBE,
                 Protocol::FLAG_RESPONSE,
                 frame.sequence,
                 writer.data(),
                 writer.length());
}

void SerialProtocol::handle_route_list(const Protocol::FrameView &frame)
{
    if (!_hello_seen) {
        send_error(frame,
                   Protocol::ErrorCode::INVALID_STATE,
                   0,
                   "HELLO required");
        return;
    }

    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint32_t cursor = 0;
    uint16_t max_records = 0;
    if (!reader.u32(cursor) ||
        !reader.u16(max_records) ||
        reader.remaining() != 0) {
        send_error(frame,
                   Protocol::ErrorCode::BAD_LENGTH,
                   0,
                   "bad ROUTE_LIST");
        return;
    }
    if (!send_routes(frame.sequence, cursor, max_records)) {
        send_error(frame,
                   Protocol::ErrorCode::INTERNAL_ERROR,
                   0,
                   "failed ROUTE_LIST");
    }
}

void SerialProtocol::handle_route_set(const Protocol::FrameView &frame)
{
    if (!_hello_seen) {
        send_error(frame,
                   Protocol::ErrorCode::INVALID_STATE,
                   0,
                   "HELLO required");
        return;
    }

    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint32_t source_id = 0;
    uint32_t destination_id = 0;
    uint16_t flags = 0;
    if (!reader.u32(source_id) ||
        !reader.u32(destination_id) ||
        !reader.u16(flags) ||
        reader.remaining() != 0) {
        send_error(frame,
                   Protocol::ErrorCode::BAD_LENGTH,
                   0,
                   "bad ROUTE_SET");
        return;
    }
    if (_assignments == nullptr) {
        send_error(frame,
                   Protocol::ErrorCode::NOT_AVAILABLE,
                   0,
                   "routing unavailable");
        return;
    }

    const AssignmentMatrix::Entry *accepted = nullptr;
    const AssignmentMatrix::SetResult result =
        _assignments->set(source_id, destination_id, flags, accepted);
    if (result != AssignmentMatrix::SetResult::OK || accepted == nullptr) {
        Protocol::ErrorCode code = Protocol::ErrorCode::INVALID_ROUTE;
        uint32_t object_id = destination_id;
        const char *detail = "invalid route";
        switch (result) {
        case AssignmentMatrix::SetResult::BAD_SOURCE:
        case AssignmentMatrix::SetResult::SOURCE_NOT_ROUTABLE:
            object_id = source_id;
            detail = "invalid route source";
            break;
        case AssignmentMatrix::SetResult::BAD_DESTINATION:
        case AssignmentMatrix::SetResult::DESTINATION_NOT_ROUTABLE:
            detail = "invalid route destination";
            break;
        case AssignmentMatrix::SetResult::TYPE_MISMATCH:
            code = Protocol::ErrorCode::TYPE_MISMATCH;
            detail = "route type mismatch";
            break;
        case AssignmentMatrix::SetResult::UNSUPPORTED_FLAGS:
            detail = "unsupported route flags";
            break;
        case AssignmentMatrix::SetResult::CYCLE:
            detail = "route cycle prohibited";
            break;
        case AssignmentMatrix::SetResult::FULL:
            code = Protocol::ErrorCode::TOO_MANY_ITEMS;
            object_id = 0;
            detail = "assignment matrix full";
            break;
        case AssignmentMatrix::SetResult::INVALID_ROUTE_ID:
            object_id = 0;
            detail = "invalid generated route ID";
            break;
        case AssignmentMatrix::SetResult::OK:
            code = Protocol::ErrorCode::INTERNAL_ERROR;
            object_id = 0;
            detail = "route result missing";
            break;
        }
        send_error(frame, code, object_id, detail);
        return;
    }

    if (!send_single_route(frame.sequence, *accepted)) {
        send_error(frame,
                   Protocol::ErrorCode::INTERNAL_ERROR,
                   accepted->route_id,
                   "failed ROUTE_SET");
    }
}

void SerialProtocol::handle_route_delete(const Protocol::FrameView &frame)
{
    if (!_hello_seen) {
        send_error(frame,
                   Protocol::ErrorCode::INVALID_STATE,
                   0,
                   "HELLO required");
        return;
    }

    Protocol::PayloadReader reader(frame.payload, frame.payload_len);
    uint32_t route_id = 0;
    if (!reader.u32(route_id) || reader.remaining() != 0) {
        send_error(frame,
                   Protocol::ErrorCode::BAD_LENGTH,
                   0,
                   "bad ROUTE_DELETE");
        return;
    }
    if (_assignments == nullptr || !_assignments->remove(route_id)) {
        send_error(frame,
                   Protocol::ErrorCode::BAD_ID,
                   route_id,
                   "unknown route");
        return;
    }

    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u32(0);
    writer.u16(0);
    if (!writer.ok() ||
        !send_payload(Protocol::MessageType::ROUTES,
                      Protocol::FLAG_RESPONSE,
                      frame.sequence,
                      writer.data(),
                      writer.length())) {
        send_error(frame,
                   Protocol::ErrorCode::INTERNAL_ERROR,
                   route_id,
                   "failed ROUTE_DELETE");
    }
}

void SerialProtocol::service_telemetry(uint64_t now_us)
{
    for (uint16_t i = 0; i < SubscriptionTable::max_subscriptions; i++) {
        SubscriptionTable::Entry *subscription =
            _subscriptions.claim_due(now_us);
        if (subscription == nullptr) {
            return;
        }

        Protocol::PayloadWriter writer(_payload, sizeof(_payload));
        writer.u16(subscription->id);
        writer.u16(subscription->sample_sequence++);
        writer.u64(now_us);
        const uint8_t quality_len =
            uint8_t((subscription->field_count * 2U + 7U) / 8U);
        writer.u8(quality_len);
        uint8_t quality_mask[2] {};
        for (uint16_t field = 0;
             field < subscription->field_count;
             field++) {
            const uint8_t quality =
                field_quality_code(subscription->field_ids[field]);
            quality_mask[field / 4U] |=
                uint8_t(quality << ((field % 4U) * 2U));
        }
        for (uint8_t quality_byte = 0;
             quality_byte < quality_len;
             quality_byte++) {
            writer.u8(quality_mask[quality_byte]);
        }
        bool values_complete = true;
        for (uint16_t field = 0;
             field < subscription->field_count;
             field++) {
            if (!write_field_payload(writer, subscription->field_ids[field])) {
                values_complete = false;
                break;
            }
        }

        if (!values_complete || !writer.ok()) {
            _tx_drops++;
            continue;
        }
        send_payload(Protocol::MessageType::TELEMETRY,
                     Protocol::FLAG_VOLATILE,
                     _telemetry_sequence++,
                     writer.data(),
                     writer.length());
    }
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
    return send_description_page(sequence,
                                 domain,
                                 cursor,
                                 max_records,
                                 1);
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

bool SerialProtocol::send_runtime_limit_descriptions(uint16_t sequence,
                                                     uint8_t domain,
                                                     uint32_t cursor,
                                                     uint16_t max_records)
{
    return send_description_page(sequence,
                                 domain,
                                 cursor,
                                 max_records,
                                 runtime_limit_count());
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
    if (domain == uint8_t(Protocol::DescriptorDomain::BOARD)) {
        return build_board_record(index, record, record_capacity, record_len);
    }
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
    if (domain == uint8_t(Protocol::DescriptorDomain::RUNTIME_LIMIT)) {
        return build_runtime_limit_record(index,
                                          record,
                                          record_capacity,
                                          record_len);
    }

    record_len = 0;
    return false;
}

bool SerialProtocol::build_board_record(uint16_t index,
                                        uint8_t *record,
                                        uint16_t record_capacity,
                                        uint16_t &record_len)
{
    if (index != 0) {
        record_len = 0;
        return false;
    }

    Protocol::PayloadWriter writer(record, record_capacity);
    writer.u8(1); // record_version
    writer.u32(_capability->board_id);
    writer.str8(_capability->product_name);
    writer.str8(_capability->hardware_revision);
    writer.str8(_capability->product_name);
    record_len = writer.length();
    return writer.ok();
}

bool SerialProtocol::build_component_record(uint16_t index,
                                            uint8_t *record,
                                            uint16_t record_capacity,
                                            uint16_t &record_len)
{
    return schema_registry().build_component_record(index,
                                                    record,
                                                    record_capacity,
                                                    record_len);
}

bool SerialProtocol::build_field_record(uint16_t index,
                                        uint8_t *record,
                                        uint16_t record_capacity,
                                        uint16_t &record_len)
{
    return schema_registry().build_field_record(index,
                                                record,
                                                record_capacity,
                                                record_len);
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

bool SerialProtocol::build_runtime_limit_record(uint16_t index,
                                                uint8_t *record,
                                                uint16_t record_capacity,
                                                uint16_t &record_len)
{
    if (index != 0) {
        record_len = 0;
        return false;
    }

    Protocol::PayloadWriter writer(record, record_capacity);
    writer.u8(1); // record_version
    writer.u32(id_for_path(protocol_runtime_limit_path));
    writer.u64(coarse_capability_flags());
    writer.u16(Protocol::MAX_PAYLOAD_SIZE);
    writer.u16(0); // max_file_chunk: file service not implemented yet
    writer.u16(SubscriptionTable::max_subscriptions);
    writer.u16(max_realtime_rate_hz);
    writer.u16(default_service_rate_hz);
    writer.u16(0); // attitude_update_hz
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

    // Cache a direct response once it is fully encoded, before attempting the
    // transport write. If the write is short, an identical retry can still
    // recover without repeating a request side effect.
    if ((flags & Protocol::FLAG_RESPONSE) != 0) {
        remember_cached_response(_tx_encoded, encoded_len);
    }

    const size_t written = _uart->write(_tx_encoded, encoded_len);
    if (written != encoded_len) {
        _tx_drops++;
        return false;
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
        const FieldDescriptor *field = schema_registry().field_by_index(i);
        if (field == nullptr ||
            !field_is_parameter(*field) ||
            (field->flags & FIELD_READABLE) == 0) {
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

bool SerialProtocol::send_routes(uint16_t sequence,
                                 uint32_t cursor,
                                 uint16_t max_records)
{
    const uint8_t total = _assignments == nullptr ? 0 :
                                                   _assignments->count();
    const uint16_t page_limit =
        (max_records == 0 || max_records > AssignmentMatrix::max_routes) ?
        uint16_t(AssignmentMatrix::max_routes) : max_records;
    const uint32_t start = cursor < total ? cursor : total;
    uint32_t index = start;

    uint8_t records[Protocol::MAX_PAYLOAD_SIZE - 6] {};
    Protocol::PayloadWriter records_writer(records, sizeof(records));
    uint16_t emitted = 0;
    while (index < total && emitted < page_limit) {
        const AssignmentMatrix::Entry *route =
            _assignments->by_index(uint8_t(index));
        if (route == nullptr || records_writer.remaining() < 14) {
            break;
        }
        records_writer.u32(route->route_id);
        records_writer.u32(route->source_output_id);
        records_writer.u32(route->destination_input_id);
        records_writer.u16(route->flags);
        emitted++;
        index++;
    }
    if (!records_writer.ok()) {
        return false;
    }

    const uint32_t next_cursor = index < total ? index : 0;
    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u32(next_cursor);
    writer.u16(emitted);
    writer.bytes(records_writer.data(), records_writer.length());
    if (!writer.ok()) {
        return false;
    }

    uint8_t flags = Protocol::FLAG_RESPONSE;
    if (next_cursor != 0) {
        flags |= Protocol::FLAG_MORE;
    }
    return send_payload(Protocol::MessageType::ROUTES,
                        flags,
                        sequence,
                        writer.data(),
                        writer.length());
}

bool SerialProtocol::send_single_route(
    uint16_t sequence,
    const AssignmentMatrix::Entry &route)
{
    Protocol::PayloadWriter writer(_payload, sizeof(_payload));
    writer.u32(0);
    writer.u16(1);
    writer.u32(route.route_id);
    writer.u32(route.source_output_id);
    writer.u32(route.destination_input_id);
    writer.u16(route.flags);
    return writer.ok() &&
           send_payload(Protocol::MessageType::ROUTES,
                        Protocol::FLAG_RESPONSE,
                        sequence,
                        writer.data(),
                        writer.length());
}

bool SerialProtocol::write_field_value(Protocol::PayloadWriter &writer,
                                       uint32_t field_id) const
{
    const FieldDescriptor *field = schema_registry().field_by_id(field_id);
    if (field == nullptr) {
        return false;
    }

    writer.u32(field_id);
    writer.u8(uint8_t(field->type));
    return write_field_payload(writer, field_id);
}

bool SerialProtocol::write_field_payload(Protocol::PayloadWriter &writer,
                                         uint32_t field_id) const
{
    const FieldDescriptor *field = schema_registry().field_by_id(field_id);
    if (field == nullptr) {
        return false;
    }

    uint32_t raw = 0;
    uint8_t rcin_channel = 0;
    if (rcin_channel_for_slot(field->slot, rcin_channel)) {
        if (_rcin == nullptr) {
            return false;
        }
        const SignalSample<float> *sample = _rcin->channel(rcin_channel);
        if (sample == nullptr) {
            return false;
        }
        raw = float_raw_value(sample->value);
        return write_typed_payload(writer, field->type, raw);
    }

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
    case FieldSlot::RCIN_PORT:
        if (_parameters == nullptr) {
            return false;
        }
        raw = uint16_t(_parameters->rcin_port.get());
        break;
    case FieldSlot::RCIN_TIMEOUT_MS:
        if (_parameters == nullptr) {
            return false;
        }
        raw = uint16_t(_parameters->rcin_timeout_ms.get());
        break;
    case FieldSlot::RCIN_PROTOCOLS: {
        const AP_Int32 *protocols = rcin_protocols_parameter();
        if (protocols == nullptr) {
            return false;
        }
        raw = uint32_t(protocols->get());
        break;
    }
    case FieldSlot::VSP_X:
        if (_vsp == nullptr) {
            return false;
        }
        raw = float_raw_value(_vsp->input_x().value);
        break;
    case FieldSlot::VSP_Y:
        if (_vsp == nullptr) {
            return false;
        }
        raw = float_raw_value(_vsp->input_y().value);
        break;
    case FieldSlot::VSP_SERVO_A:
        if (_vsp == nullptr) {
            return false;
        }
        raw = float_raw_value(_vsp->servo_a().value);
        break;
    case FieldSlot::VSP_SERVO_B:
        if (_vsp == nullptr) {
            return false;
        }
        raw = float_raw_value(_vsp->servo_b().value);
        break;
    case FieldSlot::RCIN_CHANNEL_1:
    case FieldSlot::RCIN_CHANNEL_2:
    case FieldSlot::RCIN_CHANNEL_3:
    case FieldSlot::RCIN_CHANNEL_4:
    case FieldSlot::RCIN_CHANNEL_5:
    case FieldSlot::RCIN_CHANNEL_6:
    case FieldSlot::RCIN_CHANNEL_7:
    case FieldSlot::RCIN_CHANNEL_8:
    case FieldSlot::RCIN_CHANNEL_9:
    case FieldSlot::RCIN_CHANNEL_10:
    case FieldSlot::RCIN_CHANNEL_11:
    case FieldSlot::RCIN_CHANNEL_12:
    case FieldSlot::RCIN_CHANNEL_13:
    case FieldSlot::RCIN_CHANNEL_14:
    case FieldSlot::RCIN_CHANNEL_15:
    case FieldSlot::RCIN_CHANNEL_16:
        return false;
    }

    return write_typed_payload(writer, field->type, raw);
}

uint8_t SerialProtocol::field_quality_code(uint32_t field_id) const
{
    const FieldDescriptor *field = schema_registry().field_by_id(field_id);
    if (field == nullptr) {
        return 2; // INVALID
    }

    SignalQuality quality = SignalQuality::VALID;
    uint8_t rcin_channel = 0;
    if (rcin_channel_for_slot(field->slot, rcin_channel)) {
        const SignalSample<float> *sample =
            _rcin == nullptr ? nullptr : _rcin->channel(rcin_channel);
        quality = sample == nullptr ? SignalQuality::INVALID :
                                      sample->quality;
    }
    switch (field->slot) {
    case FieldSlot::VSP_X:
        quality = _vsp == nullptr ? SignalQuality::INVALID :
                                    _vsp->input_x().quality;
        break;
    case FieldSlot::VSP_Y:
        quality = _vsp == nullptr ? SignalQuality::INVALID :
                                    _vsp->input_y().quality;
        break;
    case FieldSlot::VSP_SERVO_A:
        quality = _vsp == nullptr ? SignalQuality::INVALID :
                                    _vsp->servo_a().quality;
        break;
    case FieldSlot::VSP_SERVO_B:
        quality = _vsp == nullptr ? SignalQuality::INVALID :
                                    _vsp->servo_b().quality;
        break;
    case FieldSlot::RX_FRAMES:
    case FieldSlot::RX_DROPS:
    case FieldSlot::TX_DROPS:
    case FieldSlot::UPTIME_MS:
    case FieldSlot::LOOP_COUNT:
    case FieldSlot::LOOP_DT_US:
    case FieldSlot::LOOP_WORK_US:
    case FieldSlot::LOOP_MAX_WORK_US:
    case FieldSlot::SERVICE_RATE_HZ:
    case FieldSlot::SYS_OPTIONS:
    case FieldSlot::SYS_DESC_PAGE:
    case FieldSlot::SYS_PROTOCOL_BAUD:
    case FieldSlot::RCIN_PORT:
    case FieldSlot::RCIN_TIMEOUT_MS:
    case FieldSlot::RCIN_PROTOCOLS:
    case FieldSlot::RCIN_CHANNEL_1:
    case FieldSlot::RCIN_CHANNEL_2:
    case FieldSlot::RCIN_CHANNEL_3:
    case FieldSlot::RCIN_CHANNEL_4:
    case FieldSlot::RCIN_CHANNEL_5:
    case FieldSlot::RCIN_CHANNEL_6:
    case FieldSlot::RCIN_CHANNEL_7:
    case FieldSlot::RCIN_CHANNEL_8:
    case FieldSlot::RCIN_CHANNEL_9:
    case FieldSlot::RCIN_CHANNEL_10:
    case FieldSlot::RCIN_CHANNEL_11:
    case FieldSlot::RCIN_CHANNEL_12:
    case FieldSlot::RCIN_CHANNEL_13:
    case FieldSlot::RCIN_CHANNEL_14:
    case FieldSlot::RCIN_CHANNEL_15:
    case FieldSlot::RCIN_CHANNEL_16:
        break;
    }

    switch (quality) {
    case SignalQuality::VALID:
        return 0;
    case SignalQuality::STALE:
        return 1;
    case SignalQuality::INVALID:
        return 2;
    }
    return 2;
}

bool SerialProtocol::validate_parameter_value(uint32_t field_id,
                                              uint8_t type_id,
                                              uint32_t raw,
                                              Protocol::ErrorCode &code,
                                              const char *&detail) const
{
    const FieldDescriptor *field = schema_registry().field_by_id(field_id);
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
    if ((field->flags & FIELD_WRITABLE) == 0) {
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
    if ((field->flags & FIELD_HAS_MIN) != 0 && value < field->min_value) {
        code = Protocol::ErrorCode::OUT_OF_RANGE;
        detail = "below minimum";
        return false;
    }
    if ((field->flags & FIELD_HAS_MAX) != 0 && value > field->max_value) {
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
    const FieldDescriptor *field = schema_registry().field_by_id(field_id);
    if (_parameters == nullptr ||
        field == nullptr ||
        type_id != uint8_t(field->type) ||
        !field_is_parameter(*field)) {
        return false;
    }

    switch (field->slot) {
    case FieldSlot::SYS_OPTIONS:
        save_parameter_without_gcs(_parameters->sys_options, int32_t(raw));
        return true;
    case FieldSlot::SYS_DESC_PAGE:
        save_parameter_without_gcs(_parameters->sys_desc_page,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::SYS_PROTOCOL_BAUD:
        save_parameter_without_gcs(_parameters->sys_protocol_baud,
                                   int32_t(raw));
        return true;
    case FieldSlot::RCIN_PORT:
        save_parameter_without_gcs(_parameters->rcin_port,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::RCIN_TIMEOUT_MS:
        save_parameter_without_gcs(_parameters->rcin_timeout_ms,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::RCIN_PROTOCOLS: {
        AP_Int32 *protocols = rcin_protocols_parameter();
        if (protocols == nullptr) {
            return false;
        }
        save_parameter_without_gcs(*protocols, int32_t(raw));
        return true;
    }
    case FieldSlot::RX_FRAMES:
    case FieldSlot::RX_DROPS:
    case FieldSlot::TX_DROPS:
    case FieldSlot::UPTIME_MS:
    case FieldSlot::LOOP_COUNT:
    case FieldSlot::LOOP_DT_US:
    case FieldSlot::LOOP_WORK_US:
    case FieldSlot::LOOP_MAX_WORK_US:
    case FieldSlot::SERVICE_RATE_HZ:
    case FieldSlot::VSP_X:
    case FieldSlot::VSP_Y:
    case FieldSlot::VSP_SERVO_A:
    case FieldSlot::VSP_SERVO_B:
    case FieldSlot::RCIN_CHANNEL_1:
    case FieldSlot::RCIN_CHANNEL_2:
    case FieldSlot::RCIN_CHANNEL_3:
    case FieldSlot::RCIN_CHANNEL_4:
    case FieldSlot::RCIN_CHANNEL_5:
    case FieldSlot::RCIN_CHANNEL_6:
    case FieldSlot::RCIN_CHANNEL_7:
    case FieldSlot::RCIN_CHANNEL_8:
    case FieldSlot::RCIN_CHANNEL_9:
    case FieldSlot::RCIN_CHANNEL_10:
    case FieldSlot::RCIN_CHANNEL_11:
    case FieldSlot::RCIN_CHANNEL_12:
    case FieldSlot::RCIN_CHANNEL_13:
    case FieldSlot::RCIN_CHANNEL_14:
    case FieldSlot::RCIN_CHANNEL_15:
    case FieldSlot::RCIN_CHANNEL_16:
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
    case Protocol::PrimitiveType::FLOAT32:
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

bool SerialProtocol::initialize_descriptor_identity()
{
    static const uint8_t schema_domains[] = {
        uint8_t(Protocol::DescriptorDomain::COMPONENT),
        uint8_t(Protocol::DescriptorDomain::FIELD),
    };
    static const uint8_t capability_domains[] = {
        uint8_t(Protocol::DescriptorDomain::BOARD),
        uint8_t(Protocol::DescriptorDomain::ENDPOINT),
        uint8_t(Protocol::DescriptorDomain::TIMER_GROUP),
        uint8_t(Protocol::DescriptorDomain::RUNTIME_LIMIT),
    };

    _schema_hash = 0;
    _capability_hash = 0;
    return validate_descriptor_ids() &&
           calculate_descriptor_hash(schema_domains,
                                     sizeof(schema_domains),
                                     _schema_hash) &&
           calculate_descriptor_hash(capability_domains,
                                     sizeof(capability_domains),
                                     _capability_hash);
}

bool SerialProtocol::validate_descriptor_ids()
{
    static constexpr uint16_t max_stable_ids = 96;
    uint32_t stable_ids[max_stable_ids];
    uint16_t count = 0;

    auto append_id = [&](uint32_t id) {
        if (count >= max_stable_ids) {
            return false;
        }
        stable_ids[count++] = id;
        return true;
    };

    const SchemaRegistry &registry = schema_registry();
    for (uint16_t i = 0; i < registry.component_count(); i++) {
        if (!append_id(registry.component_id(i)) ||
            !append_id(registry.component_type_id(i))) {
            return false;
        }
    }

    static const uint8_t object_domains[] = {
        uint8_t(Protocol::DescriptorDomain::FIELD),
        uint8_t(Protocol::DescriptorDomain::ENDPOINT),
        uint8_t(Protocol::DescriptorDomain::TIMER_GROUP),
        uint8_t(Protocol::DescriptorDomain::RUNTIME_LIMIT),
    };
    for (uint8_t domain_index = 0;
         domain_index < sizeof(object_domains);
         domain_index++) {
        const uint8_t domain = object_domains[domain_index];
        for (uint16_t index = 0; index < descriptor_count(domain); index++) {
            uint32_t id = 0;
            if (!descriptor_id(domain, index, id) || !append_id(id)) {
                return false;
            }
        }
    }

    return Protocol::stable_ids_unique_nonzero(stable_ids, count);
}

bool SerialProtocol::calculate_descriptor_hash(const uint8_t *domains,
                                               uint8_t domains_count,
                                               uint64_t &hash)
{
    if (domains == nullptr || domains_count == 0) {
        hash = 0;
        return false;
    }

    uint16_t total_records = 0;
    for (uint8_t i = 0; i < domains_count; i++) {
        total_records += descriptor_count(domains[i]);
    }
    if (total_records == 0) {
        hash = 0;
        return false;
    }

    hash = 0xCBF29CE484222325ULL;
    uint32_t previous_id = 0;
    bool have_previous = false;
    for (uint16_t emitted = 0; emitted < total_records; emitted++) {
        bool found = false;
        uint32_t selected_id = 0;
        uint8_t selected_domain = 0;
        uint16_t selected_index = 0;

        for (uint8_t domain_index = 0;
             domain_index < domains_count;
             domain_index++) {
            const uint8_t domain = domains[domain_index];
            for (uint16_t index = 0;
                 index < descriptor_count(domain);
                 index++) {
                uint32_t id = 0;
                if (!descriptor_id(domain, index, id)) {
                    hash = 0;
                    return false;
                }
                if (have_previous && id <= previous_id) {
                    continue;
                }
                if (!found || id < selected_id) {
                    found = true;
                    selected_id = id;
                    selected_domain = domain;
                    selected_index = index;
                }
            }
        }
        if (!found) {
            hash = 0;
            return false;
        }

        uint8_t record[128];
        uint16_t record_len = 0;
        if (!build_descriptor_record(selected_domain,
                                     selected_index,
                                     record,
                                     sizeof(record),
                                     record_len)) {
            hash = 0;
            return false;
        }
        const uint8_t envelope[] = {
            selected_domain,
            uint8_t(record_len),
            uint8_t(record_len >> 8),
        };
        hash = Protocol::fnv1a64_update(hash, envelope, sizeof(envelope));
        hash = Protocol::fnv1a64_update(hash, record, record_len);
        previous_id = selected_id;
        have_previous = true;
    }
    return true;
}

bool SerialProtocol::descriptor_id(uint8_t domain,
                                   uint16_t index,
                                   uint32_t &id)
{
    uint8_t record[128];
    uint16_t record_len = 0;
    if (!build_descriptor_record(domain,
                                 index,
                                 record,
                                 sizeof(record),
                                 record_len)) {
        return false;
    }

    Protocol::PayloadReader reader(record, record_len);
    uint8_t record_version = 0;
    return reader.u8(record_version) &&
           record_version != 0 &&
           reader.u32(id);
}

uint16_t SerialProtocol::descriptor_count(uint8_t domain) const
{
    switch (Protocol::DescriptorDomain(domain)) {
    case Protocol::DescriptorDomain::BOARD:
        return 1;
    case Protocol::DescriptorDomain::COMPONENT:
        return component_count();
    case Protocol::DescriptorDomain::FIELD:
        return field_count();
    case Protocol::DescriptorDomain::ENDPOINT:
        return endpoint_count();
    case Protocol::DescriptorDomain::TIMER_GROUP:
        return timer_group_count();
    case Protocol::DescriptorDomain::RUNTIME_LIMIT:
        return runtime_limit_count();
    case Protocol::DescriptorDomain::STORAGE_AREA:
    case Protocol::DescriptorDomain::ENUM_TABLE:
    case Protocol::DescriptorDomain::EVENT_TYPE:
    case Protocol::DescriptorDomain::ACTION_SCHEMA:
        return 0;
    }
    return 0;
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
    flags |= 1ULL << 11; // CAP_ROUTING
    flags |= 1ULL << 13; // CAP_REALTIME
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

uint16_t SerialProtocol::runtime_limit_count() const
{
    return 1;
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
    return schema_registry().field_id(index);
}

uint16_t SerialProtocol::parameter_field_count() const
{
    return schema_registry().parameter_count();
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
    return schema_registry().component_count();
}

uint16_t SerialProtocol::field_count() const
{
    return schema_registry().field_count();
}

} // namespace Vektor
