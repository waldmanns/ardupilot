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
    parameter.save(true);
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
                          const AttitudeSource &attitude,
                          const VspComponent &vsp,
                          const RcinSource &rcin,
                          PwmInput &pwm_input,
                          PwmOutput &pwm_output,
                          AssignmentMatrix &assignments,
                          SerialRoleManager *serial_roles,
                          uint32_t transport_baud)
{
    _uart = uart;
    _capability = &capability;
    _parameters = &parameters;
    _runtime = &runtime;
    _attitude = &attitude;
    _vsp = &vsp;
    _rcin = &rcin;
    _pwm_input = &pwm_input;
    _pwm_output = &pwm_output;
    _assignments = &assignments;
    _serial_roles = serial_roles;
    _server_nonce = uint32_t(AP_HAL::micros64()) ^ uint32_t(device_id());
    _ready = (_uart != nullptr) &&
             capability_valid(capability) &&
             Protocol::self_test() &&
             initialize_descriptor_identity();
    _hello_seen = false;
    _hello_sequence = 0;
    _hello_payload_crc = 0;
    _capture_response = false;
    _request_cache.reset();
    _subscriptions.reset(1000000U / max_realtime_rate_hz);
    _telemetry_sequence = 0;

    if (!_ready) {
        return;
    }

    if (transport_baud < uint32_t(protocol_baud_min) ||
        transport_baud > uint32_t(protocol_baud_max)) {
        transport_baud = protocol_baud;
    }
    _uart->begin(transport_baud, protocol_rx_space, protocol_tx_space);
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

#include "Vektor_SerialProtocol_Requests.inc"

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
        uint8_t quality_mask[SubscriptionTable::max_quality_bytes] {};
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

#include "Vektor_SerialProtocol_Descriptors.inc"

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
        const FieldDescriptor *field = available_field_by_index(i);
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
    if (field == nullptr || !field_available(*field)) {
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
    if (field == nullptr || !field_available(*field)) {
        return false;
    }

    uint32_t raw = 0;
    uint8_t channel_index = 0;
    if (rcin_channel_for_slot(field->slot, channel_index)) {
        if (_rcin == nullptr) {
            return false;
        }
        const SignalSample<float> *sample = _rcin->channel(channel_index);
        if (sample == nullptr) {
            return false;
        }
        raw = float_raw_value(sample->value);
        return write_typed_payload(writer, field->type, raw);
    }
    if (rcin_pwm_channel_for_slot(field->slot, channel_index)) {
        if (_rcin == nullptr) {
            return false;
        }
        raw = _rcin->raw_pwm(channel_index);
        return write_typed_payload(writer, field->type, raw);
    }
    if (pwmin_status_for_slot(field->slot, channel_index)) {
        if (_pwm_input == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_input->attach_status(channel_index));
        return write_typed_payload(writer, field->type, raw);
    }
    if (pwmin_pin_for_slot(field->slot, channel_index)) {
        if (_pwm_input == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_input->pins[channel_index].get());
        return write_typed_payload(writer, field->type, raw);
    }
    if (pwmin_channel_for_slot(field->slot, channel_index)) {
        const SignalSample<float> *sample =
            _pwm_input == nullptr ? nullptr :
                                    _pwm_input->channel(channel_index);
        if (sample == nullptr) {
            return false;
        }
        raw = float_raw_value(sample->value);
        return write_typed_payload(writer, field->type, raw);
    }
    if (pwmout_channel_for_slot(field->slot, channel_index)) {
        const SignalSample<float> *sample =
            _pwm_output == nullptr ? nullptr :
                                     _pwm_output->command(channel_index);
        if (sample == nullptr) {
            return false;
        }
        raw = float_raw_value(sample->value);
        return write_typed_payload(writer, field->type, raw);
    }
    if (pwmout_pulse_channel_for_slot(field->slot, channel_index)) {
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = _pwm_output->pwm_us(channel_index);
        return write_typed_payload(writer, field->type, raw);
    }
    if (serial_role_for_slot(field->slot, channel_index)) {
        if (_serial_roles == nullptr) {
            return false;
        }
        raw = uint32_t(_serial_roles->configured_role(channel_index));
        return write_typed_payload(writer, field->type, raw);
    }
    if (serial_baud_for_slot(field->slot, channel_index)) {
        if (_serial_roles == nullptr) {
            return false;
        }
        raw = _serial_roles->configured_baud(channel_index);
        return write_typed_payload(writer, field->type, raw);
    }
    if (serial_active_role_for_slot(field->slot, channel_index)) {
        if (_serial_roles == nullptr) {
            return false;
        }
        raw = uint32_t(_serial_roles->active_role(channel_index));
        return write_typed_payload(writer, field->type, raw);
    }
    if (serial_status_for_slot(field->slot, channel_index)) {
        if (_serial_roles == nullptr) {
            return false;
        }
        raw = uint32_t(_serial_roles->status(channel_index));
        return write_typed_payload(writer, field->type, raw);
    }
    if (serial_reboot_required_for_slot(field->slot, channel_index)) {
        if (_serial_roles == nullptr) {
            return false;
        }
        raw = _serial_roles->reboot_required(channel_index) ? 1U : 0U;
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
    case FieldSlot::LOOP_LATE:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->last_loop_late() ? 1U : 0U;
        break;
    case FieldSlot::LOOP_LATENESS_US:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->last_loop_lateness_us();
        break;
    case FieldSlot::LOOP_LATE_COUNT:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->late_loop_count();
        break;
    case FieldSlot::SERVICE_RATE_HZ:
        if (_runtime == nullptr) {
            return false;
        }
        raw = _runtime->service_rate_hz();
        break;
    case FieldSlot::ATTITUDE_ROLL_DEG:
        if (_attitude == nullptr) {
            return false;
        }
        raw = float_raw_value(_attitude->value().roll_deg);
        break;
    case FieldSlot::ATTITUDE_PITCH_DEG:
        if (_attitude == nullptr) {
            return false;
        }
        raw = float_raw_value(_attitude->value().pitch_deg);
        break;
    case FieldSlot::ATTITUDE_YAW_DEG:
        if (_attitude == nullptr) {
            return false;
        }
        raw = float_raw_value(_attitude->value().yaw_deg);
        break;
    case FieldSlot::ATTITUDE_QUATERNION:
        if (_attitude == nullptr) {
            return false;
        }
        writer.u32(float_raw_value(_attitude->value().quaternion_w));
        writer.u32(float_raw_value(_attitude->value().quaternion_x));
        writer.u32(float_raw_value(_attitude->value().quaternion_y));
        writer.u32(float_raw_value(_attitude->value().quaternion_z));
        return writer.ok();
    case FieldSlot::ATTITUDE_BODY_RATES_RAD_S:
        if (_attitude == nullptr) {
            return false;
        }
        writer.u32(float_raw_value(_attitude->value().body_rate_x_rad_s));
        writer.u32(float_raw_value(_attitude->value().body_rate_y_rad_s));
        writer.u32(float_raw_value(_attitude->value().body_rate_z_rad_s));
        return writer.ok();
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
    case FieldSlot::USB_MODE:
        if (_serial_roles == nullptr) {
            return false;
        }
        raw = uint32_t(_serial_roles->configured_usb_mode());
        break;
    case FieldSlot::USB_ACTIVE_MODE:
        if (_serial_roles == nullptr) {
            return false;
        }
        raw = uint32_t(_serial_roles->active_usb_mode());
        break;
    case FieldSlot::USB_STATUS:
        if (_serial_roles == nullptr) {
            return false;
        }
        raw = uint32_t(_serial_roles->usb_status());
        break;
    case FieldSlot::USB_REBOOT_REQUIRED:
        if (_serial_roles == nullptr) {
            return false;
        }
        raw = _serial_roles->usb_reboot_required() ? 1U : 0U;
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
    case FieldSlot::PWMIN_TIMEOUT_MS:
        if (_pwm_input == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_input->timeout_ms.get());
        break;
    case FieldSlot::PWMIN_MIN_US:
        if (_pwm_input == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_input->pwm_min.get());
        break;
    case FieldSlot::PWMIN_TRIM_US:
        if (_pwm_input == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_input->pwm_trim.get());
        break;
    case FieldSlot::PWMIN_MAX_US:
        if (_pwm_input == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_input->pwm_max.get());
        break;
    case FieldSlot::PWMOUT_RATE_HZ:
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_output->rate_hz.get());
        break;
    case FieldSlot::PWMOUT_MIN_US:
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_output->pwm_min.get());
        break;
    case FieldSlot::PWMOUT_TRIM_US:
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_output->pwm_trim.get());
        break;
    case FieldSlot::PWMOUT_MAX_US:
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_output->pwm_max.get());
        break;
    case FieldSlot::PWMOUT_REVERSE_MASK:
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_output->reverse_mask.get());
        break;
    case FieldSlot::PWMOUT_FAILSAFE_US:
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = uint16_t(_pwm_output->failsafe_pwm.get());
        break;
    case FieldSlot::PWMIN_CONFIG_VALID:
        if (_pwm_input == nullptr) {
            return false;
        }
        raw = _pwm_input->calibration_valid() ? 1U : 0U;
        break;
    case FieldSlot::PWMOUT_CONFIG_VALID:
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = _pwm_output->configuration_valid() ? 1U : 0U;
        break;
    case FieldSlot::PWMOUT_EFFECTIVE_RATE_HZ:
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = _pwm_output->configuration_valid() ?
            _pwm_output->effective_rate_hz() : 0U;
        break;
    case FieldSlot::PWMOUT_EFFECTIVE_FAILSAFE_US:
        if (_pwm_output == nullptr) {
            return false;
        }
        raw = _pwm_output->effective_failsafe_us();
        break;
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
    case FieldSlot::RCIN_PWM_1:
    case FieldSlot::RCIN_PWM_2:
    case FieldSlot::RCIN_PWM_3:
    case FieldSlot::RCIN_PWM_4:
    case FieldSlot::RCIN_PWM_5:
    case FieldSlot::RCIN_PWM_6:
    case FieldSlot::RCIN_PWM_7:
    case FieldSlot::RCIN_PWM_8:
    case FieldSlot::RCIN_PWM_9:
    case FieldSlot::RCIN_PWM_10:
    case FieldSlot::RCIN_PWM_11:
    case FieldSlot::RCIN_PWM_12:
    case FieldSlot::RCIN_PWM_13:
    case FieldSlot::RCIN_PWM_14:
    case FieldSlot::RCIN_PWM_15:
    case FieldSlot::RCIN_PWM_16:
    case FieldSlot::PWMIN_PIN_1:
    case FieldSlot::PWMIN_PIN_2:
    case FieldSlot::PWMIN_PIN_3:
    case FieldSlot::PWMIN_PIN_4:
    case FieldSlot::PWMIN_PIN_5:
    case FieldSlot::PWMIN_PIN_6:
    case FieldSlot::PWMIN_ATTACH_STATUS_1:
    case FieldSlot::PWMIN_ATTACH_STATUS_2:
    case FieldSlot::PWMIN_ATTACH_STATUS_3:
    case FieldSlot::PWMIN_ATTACH_STATUS_4:
    case FieldSlot::PWMIN_ATTACH_STATUS_5:
    case FieldSlot::PWMIN_ATTACH_STATUS_6:
    case FieldSlot::PWMIN_CHANNEL_1:
    case FieldSlot::PWMIN_CHANNEL_2:
    case FieldSlot::PWMIN_CHANNEL_3:
    case FieldSlot::PWMIN_CHANNEL_4:
    case FieldSlot::PWMIN_CHANNEL_5:
    case FieldSlot::PWMIN_CHANNEL_6:
    case FieldSlot::PWMOUT_CHANNEL_1:
    case FieldSlot::PWMOUT_CHANNEL_2:
    case FieldSlot::PWMOUT_CHANNEL_3:
    case FieldSlot::PWMOUT_CHANNEL_4:
    case FieldSlot::PWMOUT_CHANNEL_5:
    case FieldSlot::PWMOUT_CHANNEL_6:
    case FieldSlot::PWMOUT_CHANNEL_7:
    case FieldSlot::PWMOUT_CHANNEL_8:
    case FieldSlot::PWMOUT_CHANNEL_9:
    case FieldSlot::PWMOUT_CHANNEL_10:
    case FieldSlot::PWMOUT_CHANNEL_11:
    case FieldSlot::PWMOUT_CHANNEL_12:
    case FieldSlot::PWMOUT_PULSE_1:
    case FieldSlot::PWMOUT_PULSE_2:
    case FieldSlot::PWMOUT_PULSE_3:
    case FieldSlot::PWMOUT_PULSE_4:
    case FieldSlot::PWMOUT_PULSE_5:
    case FieldSlot::PWMOUT_PULSE_6:
    case FieldSlot::PWMOUT_PULSE_7:
    case FieldSlot::PWMOUT_PULSE_8:
    case FieldSlot::PWMOUT_PULSE_9:
    case FieldSlot::PWMOUT_PULSE_10:
    case FieldSlot::PWMOUT_PULSE_11:
    case FieldSlot::PWMOUT_PULSE_12:
        return false;
    default:
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
    uint8_t channel_index = 0;
    if (rcin_channel_for_slot(field->slot, channel_index)) {
        const SignalSample<float> *sample =
            _rcin == nullptr ? nullptr : _rcin->channel(channel_index);
        quality = sample == nullptr ? SignalQuality::INVALID :
                                      sample->quality;
    } else if (rcin_pwm_channel_for_slot(field->slot, channel_index)) {
        const SignalSample<float> *sample =
            _rcin == nullptr ? nullptr : _rcin->channel(channel_index);
        quality = sample == nullptr ? SignalQuality::INVALID :
                                      sample->quality;
    } else if (pwmin_channel_for_slot(field->slot, channel_index)) {
        const SignalSample<float> *sample =
            _pwm_input == nullptr ? nullptr :
                                    _pwm_input->channel(channel_index);
        quality = sample == nullptr ? SignalQuality::INVALID :
                                      sample->quality;
    } else if (pwmout_channel_for_slot(field->slot, channel_index)) {
        const SignalSample<float> *sample =
            _pwm_output == nullptr ? nullptr :
                                     _pwm_output->command(channel_index);
        quality = sample == nullptr ? SignalQuality::INVALID :
                                      sample->quality;
    } else if (pwmout_pulse_channel_for_slot(field->slot, channel_index)) {
        quality = _pwm_output != nullptr &&
                          _pwm_output->active(channel_index) ?
                      SignalQuality::VALID : SignalQuality::INVALID;
    }
    switch (field->slot) {
    case FieldSlot::ATTITUDE_ROLL_DEG:
    case FieldSlot::ATTITUDE_PITCH_DEG:
    case FieldSlot::ATTITUDE_YAW_DEG:
    case FieldSlot::ATTITUDE_QUATERNION:
    case FieldSlot::ATTITUDE_BODY_RATES_RAD_S:
        quality = _attitude == nullptr ? SignalQuality::INVALID :
                                         _attitude->quality();
        break;
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
    case FieldSlot::LOOP_LATE:
    case FieldSlot::LOOP_LATENESS_US:
    case FieldSlot::LOOP_LATE_COUNT:
    case FieldSlot::SERVICE_RATE_HZ:
    case FieldSlot::SYS_OPTIONS:
    case FieldSlot::SYS_DESC_PAGE:
    case FieldSlot::USB_MODE:
    case FieldSlot::USB_ACTIVE_MODE:
    case FieldSlot::USB_STATUS:
    case FieldSlot::USB_REBOOT_REQUIRED:
    case FieldSlot::RCIN_TIMEOUT_MS:
    case FieldSlot::RCIN_PROTOCOLS:
    case FieldSlot::PWMIN_PIN_1:
    case FieldSlot::PWMIN_PIN_2:
    case FieldSlot::PWMIN_PIN_3:
    case FieldSlot::PWMIN_PIN_4:
    case FieldSlot::PWMIN_PIN_5:
    case FieldSlot::PWMIN_PIN_6:
    case FieldSlot::PWMIN_ATTACH_STATUS_1:
    case FieldSlot::PWMIN_ATTACH_STATUS_2:
    case FieldSlot::PWMIN_ATTACH_STATUS_3:
    case FieldSlot::PWMIN_ATTACH_STATUS_4:
    case FieldSlot::PWMIN_ATTACH_STATUS_5:
    case FieldSlot::PWMIN_ATTACH_STATUS_6:
    case FieldSlot::PWMIN_TIMEOUT_MS:
    case FieldSlot::PWMIN_MIN_US:
    case FieldSlot::PWMIN_TRIM_US:
    case FieldSlot::PWMIN_MAX_US:
    case FieldSlot::PWMOUT_RATE_HZ:
    case FieldSlot::PWMOUT_MIN_US:
    case FieldSlot::PWMOUT_TRIM_US:
    case FieldSlot::PWMOUT_MAX_US:
    case FieldSlot::PWMOUT_REVERSE_MASK:
    case FieldSlot::PWMOUT_FAILSAFE_US:
    case FieldSlot::PWMIN_CONFIG_VALID:
    case FieldSlot::PWMOUT_CONFIG_VALID:
    case FieldSlot::PWMOUT_EFFECTIVE_RATE_HZ:
    case FieldSlot::PWMOUT_EFFECTIVE_FAILSAFE_US:
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
    case FieldSlot::RCIN_PWM_1:
    case FieldSlot::RCIN_PWM_2:
    case FieldSlot::RCIN_PWM_3:
    case FieldSlot::RCIN_PWM_4:
    case FieldSlot::RCIN_PWM_5:
    case FieldSlot::RCIN_PWM_6:
    case FieldSlot::RCIN_PWM_7:
    case FieldSlot::RCIN_PWM_8:
    case FieldSlot::RCIN_PWM_9:
    case FieldSlot::RCIN_PWM_10:
    case FieldSlot::RCIN_PWM_11:
    case FieldSlot::RCIN_PWM_12:
    case FieldSlot::RCIN_PWM_13:
    case FieldSlot::RCIN_PWM_14:
    case FieldSlot::RCIN_PWM_15:
    case FieldSlot::RCIN_PWM_16:
    case FieldSlot::PWMIN_CHANNEL_1:
    case FieldSlot::PWMIN_CHANNEL_2:
    case FieldSlot::PWMIN_CHANNEL_3:
    case FieldSlot::PWMIN_CHANNEL_4:
    case FieldSlot::PWMIN_CHANNEL_5:
    case FieldSlot::PWMIN_CHANNEL_6:
    case FieldSlot::PWMOUT_CHANNEL_1:
    case FieldSlot::PWMOUT_CHANNEL_2:
    case FieldSlot::PWMOUT_CHANNEL_3:
    case FieldSlot::PWMOUT_CHANNEL_4:
    case FieldSlot::PWMOUT_CHANNEL_5:
    case FieldSlot::PWMOUT_CHANNEL_6:
    case FieldSlot::PWMOUT_CHANNEL_7:
    case FieldSlot::PWMOUT_CHANNEL_8:
    case FieldSlot::PWMOUT_CHANNEL_9:
    case FieldSlot::PWMOUT_CHANNEL_10:
    case FieldSlot::PWMOUT_CHANNEL_11:
    case FieldSlot::PWMOUT_CHANNEL_12:
    case FieldSlot::PWMOUT_PULSE_1:
    case FieldSlot::PWMOUT_PULSE_2:
    case FieldSlot::PWMOUT_PULSE_3:
    case FieldSlot::PWMOUT_PULSE_4:
    case FieldSlot::PWMOUT_PULSE_5:
    case FieldSlot::PWMOUT_PULSE_6:
    case FieldSlot::PWMOUT_PULSE_7:
    case FieldSlot::PWMOUT_PULSE_8:
    case FieldSlot::PWMOUT_PULSE_9:
    case FieldSlot::PWMOUT_PULSE_10:
    case FieldSlot::PWMOUT_PULSE_11:
    case FieldSlot::PWMOUT_PULSE_12:
        break;
    default:
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

#include "Vektor_SerialProtocol_Parameters.inc"

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

#include "Vektor_SerialProtocol_Identity.inc"

} // namespace Vektor
