// Session-gated request dispatch and request handlers.
// Included inside namespace Vektor.

void SerialProtocol::handle_frame(const Protocol::FrameView &frame)
{
    if ((frame.flags & Protocol::FLAG_RESPONSE) != 0) {
        return;
    }

    if (frame.message_type == Protocol::MessageType::HELLO) {
        const uint32_t payload_crc =
            Protocol::crc32_iso_hdlc(frame.payload, frame.payload_len);
        if (_hello_seen && frame.sequence == _hello_sequence &&
            payload_crc == _hello_payload_crc &&
            replay_cached_response_or_reject(frame)) {
            return;
        }
    } else {
        if (!_hello_seen) {
            send_error(frame,
                       Protocol::ErrorCode::INVALID_STATE,
                       0,
                       "HELLO required");
            return;
        }
        if (replay_cached_response_or_reject(frame)) {
            return;
        }
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

    (void)client_nonce;
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
    writer.u16(capability_has(*_capability, CAP_ONBOARD_IMU) ?
               attitude_update_rate_hz : 0);
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
    if (_hello_seen) {
        _hello_sequence = frame.sequence;
        _hello_payload_crc =
            Protocol::crc32_iso_hdlc(frame.payload, frame.payload_len);
    }
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
    case Protocol::DescriptorDomain::ENUM_TABLE:
        send_enum_descriptions(frame.sequence, domain, cursor, max_records);
        return;
    case Protocol::DescriptorDomain::STORAGE_AREA:
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
    if (!field_available(*field)) {
        send_error(frame,
                   Protocol::ErrorCode::NOT_AVAILABLE,
                   field_id,
                   "field unavailable on this board");
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
    if (!field_available(*field)) {
        send_error(frame,
                   Protocol::ErrorCode::NOT_AVAILABLE,
                   field_id,
                   "field unavailable on this board");
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

    const ParameterUpdate update { field_id, type_id, raw };
    uint32_t failing_field_id = field_id;
    if (!validate_parameter_set(&update,
                                1,
                                code,
                                failing_field_id,
                                detail)) {
        send_error(frame, code, failing_field_id, detail);
        return;
    }

    if (!apply_parameter_value(field_id, type_id, raw)) {
        send_error(frame,
                   Protocol::ErrorCode::INTERNAL_ERROR,
                   field_id,
                   "failed to apply parameter");
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
        if (!field_available(*field)) {
            send_error(frame,
                       Protocol::ErrorCode::NOT_AVAILABLE,
                       field_ids[i],
                       "field unavailable on this board");
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

    ParameterUpdate pending[8];
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
        if (!field_available(*field)) {
            send_error(frame,
                       Protocol::ErrorCode::NOT_AVAILABLE,
                       pending[i].field_id,
                       "field unavailable on this board");
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

    Protocol::ErrorCode batch_code = Protocol::ErrorCode::INTERNAL_ERROR;
    uint32_t failing_field_id = 0;
    const char *batch_detail = "bad SET_MANY";
    if (!validate_parameter_set(pending,
                                count,
                                batch_code,
                                failing_field_id,
                                batch_detail)) {
        send_error(frame, batch_code, failing_field_id, batch_detail);
        return;
    }

    for (uint16_t i = 0; i < count; i++) {
        if (!apply_parameter_value(pending[i].field_id,
                                   pending[i].type_id,
                                   pending[i].raw)) {
            send_error(frame,
                       Protocol::ErrorCode::INTERNAL_ERROR,
                       pending[i].field_id,
                       "failed to apply parameter");
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
        if (!field_available(*field)) {
            send_error(frame,
                       Protocol::ErrorCode::NOT_AVAILABLE,
                       field_ids[i],
                       "field unavailable on this board");
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

    uint16_t telemetry_bytes = 2 + 2 + 8 + 1 +
        uint16_t((count * 2U + 7U) / 8U);
    for (uint16_t i = 0; i < count; i++) {
        const FieldDescriptor *field = schema_registry().field_by_id(field_ids[i]);
        const uint16_t value_size = field == nullptr ? 0 :
            Protocol::primitive_size(field->type);
        if (value_size == 0 ||
            telemetry_bytes > Protocol::MAX_PAYLOAD_SIZE - value_size) {
            send_error(frame,
                       Protocol::ErrorCode::PAYLOAD_TOO_LARGE,
                       field_ids[i],
                       "subscription sample exceeds payload limit");
            return;
        }
        telemetry_bytes += value_size;
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
