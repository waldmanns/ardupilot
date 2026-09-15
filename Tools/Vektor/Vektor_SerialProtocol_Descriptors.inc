// Descriptor paging and serialization. Included inside namespace Vektor.

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

bool SerialProtocol::send_enum_descriptions(uint16_t sequence,
                                            uint8_t domain,
                                            uint32_t cursor,
                                            uint16_t max_records)
{
    return send_description_page(sequence,
                                 domain,
                                 cursor,
                                 max_records,
                                 enum_count());
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
        uint8_t record[247];
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
    if (domain == uint8_t(Protocol::DescriptorDomain::ENUM_TABLE)) {
        return build_enum_record(index,
                                 record,
                                 record_capacity,
                                 record_len);
    }

    record_len = 0;
    return false;
}

bool SerialProtocol::build_enum_record(uint16_t index,
                                       uint8_t *record,
                                       uint16_t record_capacity,
                                       uint16_t &record_len)
{
    struct EnumValue {
        int32_t value;
        const char *name;
    };
    static constexpr EnumValue roles[] = {
        { int32_t(SerialRole::NONE), "NONE" },
        { int32_t(SerialRole::RCIN), "RCIN" },
        { int32_t(SerialRole::VEKTOR), "VEKTOR" },
        { int32_t(SerialRole::MAVLINK1), "MAVLINK1_RESERVED" },
        { int32_t(SerialRole::MAVLINK2), "MAVLINK2_RESERVED" },
    };
    static constexpr EnumValue usb_modes[] = {
        { int32_t(UsbMode::VEKTOR_ONLY), "VEKTOR_ONLY" },
        { int32_t(UsbMode::VEKTOR_MAVLINK),
          "VEKTOR_MAVLINK_RESERVED" },
    };
    static constexpr EnumValue statuses[] = {
        { int32_t(SerialEndpointStatus::OK), "OK" },
        { int32_t(SerialEndpointStatus::RESERVED_NOT_IMPLEMENTED),
          "RESERVED_NOT_IMPLEMENTED" },
        { int32_t(SerialEndpointStatus::INVALID_ROLE), "INVALID_ROLE" },
        { int32_t(SerialEndpointStatus::UNSUPPORTED_ROLE),
          "UNSUPPORTED_ROLE" },
        { int32_t(SerialEndpointStatus::DUPLICATE_RCIN), "DUPLICATE_RCIN" },
        { int32_t(SerialEndpointStatus::DUPLICATE_VEKTOR),
          "DUPLICATE_VEKTOR" },
        { int32_t(SerialEndpointStatus::INVALID_BAUD), "INVALID_BAUD" },
        { int32_t(SerialEndpointStatus::ENDPOINT_UNAVAILABLE),
          "ENDPOINT_UNAVAILABLE" },
        { int32_t(SerialEndpointStatus::CONFIGURATION_INVALID),
          "CONFIGURATION_INVALID" },
    };

    const char *path = nullptr;
    const char *name = nullptr;
    const char *display_name = nullptr;
    const EnumValue *values = nullptr;
    uint8_t count = 0;
    switch (index) {
    case 0:
        path = "enum/serial_role";
        name = "serial_role";
        display_name = "Serial Role";
        values = roles;
        count = sizeof(roles) / sizeof(roles[0]);
        break;
    case 1:
        path = "enum/usb_mode";
        name = "usb_mode";
        display_name = "USB Mode";
        values = usb_modes;
        count = sizeof(usb_modes) / sizeof(usb_modes[0]);
        break;
    case 2:
        path = "enum/serial_endpoint_status";
        name = "serial_endpoint_status";
        display_name = "Serial Endpoint Status";
        values = statuses;
        count = sizeof(statuses) / sizeof(statuses[0]);
        break;
    default:
        record_len = 0;
        return false;
    }

    Protocol::PayloadWriter writer(record, record_capacity);
    writer.u8(1);
    writer.u32(id_for_path(path));
    writer.u8(uint8_t(Protocol::PrimitiveType::ENUM));
    writer.str8(name);
    writer.str8(display_name);
    writer.u8(count);
    for (uint8_t i = 0; i < count; i++) {
        writer.u32(uint32_t(values[i].value));
        writer.str8(values[i].name);
    }
    record_len = writer.length();
    return writer.ok();
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
    uint16_t schema_index = 0;
    if (available_field_by_index(index, &schema_index) == nullptr) {
        record_len = 0;
        return false;
    }
    return schema_registry().build_field_record(schema_index,
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

    for (uint8_t serial_index = 0;
         serial_index < _capability->serial_endpoint_count;
         serial_index++) {
        if (!serial_endpoint_usable(*_capability, serial_index)) {
            continue;
        }
        if (remaining != 0) {
            remaining--;
            continue;
        }

        const bool usb = serial_endpoint_is_usb(*_capability, serial_index);
        const uint8_t physical_flags =
            _capability->serial_endpoint_flags[serial_index];
        uint64_t flags = 0;
        if ((physical_flags & SERIAL_ENDPOINT_INPUT) != 0) {
            flags |= ENDPOINT_INPUT;
        }
        if ((physical_flags & SERIAL_ENDPOINT_OUTPUT) != 0) {
            flags |= ENDPOINT_OUTPUT;
        }
        if (usb || (_serial_roles != nullptr &&
                    _serial_roles->active_role(serial_index) ==
                        SerialRole::VEKTOR)) {
            flags |= ENDPOINT_VEKTOR_TRANSPORT;
        }

        char path[24];
        char name[16];
        char display[40];
        hal.util->snprintf(path, sizeof(path), "hw/serial/%u",
                           unsigned(serial_index));
        hal.util->snprintf(name, sizeof(name), "serial%u",
                           unsigned(serial_index));
        hal.util->snprintf(display, sizeof(display), "SERIAL%u (%s)",
                           unsigned(serial_index),
                           _capability->serial_endpoint_names[serial_index]);

        uint32_t configured_id = 0;
        uint32_t active_id = 0;
        uint32_t baud_id = 0;
        uint32_t status_id = 0;
        uint32_t reboot_id = 0;
        const char *enum_path = nullptr;
        uint32_t supported = 0;
        if (usb) {
            configured_id = id_for_path(
                "component/system/0/parameter/usb_mode");
            active_id = id_for_path(
                "component/system/0/observable/usb_active_mode");
            status_id = id_for_path(
                "component/system/0/observable/usb_status");
            reboot_id = id_for_path(
                "component/system/0/observable/usb_reboot_required");
            enum_path = "enum/usb_mode";
            supported = (1U << uint8_t(UsbMode::VEKTOR_ONLY)) |
                        (1U << uint8_t(UsbMode::VEKTOR_MAVLINK));
        } else {
            char field_path[72];
            hal.util->snprintf(field_path, sizeof(field_path),
                "component/system/0/parameter/serial_%u_role",
                unsigned(serial_index));
            configured_id = id_for_path(field_path);
            hal.util->snprintf(field_path, sizeof(field_path),
                "component/system/0/observable/serial_%u_active_role",
                unsigned(serial_index));
            active_id = id_for_path(field_path);
            hal.util->snprintf(field_path, sizeof(field_path),
                "component/system/0/parameter/serial_%u_baud",
                unsigned(serial_index));
            baud_id = id_for_path(field_path);
            hal.util->snprintf(field_path, sizeof(field_path),
                "component/system/0/observable/serial_%u_status",
                unsigned(serial_index));
            status_id = id_for_path(field_path);
            hal.util->snprintf(field_path, sizeof(field_path),
                "component/system/0/observable/serial_%u_reboot_required",
                unsigned(serial_index));
            reboot_id = id_for_path(field_path);
            enum_path = "enum/serial_role";
            supported = serial_endpoint_supported_role_mask(*_capability,
                                                             serial_index);
        }

        Protocol::PayloadWriter writer(record, record_capacity);
        writer.u8(2); // SERIAL_ORDER-backed endpoint extension
        writer.u32(id_for_path(path));
        writer.u8(uint8_t(usb ? Protocol::EndpointKind::USB :
                               Protocol::EndpointKind::UART));
        writer.u32(0);
        writer.u64(flags);
        writer.str8(name);
        writer.str8(display);
        writer.u8(serial_index);
        writer.str8(_capability->serial_endpoint_names[serial_index]);
        writer.u32(id_for_path(enum_path));
        writer.u32(supported);
        writer.u32(configured_id);
        writer.u32(active_id);
        writer.u32(baud_id);
        writer.u32(status_id);
        writer.u32(reboot_id);
        record_len = writer.length();
        return writer.ok();
    }

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
                              _capability->flex_mode_flags[port],
                              name,
                              display);
    }
    remaining -= _capability->flex_timer_channels;

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
    bool flex_group = false;
    uint16_t remaining = index;

    if (remaining < _capability->pwm_timer_group_count) {
        group = &_capability->pwm_timer_groups[remaining];
    } else {
        remaining -= _capability->pwm_timer_group_count;
        if (remaining >= _capability->flex_timer_group_count) {
            record_len = 0;
            return false;
        }
        flex_group = true;
        group = &_capability->flex_timer_groups[remaining];
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
    uint8_t member_count = 0;
    for (uint8_t member = 0; member < 16; member++) {
        if ((group->channel_mask & (1U << member)) != 0) {
            member_count++;
        }
    }
    writer.u8(member_count);
    for (uint8_t member = 0; member < 16; member++) {
        if ((group->channel_mask & (1U << member)) == 0) {
            continue;
        }
        writer.u32(flex_group ? flex_id(member) :
                               pwm_channel_id(member + 1));
    }
    uint8_t rate_count = 0;
    for (uint8_t i = 0; i < sizeof(supported_rates) / sizeof(supported_rates[0]); i++) {
        if ((group->supported_rate_flags & (1U << i)) != 0) {
            rate_count++;
        }
    }
    writer.u8(rate_count);
    for (uint8_t i = 0; i < sizeof(supported_rates) / sizeof(supported_rates[0]); i++) {
        if ((group->supported_rate_flags & (1U << i)) != 0) {
            writer.u16(supported_rates[i]);
        }
    }
    writer.u16(flex_group || _pwm_output == nullptr ||
                       !_pwm_output->configuration_valid() ?
                   0 :
                   _pwm_output->effective_rate_hz());

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
    writer.u16(capability_has(*_capability, CAP_ONBOARD_IMU) ?
               attitude_update_rate_hz : 0);
    record_len = writer.length();
    return writer.ok();
}
