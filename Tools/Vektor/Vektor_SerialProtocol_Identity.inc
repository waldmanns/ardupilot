// Descriptor identity, capability hashing, and board field filtering.
// Included inside namespace Vektor.

bool SerialProtocol::initialize_descriptor_identity()
{
    static const uint8_t schema_domains[] = {
        uint8_t(Protocol::DescriptorDomain::COMPONENT),
        uint8_t(Protocol::DescriptorDomain::FIELD),
    };
    _schema_hash = 0;
    _capability_hash = 0;
    return validate_descriptor_ids() &&
           calculate_descriptor_hash(schema_domains,
                                     sizeof(schema_domains),
                                     _schema_hash) &&
           refresh_capability_hash();
}

bool SerialProtocol::refresh_capability_hash()
{
    static const uint8_t capability_domains[] = {
        uint8_t(Protocol::DescriptorDomain::BOARD),
        uint8_t(Protocol::DescriptorDomain::ENDPOINT),
        uint8_t(Protocol::DescriptorDomain::TIMER_GROUP),
        uint8_t(Protocol::DescriptorDomain::RUNTIME_LIMIT),
        uint8_t(Protocol::DescriptorDomain::ENUM_TABLE),
    };
    return calculate_descriptor_hash(capability_domains,
                                     sizeof(capability_domains),
                                     _capability_hash);
}

bool SerialProtocol::validate_descriptor_ids()
{
    static constexpr uint16_t max_stable_ids = 256;
    static uint32_t stable_ids[max_stable_ids];
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
        uint8_t(Protocol::DescriptorDomain::ENUM_TABLE),
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

        uint8_t record[247];
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
    uint8_t record[247];
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
    case Protocol::DescriptorDomain::ENUM_TABLE:
        return enum_count();
    case Protocol::DescriptorDomain::STORAGE_AREA:
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
    if (_serial_roles != nullptr) {
        for (uint8_t i = 0; i < _capability->serial_endpoint_count; i++) {
            if (!serial_endpoint_is_usb(*_capability, i) &&
                _serial_roles->active_role(i) == SerialRole::VEKTOR) {
                flags |= 1ULL << 1; // CAP_UART_VEKTOR
                break;
            }
        }
    }
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
    }
    if (capability_has(*_capability, CAP_SD_STORAGE)) {
        flags |= 1ULL << 14;
    }
    if (capability_has(*_capability, CAP_DATAFLASH_STORAGE)) {
        flags |= 1ULL << 15;
    }
    flags |= 1ULL << 11; // CAP_ROUTING
    flags |= 1ULL << 13; // CAP_REALTIME
    return flags;
}

uint16_t SerialProtocol::endpoint_count() const
{
    uint16_t count = 0;
    for (uint8_t i = 0; i < _capability->serial_endpoint_count; i++) {
        if (serial_endpoint_usable(*_capability, i)) {
            count++;
        }
    }
    if (_capability->pwm_outputs != 0) {
        count += 1; // PWM bank
        count += _capability->pwm_outputs;
    }
    count += _capability->flex_timer_channels;
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

uint16_t SerialProtocol::enum_count() const
{
    return 3;
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
    uint16_t schema_index = 0;
    return available_field_by_index(index, &schema_index) == nullptr ?
        0 : schema_registry().field_id(schema_index);
}

uint16_t SerialProtocol::parameter_field_count() const
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < schema_registry().field_count(); i++) {
        const FieldDescriptor *field = schema_registry().field_by_index(i);
        if (field != nullptr && field_available(*field) &&
            field_is_parameter(*field) &&
            (field->flags & FIELD_READABLE) != 0) {
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
    return schema_registry().component_count();
}

uint16_t SerialProtocol::field_count() const
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < schema_registry().field_count(); i++) {
        const FieldDescriptor *field = schema_registry().field_by_index(i);
        if (field != nullptr && field_available(*field)) {
            count++;
        }
    }
    return count;
}

const FieldDescriptor *SerialProtocol::available_field_by_index(
    uint16_t index,
    uint16_t *schema_index) const
{
    uint16_t available_index = 0;
    for (uint16_t i = 0; i < schema_registry().field_count(); i++) {
        const FieldDescriptor *field = schema_registry().field_by_index(i);
        if (field == nullptr || !field_available(*field)) {
            continue;
        }
        if (available_index++ == index) {
            if (schema_index != nullptr) {
                *schema_index = i;
            }
            return field;
        }
    }
    return nullptr;
}

bool SerialProtocol::field_available(const FieldDescriptor &field) const
{
    if (_capability == nullptr) {
        return false;
    }

    uint8_t channel_index = 0;
    if (pwmin_pin_for_slot(field.slot, channel_index) ||
        pwmin_status_for_slot(field.slot, channel_index) ||
        pwmin_channel_for_slot(field.slot, channel_index)) {
        return channel_index < _capability->pwm_inputs;
    }
    if (pwmout_channel_for_slot(field.slot, channel_index) ||
        pwmout_pulse_channel_for_slot(field.slot, channel_index)) {
        return channel_index < _capability->pwm_outputs;
    }
    if (serial_role_for_slot(field.slot, channel_index) ||
        serial_baud_for_slot(field.slot, channel_index) ||
        serial_active_role_for_slot(field.slot, channel_index) ||
        serial_status_for_slot(field.slot, channel_index) ||
        serial_reboot_required_for_slot(field.slot, channel_index)) {
        return _serial_roles != nullptr &&
               channel_index < _capability->serial_endpoint_count &&
               serial_endpoint_usable(*_capability, channel_index) &&
               !serial_endpoint_is_usb(*_capability, channel_index);
    }

    switch (field.slot) {
    case FieldSlot::ATTITUDE_ROLL_DEG:
    case FieldSlot::ATTITUDE_PITCH_DEG:
    case FieldSlot::ATTITUDE_YAW_DEG:
    case FieldSlot::ATTITUDE_QUATERNION:
    case FieldSlot::ATTITUDE_BODY_RATES_RAD_S:
        return capability_has(*_capability, CAP_ONBOARD_IMU);
    case FieldSlot::USB_MODE:
    case FieldSlot::USB_ACTIVE_MODE:
    case FieldSlot::USB_STATUS:
    case FieldSlot::USB_REBOOT_REQUIRED:
        return _serial_roles != nullptr &&
               recovery_usb_serial_index(*_capability) >= 0;
    case FieldSlot::RCIN_PROTOCOLS:
        return rcin_protocols_parameter() != nullptr;
    case FieldSlot::PWMIN_TIMEOUT_MS:
    case FieldSlot::PWMIN_MIN_US:
    case FieldSlot::PWMIN_TRIM_US:
    case FieldSlot::PWMIN_MAX_US:
    case FieldSlot::PWMIN_CONFIG_VALID:
        return _capability->pwm_inputs != 0;
    case FieldSlot::PWMOUT_RATE_HZ:
    case FieldSlot::PWMOUT_MIN_US:
    case FieldSlot::PWMOUT_TRIM_US:
    case FieldSlot::PWMOUT_MAX_US:
    case FieldSlot::PWMOUT_REVERSE_MASK:
    case FieldSlot::PWMOUT_FAILSAFE_US:
    case FieldSlot::PWMOUT_CONFIG_VALID:
    case FieldSlot::PWMOUT_EFFECTIVE_RATE_HZ:
    case FieldSlot::PWMOUT_EFFECTIVE_FAILSAFE_US:
        return _capability->pwm_outputs != 0;
    default:
        return true;
    }
}
