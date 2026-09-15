// Parameter decoding, staged validation, and application.
// Included inside namespace Vektor.

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

    if (field->slot == FieldSlot::PWMOUT_RATE_HZ &&
        !PwmOutput::supported_rate(uint16_t(value))) {
        code = Protocol::ErrorCode::OUT_OF_RANGE;
        detail = "unsupported PWM rate";
        return false;
    }

    if (field->slot == FieldSlot::PWMOUT_REVERSE_MASK &&
        _capability != nullptr) {
        const uint32_t available_mask = _capability->pwm_outputs >= 16 ?
            UINT16_MAX : (1U << _capability->pwm_outputs) - 1U;
        if ((uint32_t(value) & ~available_mask) != 0) {
            code = Protocol::ErrorCode::OUT_OF_RANGE;
            detail = "reverse mask selects unavailable output";
            return false;
        }
    }

    uint8_t serial_index = 0;
    if (serial_role_for_slot(field->slot, serial_index)) {
        const SerialRole role = SerialRole(value);
        if (_capability == nullptr || _serial_roles == nullptr ||
            (serial_endpoint_supported_role_mask(*_capability, serial_index) &
             (1U << uint8_t(role))) == 0) {
            code = Protocol::ErrorCode::OUT_OF_RANGE;
            detail = "role unsupported by serial endpoint";
            return false;
        }
    }

    code = Protocol::ErrorCode::UNKNOWN_MESSAGE;
    detail = "";
    return true;
}

bool SerialProtocol::validate_parameter_set(const ParameterUpdate *updates,
                                            uint16_t count,
                                            Protocol::ErrorCode &code,
                                            uint32_t &failing_field_id,
                                            const char *&detail) const
{
    if (updates == nullptr || count == 0) {
        code = Protocol::ErrorCode::BAD_LENGTH;
        failing_field_id = 0;
        detail = "empty parameter set";
        return false;
    }

    int32_t input_min = _pwm_input == nullptr ? 0 : _pwm_input->pwm_min.get();
    int32_t input_trim = _pwm_input == nullptr ? 0 : _pwm_input->pwm_trim.get();
    int32_t input_max = _pwm_input == nullptr ? 0 : _pwm_input->pwm_max.get();
    int32_t output_min = _pwm_output == nullptr ? 0 : _pwm_output->pwm_min.get();
    int32_t output_trim = _pwm_output == nullptr ? 0 : _pwm_output->pwm_trim.get();
    int32_t output_max = _pwm_output == nullptr ? 0 : _pwm_output->pwm_max.get();
    int32_t output_failsafe = _pwm_output == nullptr ? 0 :
                                                       _pwm_output->failsafe_pwm.get();
    bool input_calibration_changed = false;
    bool output_calibration_changed = false;
    uint32_t input_calibration_field_id = 0;
    uint32_t output_calibration_field_id = 0;
    int16_t serial_roles[SerialRoleParameters::max_endpoints] {};
    bool serial_roles_changed = false;
    if (_serial_roles != nullptr) {
        for (uint8_t i = 0; i < SerialRoleParameters::max_endpoints; i++) {
            serial_roles[i] = int16_t(_serial_roles->configured_role(i));
        }
    }

    for (uint16_t i = 0; i < count; i++) {
        const FieldDescriptor *field =
            schema_registry().field_by_id(updates[i].field_id);
        if (field == nullptr) {
            code = Protocol::ErrorCode::BAD_ID;
            failing_field_id = updates[i].field_id;
            detail = "unknown field";
            return false;
        }
        const int32_t value = signed_raw_value(field->type, updates[i].raw);
        uint8_t serial_index = 0;
        if (serial_role_for_slot(field->slot, serial_index)) {
            serial_roles[serial_index] = int16_t(value);
            serial_roles_changed = true;
        }
        switch (field->slot) {
        case FieldSlot::PWMIN_MIN_US:
            input_min = value;
            input_calibration_changed = true;
            input_calibration_field_id = updates[i].field_id;
            break;
        case FieldSlot::PWMIN_TRIM_US:
            input_trim = value;
            input_calibration_changed = true;
            input_calibration_field_id = updates[i].field_id;
            break;
        case FieldSlot::PWMIN_MAX_US:
            input_max = value;
            input_calibration_changed = true;
            input_calibration_field_id = updates[i].field_id;
            break;
        case FieldSlot::PWMOUT_MIN_US:
            output_min = value;
            output_calibration_changed = true;
            output_calibration_field_id = updates[i].field_id;
            break;
        case FieldSlot::PWMOUT_TRIM_US:
            output_trim = value;
            output_calibration_changed = true;
            output_calibration_field_id = updates[i].field_id;
            break;
        case FieldSlot::PWMOUT_MAX_US:
            output_max = value;
            output_calibration_changed = true;
            output_calibration_field_id = updates[i].field_id;
            break;
        case FieldSlot::PWMOUT_FAILSAFE_US:
            output_failsafe = value;
            output_calibration_changed = true;
            output_calibration_field_id = updates[i].field_id;
            break;
        default:
            break;
        }
    }

    if (serial_roles_changed) {
        SerialEndpointStatus statuses[SerialRoleParameters::max_endpoints] {};
        if (_capability == nullptr ||
            !validate_serial_roles(*_capability, serial_roles, statuses)) {
            code = Protocol::ErrorCode::INVALID_STATE;
            for (uint16_t i = 0; i < count; i++) {
                const FieldDescriptor *field =
                    schema_registry().field_by_id(updates[i].field_id);
                uint8_t serial_index = 0;
                if (field != nullptr &&
                    serial_role_for_slot(field->slot, serial_index) &&
                    statuses[serial_index] != SerialEndpointStatus::OK &&
                    statuses[serial_index] !=
                        SerialEndpointStatus::RESERVED_NOT_IMPLEMENTED) {
                    failing_field_id = updates[i].field_id;
                    break;
                }
            }
            detail = "duplicate or invalid serial role assignment";
            return false;
        }
    }

    if (input_calibration_changed &&
        !(input_min < input_trim && input_trim < input_max)) {
        code = Protocol::ErrorCode::OUT_OF_RANGE;
        failing_field_id = input_calibration_field_id;
        detail = "invalid staged PWM input calibration";
        return false;
    }
    if (output_calibration_changed &&
        (!(output_min < output_trim && output_trim < output_max) ||
         (output_failsafe != 0 &&
          (output_failsafe < output_min || output_failsafe > output_max)))) {
        code = Protocol::ErrorCode::OUT_OF_RANGE;
        failing_field_id = output_calibration_field_id;
        detail = "invalid staged PWM output calibration or failsafe";
        return false;
    }

    code = Protocol::ErrorCode::UNKNOWN_MESSAGE;
    failing_field_id = 0;
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

    uint8_t channel_index = 0;
    if (pwmin_pin_for_slot(field->slot, channel_index)) {
        if (_pwm_input == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_input->pins[channel_index],
                                   int16_t(uint16_t(raw)));
        return true;
    }
    if (serial_role_for_slot(field->slot, channel_index)) {
        if (_serial_roles == nullptr) {
            return false;
        }
        _serial_roles->set_configured_role(channel_index,
                                           SerialRole(int32_t(raw)));
        return true;
    }
    if (serial_baud_for_slot(field->slot, channel_index)) {
        if (_serial_roles == nullptr) {
            return false;
        }
        _serial_roles->set_configured_baud(channel_index, raw);
        return true;
    }

    switch (field->slot) {
    case FieldSlot::SYS_OPTIONS:
        save_parameter_without_gcs(_parameters->sys_options, int32_t(raw));
        return true;
    case FieldSlot::SYS_DESC_PAGE:
        save_parameter_without_gcs(_parameters->sys_desc_page,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::USB_MODE:
        if (_serial_roles == nullptr) {
            return false;
        }
        _serial_roles->set_configured_usb_mode(UsbMode(int32_t(raw)));
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
    case FieldSlot::PWMIN_TIMEOUT_MS:
        if (_pwm_input == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_input->timeout_ms,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::PWMIN_MIN_US:
        if (_pwm_input == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_input->pwm_min,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::PWMIN_TRIM_US:
        if (_pwm_input == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_input->pwm_trim,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::PWMIN_MAX_US:
        if (_pwm_input == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_input->pwm_max,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::PWMOUT_RATE_HZ:
        if (_pwm_output == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_output->rate_hz,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::PWMOUT_MIN_US:
        if (_pwm_output == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_output->pwm_min,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::PWMOUT_TRIM_US:
        if (_pwm_output == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_output->pwm_trim,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::PWMOUT_MAX_US:
        if (_pwm_output == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_output->pwm_max,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::PWMOUT_REVERSE_MASK:
        if (_pwm_output == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_output->reverse_mask,
                                   int16_t(uint16_t(raw)));
        return true;
    case FieldSlot::PWMOUT_FAILSAFE_US:
        if (_pwm_output == nullptr) {
            return false;
        }
        save_parameter_without_gcs(_pwm_output->failsafe_pwm,
                                   int16_t(uint16_t(raw)));
        return true;
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
    case FieldSlot::ATTITUDE_ROLL_DEG:
    case FieldSlot::ATTITUDE_PITCH_DEG:
    case FieldSlot::ATTITUDE_YAW_DEG:
    case FieldSlot::ATTITUDE_QUATERNION:
    case FieldSlot::ATTITUDE_BODY_RATES_RAD_S:
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
    case FieldSlot::PWMIN_CONFIG_VALID:
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
    case FieldSlot::PWMOUT_CONFIG_VALID:
    case FieldSlot::PWMOUT_EFFECTIVE_RATE_HZ:
    case FieldSlot::PWMOUT_EFFECTIVE_FAILSAFE_US:
        break;
    default:
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
    case Protocol::PrimitiveType::ENUM:
        return reader.u32(raw);
    default:
        return false;
    }
}
