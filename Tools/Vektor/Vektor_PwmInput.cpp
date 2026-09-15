#include "Vektor_PwmInput.h"

#include "Config.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <math.h>

extern const AP_HAL::HAL &hal;

namespace Vektor {

const AP_Param::GroupInfo PwmInput::var_info[] = {
    // @Param: 1_PIN
    // @DisplayName: PWM input 1 pin
    // @Description: GPIO number used to capture PWM input channel 1. Set to -1 to disable. On revo-mini the PWM connector GPIO numbers are 50 through 55; a pin used as input cannot also be used as an output.
    // @Range: -1 255
    // @RebootRequired: True
    AP_GROUPINFO("1_PIN", 1, PwmInput, pins[0], -1),

    // @Param: 2_PIN
    // @CopyFieldsFrom: PWIN1_PIN
    AP_GROUPINFO("2_PIN", 2, PwmInput, pins[1], -1),

    // @Param: 3_PIN
    // @CopyFieldsFrom: PWIN1_PIN
    AP_GROUPINFO("3_PIN", 3, PwmInput, pins[2], -1),

    // @Param: 4_PIN
    // @CopyFieldsFrom: PWIN1_PIN
    AP_GROUPINFO("4_PIN", 4, PwmInput, pins[3], -1),

    // @Param: 5_PIN
    // @CopyFieldsFrom: PWIN1_PIN
    AP_GROUPINFO("5_PIN", 5, PwmInput, pins[4], -1),

    // @Param: 6_PIN
    // @CopyFieldsFrom: PWIN1_PIN
    AP_GROUPINFO("6_PIN", 6, PwmInput, pins[5], -1),

    // @Param: _TIMEOUT
    // @DisplayName: PWM input timeout
    // @Description: Time without a complete pulse before a PWM input becomes stale.
    // @Units: ms
    // @Range: 20 2000
    // @Increment: 10
    AP_GROUPINFO("_TIMEOUT", 7, PwmInput, timeout_ms, default_pwmin_timeout_ms),

    // @Param: _MIN
    // @DisplayName: PWM input minimum
    // @Description: Pulse width mapped to normalized -1.
    // @Units: PWM
    // @Range: 800 2200
    AP_GROUPINFO("_MIN", 8, PwmInput, pwm_min, default_pwm_min_us),

    // @Param: _TRIM
    // @DisplayName: PWM input trim
    // @Description: Pulse width mapped to normalized zero.
    // @Units: PWM
    // @Range: 800 2200
    AP_GROUPINFO("_TRIM", 9, PwmInput, pwm_trim, default_pwm_trim_us),

    // @Param: _MAX
    // @DisplayName: PWM input maximum
    // @Description: Pulse width mapped to normalized +1.
    // @Units: PWM
    // @Range: 800 2200
    AP_GROUPINFO("_MAX", 10, PwmInput, pwm_max, default_pwm_max_us),

    AP_GROUPEND
};

PwmInput::PwmInput()
{
    AP_Param::setup_object_defaults(this, var_info);
    reset();
}

void PwmInput::reset()
{
    for (uint8_t i = 0; i < max_channels; i++) {
        _channels[i] = {};
        _raw_pwm[i] = 0;
        _last_pulse_us[i] = 0;
        _have_pulse[i] = false;
        _value_valid[i] = false;
        _attached[i] = false;
        _attach_status[i] = AttachStatus::DISABLED;
        _capture_pin[i] = 0;
        _rise_us[i] = 0;
        _have_rise[i] = false;
        _captured_width_us[i] = 0;
        _captured_timestamp_us[i] = 0;
        _new_pulse[i] = false;
    }
    _reserved_output_mask = 0;
    _channel_count = 0;
}

void PwmInput::init(uint8_t channel_count)
{
    _channel_count = MIN(channel_count, max_channels);

    for (uint8_t channel_index = 0;
         channel_index < _channel_count;
         channel_index++) {
        const int16_t configured_pin = pins[channel_index].get();
        if (configured_pin < 0) {
            continue;
        }
        if (configured_pin > UINT8_MAX) {
            _attach_status[channel_index] = AttachStatus::INVALID_PIN;
            continue;
        }

        bool duplicate = false;
        for (uint8_t previous = 0; previous < channel_index; previous++) {
            duplicate |= pins[previous].get() == configured_pin &&
                         _capture_pin[previous] == configured_pin;
        }
        if (duplicate) {
            _attach_status[channel_index] = AttachStatus::DUPLICATE_PIN;
            continue;
        }

        _capture_pin[channel_index] = uint8_t(configured_pin);
        if (hal.gpio == nullptr) {
            _attach_status[channel_index] = AttachStatus::GPIO_UNAVAILABLE;
            continue;
        }
        if (!hal.gpio->valid_pin(uint8_t(configured_pin))) {
            _attach_status[channel_index] = AttachStatus::INVALID_PIN;
            continue;
        }

        uint8_t output_channel = 0;
        if (hal.gpio->pin_to_servo_channel(uint8_t(configured_pin),
                                          output_channel) &&
            output_channel < 32) {
            // Honor the configured input ownership even if IRQ attachment
            // fails, so this physical pin can never be driven as an output.
            _reserved_output_mask |= 1U << output_channel;
        }
        hal.gpio->pinMode(uint8_t(configured_pin), HAL_GPIO_INPUT);
        _attached[channel_index] = hal.gpio->attach_interrupt(
            uint8_t(configured_pin),
            FUNCTOR_BIND_MEMBER(&PwmInput::irq_handler,
                                void,
                                uint8_t,
                                bool,
                                uint32_t),
            AP_HAL::GPIO::INTERRUPT_BOTH);
        if (!_attached[channel_index]) {
            _attach_status[channel_index] = AttachStatus::ATTACH_FAILED;
            continue;
        }
        _attach_status[channel_index] = AttachStatus::ATTACHED;
    }
}

void PwmInput::update(uint64_t now_us)
{
    int16_t configured_timeout_ms = timeout_ms.get();
    if (configured_timeout_ms < pwmin_timeout_ms_min ||
        configured_timeout_ms > pwmin_timeout_ms_max) {
        configured_timeout_ms = default_pwmin_timeout_ms;
    }
    const uint64_t timeout_us = uint64_t(configured_timeout_ms) * 1000U;

    for (uint8_t i = 0; i < max_channels; i++) {
        uint32_t captured_width = 0;
        uint32_t captured_timestamp = 0;
        bool have_new_pulse = false;
        void *irq_state = hal.scheduler->disable_interrupts_save();
        if (_new_pulse[i]) {
            captured_width = _captured_width_us[i];
            captured_timestamp = _captured_timestamp_us[i];
            _new_pulse[i] = false;
            have_new_pulse = true;
        }
        hal.scheduler->restore_interrupts(irq_state);

        if (have_new_pulse) {
            ingest_pulse(i,
                         captured_width > UINT16_MAX ?
                         UINT16_MAX : uint16_t(captured_width),
                         extend_irq_timestamp(captured_timestamp, now_us));
        }

        if (!_have_pulse[i]) {
            _channels[i].quality = SignalQuality::INVALID;
        } else if (!_value_valid[i]) {
            _channels[i].quality = SignalQuality::INVALID;
        } else if (_last_pulse_us[i] > now_us) {
            _channels[i].quality = SignalQuality::INVALID;
        } else if (now_us - _last_pulse_us[i] > timeout_us) {
            _channels[i].quality = SignalQuality::STALE;
        } else {
            _channels[i].quality = SignalQuality::VALID;
        }
    }
}

void PwmInput::ingest_pulse(uint8_t channel_index,
                            uint16_t pulse_us,
                            uint64_t timestamp_us)
{
    if (channel_index >= max_channels) {
        return;
    }

    uint16_t minimum = default_pwm_min_us;
    uint16_t trim = default_pwm_trim_us;
    uint16_t maximum = default_pwm_max_us;
    const bool calibration_ok = calibration(minimum, trim, maximum);

    _raw_pwm[channel_index] = pulse_us;
    _last_pulse_us[channel_index] = timestamp_us;
    _have_pulse[channel_index] = true;
    _value_valid[channel_index] = calibration_ok &&
                                  pulse_us >= pwm_input_valid_min_us &&
                                  pulse_us <= pwm_input_valid_max_us;
    _channels[channel_index].timestamp_us = timestamp_us;
    if (_value_valid[channel_index]) {
        _channels[channel_index].value = normalize_pwm(pulse_us,
                                                       minimum,
                                                       trim,
                                                       maximum);
        _channels[channel_index].quality = SignalQuality::VALID;
    } else {
        _channels[channel_index].value = 0.0F;
        _channels[channel_index].quality = SignalQuality::INVALID;
    }
}

const SignalSample<float> *PwmInput::channel(uint8_t index) const
{
    return index < max_channels ? &_channels[index] : nullptr;
}

uint16_t PwmInput::raw_pwm(uint8_t index) const
{
    return index < max_channels ? _raw_pwm[index] : 0;
}

bool PwmInput::attached(uint8_t index) const
{
    return index < max_channels && _attached[index];
}

PwmInput::AttachStatus PwmInput::attach_status(uint8_t index) const
{
    return index < max_channels ? _attach_status[index] :
                                  AttachStatus::INVALID_PIN;
}

bool PwmInput::calibration_valid() const
{
    uint16_t minimum = 0;
    uint16_t trim = 0;
    uint16_t maximum = 0;
    return calibration(minimum, trim, maximum);
}

uint64_t PwmInput::extend_irq_timestamp(uint32_t timestamp_us,
                                        uint64_t now_us)
{
    return now_us - uint32_t(uint32_t(now_us) - timestamp_us);
}

float PwmInput::normalize_pwm(uint16_t pulse_us,
                              uint16_t minimum,
                              uint16_t trim,
                              uint16_t maximum)
{
    if (minimum >= trim || trim >= maximum) {
        minimum = default_pwm_min_us;
        trim = default_pwm_trim_us;
        maximum = default_pwm_max_us;
    }
    if (pulse_us < trim) {
        return constrain_float(float(int32_t(pulse_us) - trim) /
                               float(trim - minimum),
                               -1.0F,
                               0.0F);
    }
    return constrain_float(float(pulse_us - trim) /
                           float(maximum - trim),
                           0.0F,
                           1.0F);
}

void PwmInput::irq_handler(uint8_t pin,
                           bool pin_high,
                           uint32_t timestamp_us)
{
    for (uint8_t i = 0; i < max_channels; i++) {
        if (!_attached[i] || _capture_pin[i] != pin) {
            continue;
        }
        if (pin_high) {
            _rise_us[i] = timestamp_us;
            _have_rise[i] = true;
        } else if (_have_rise[i]) {
            _captured_width_us[i] = timestamp_us - _rise_us[i];
            _captured_timestamp_us[i] = timestamp_us;
            _have_rise[i] = false;
            _new_pulse[i] = true;
        }
        return;
    }
}

bool PwmInput::calibration(uint16_t &minimum,
                           uint16_t &trim,
                           uint16_t &maximum) const
{
    const int16_t configured_minimum = pwm_min.get();
    const int16_t configured_trim = pwm_trim.get();
    const int16_t configured_maximum = pwm_max.get();
    if (configured_minimum < pwm_calibration_min_us ||
        configured_maximum > pwm_calibration_max_us ||
        configured_minimum >= configured_trim ||
        configured_trim >= configured_maximum) {
        return false;
    }
    minimum = uint16_t(configured_minimum);
    trim = uint16_t(configured_trim);
    maximum = uint16_t(configured_maximum);
    return true;
}

} // namespace Vektor
