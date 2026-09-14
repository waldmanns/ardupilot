#include "Vektor_PwmOutput.h"

#include "Config.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <math.h>

extern const AP_HAL::HAL &hal;

namespace Vektor {

const AP_Param::GroupInfo PwmOutput::var_info[] = {
    // @Param: _RATE
    // @DisplayName: PWM output rate
    // @Description: Frame rate shared by the routed PWM output bank. Supported values are 50, 100, 200, and 330 Hz.
    // @Units: Hz
    // @Values: 50:50 Hz,100:100 Hz,200:200 Hz,330:330 Hz
    AP_GROUPINFO("_RATE", 1, PwmOutput, rate_hz, default_pwm_rate_hz),

    // @Param: _MIN
    // @DisplayName: PWM output minimum
    // @Description: Pulse width generated for a normalized command of -1.
    // @Units: PWM
    // @Range: 800 2200
    AP_GROUPINFO("_MIN", 2, PwmOutput, pwm_min, default_pwm_min_us),

    // @Param: _TRIM
    // @DisplayName: PWM output trim
    // @Description: Pulse width generated for a normalized command of zero.
    // @Units: PWM
    // @Range: 800 2200
    AP_GROUPINFO("_TRIM", 3, PwmOutput, pwm_trim, default_pwm_trim_us),

    // @Param: _MAX
    // @DisplayName: PWM output maximum
    // @Description: Pulse width generated for a normalized command of +1.
    // @Units: PWM
    // @Range: 800 2200
    AP_GROUPINFO("_MAX", 4, PwmOutput, pwm_max, default_pwm_max_us),

    // @Param: _REVERSE
    // @DisplayName: PWM output reverse mask
    // @Description: Bitmask selecting output channels whose normalized command is reversed.
    // @Bitmask: 0:Output1,1:Output2,2:Output3,3:Output4,4:Output5,5:Output6,6:Output7,7:Output8,8:Output9,9:Output10,10:Output11,11:Output12
    AP_GROUPINFO("_REVERSE", 5, PwmOutput, reverse_mask, 0),

    // @Param: _FAILSAFE
    // @DisplayName: PWM output failsafe pulse
    // @Description: Pulse sent when an assigned source is stale or invalid. Zero disables the affected output instead.
    // @Units: PWM
    // @Range: 0 2200
    AP_GROUPINFO("_FAILSAFE", 6, PwmOutput, failsafe_pwm, 0),

    AP_GROUPEND
};

PwmOutput::PwmOutput()
{
    AP_Param::setup_object_defaults(this, var_info);
    reset();
}

void PwmOutput::reset()
{
    for (uint8_t i = 0; i < max_channels; i++) {
        _commands[i] = {};
        _pwm_us[i] = 0;
        _active[i] = false;
    }
    _reserved_channel_mask = 0;
    _channel_count = 0;
    _applied_rate_hz = 0;
    _initialized = false;
}

void PwmOutput::init(uint8_t channel_count,
                     uint32_t reserved_channel_mask)
{
    _channel_count = MIN(channel_count, max_channels);
    _reserved_channel_mask = reserved_channel_mask;
    if (hal.rcout == nullptr) {
        return;
    }

    hal.rcout->init();
    for (uint8_t i = 0; i < _channel_count; i++) {
        hal.rcout->disable_ch(i);
    }

    // Vektor has no vehicle arming layer. Route validity and signal quality
    // gate every channel, so release the HAL safety gate and keep unrouted or
    // unhealthy channels disabled locally.
    hal.rcout->force_safety_off();
    _initialized = true;
    apply_rate();
}

void PwmOutput::update(const SignalSample<float> *commands,
                       uint8_t command_count,
                       uint32_t assigned_channel_mask)
{
    if (!_initialized || hal.rcout == nullptr) {
        return;
    }

    apply_rate();

    uint16_t minimum = default_pwm_min_us;
    uint16_t trim = default_pwm_trim_us;
    uint16_t maximum = default_pwm_max_us;
    (void)calibration(minimum, trim, maximum);
    const uint16_t configured_failsafe =
        failsafe_pwm.get() <= 0 ? 0 :
        constrain_int16(failsafe_pwm.get(), minimum, maximum);
    const uint16_t reversed = uint16_t(reverse_mask.get());

    hal.rcout->cork();
    for (uint8_t i = 0; i < _channel_count; i++) {
        const SignalSample<float> sample =
            commands != nullptr && i < command_count ? commands[i] :
                                                    SignalSample<float> {};
        _commands[i] = sample;
        const bool reserved = (_reserved_channel_mask & (1U << i)) != 0;
        const bool assigned = (assigned_channel_mask & (1U << i)) != 0;
        const bool valid = !reserved &&
                           assigned &&
                           sample.quality == SignalQuality::VALID &&
                           isfinite(sample.value);
        if (valid) {
            _pwm_us[i] = normalized_to_pwm(sample.value,
                                           minimum,
                                           trim,
                                           maximum,
                                           (reversed & (1U << i)) != 0);
            _active[i] = true;
            hal.rcout->enable_ch(i);
            hal.rcout->write(i, _pwm_us[i]);
        } else if (!reserved && assigned && configured_failsafe != 0) {
            _pwm_us[i] = configured_failsafe;
            _active[i] = true;
            hal.rcout->enable_ch(i);
            hal.rcout->write(i, _pwm_us[i]);
        } else {
            _pwm_us[i] = 0;
            _active[i] = false;
            hal.rcout->disable_ch(i);
        }
    }
    hal.rcout->push();

    for (uint8_t i = _channel_count; i < max_channels; i++) {
        _commands[i] = {};
        _pwm_us[i] = 0;
        _active[i] = false;
    }
}

void PwmOutput::set_reserved_channel_mask(uint32_t reserved_channel_mask)
{
    _reserved_channel_mask = reserved_channel_mask;
    _applied_rate_hz = 0;
    if (!_initialized || hal.rcout == nullptr) {
        return;
    }
    for (uint8_t i = 0; i < _channel_count; i++) {
        if ((_reserved_channel_mask & (1U << i)) != 0) {
            _active[i] = false;
            _pwm_us[i] = 0;
            hal.rcout->disable_ch(i);
        }
    }
    apply_rate();
}

const SignalSample<float> *PwmOutput::command(uint8_t index) const
{
    return index < max_channels ? &_commands[index] : nullptr;
}

uint16_t PwmOutput::pwm_us(uint8_t index) const
{
    return index < max_channels ? _pwm_us[index] : 0;
}

bool PwmOutput::active(uint8_t index) const
{
    return index < max_channels && _active[index];
}

bool PwmOutput::supported_rate(uint16_t configured_rate_hz)
{
    return configured_rate_hz == 50 ||
           configured_rate_hz == 100 ||
           configured_rate_hz == 200 ||
           configured_rate_hz == 330;
}

uint16_t PwmOutput::normalized_to_pwm(float value,
                                      uint16_t minimum,
                                      uint16_t trim,
                                      uint16_t maximum,
                                      bool reversed)
{
    if (minimum >= trim || trim >= maximum || !isfinite(value)) {
        return trim;
    }
    value = constrain_float(value, -1.0F, 1.0F);
    if (reversed) {
        value = -value;
    }
    const float pulse = value < 0.0F ?
        float(trim) + value * float(trim - minimum) :
        float(trim) + value * float(maximum - trim);
    return uint16_t(pulse + 0.5F);
}

bool PwmOutput::calibration(uint16_t &minimum,
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

uint16_t PwmOutput::configured_rate() const
{
    const uint16_t configured = uint16_t(rate_hz.get());
    return supported_rate(configured) ? configured : default_pwm_rate_hz;
}

void PwmOutput::apply_rate()
{
    const uint16_t rate = configured_rate();
    if (!_initialized || rate == _applied_rate_hz || _channel_count == 0) {
        return;
    }
    const uint32_t channel_mask =
        ((1U << _channel_count) - 1U) & ~_reserved_channel_mask;
    if (channel_mask != 0) {
        hal.rcout->set_freq(channel_mask, rate);
    }
    _applied_rate_hz = rate;
}

} // namespace Vektor
