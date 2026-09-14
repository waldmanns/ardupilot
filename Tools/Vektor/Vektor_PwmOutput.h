#pragma once

#include "Vektor_Signal.h"

#include <AP_Param/AP_Param.h>

#include <stdint.h>

namespace Vektor {

class PwmOutput {
public:
    static constexpr uint8_t max_channels = 12;

    PwmOutput();

    void reset();
    void init(uint8_t channel_count, uint32_t reserved_channel_mask);
    void set_reserved_channel_mask(uint32_t reserved_channel_mask);
    void update(const SignalSample<float> *commands,
                uint8_t command_count,
                uint32_t assigned_channel_mask);

    const SignalSample<float> *command(uint8_t index) const;
    uint16_t pwm_us(uint8_t index) const;
    bool active(uint8_t index) const;
    uint8_t channel_count() const { return _channel_count; }
    uint16_t effective_rate_hz() const { return configured_rate(); }

    static bool supported_rate(uint16_t rate_hz);
    static uint16_t normalized_to_pwm(float value,
                                      uint16_t minimum,
                                      uint16_t trim,
                                      uint16_t maximum,
                                      bool reversed);
    static const AP_Param::GroupInfo var_info[];

    AP_Int16 rate_hz;
    AP_Int16 pwm_min;
    AP_Int16 pwm_trim;
    AP_Int16 pwm_max;
    AP_Int16 reverse_mask;
    AP_Int16 failsafe_pwm;

private:
    bool calibration(uint16_t &minimum,
                     uint16_t &trim,
                     uint16_t &maximum) const;
    uint16_t configured_rate() const;
    void apply_rate();

    SignalSample<float> _commands[max_channels] {};
    uint16_t _pwm_us[max_channels] {};
    bool _active[max_channels] {};
    uint32_t _reserved_channel_mask = 0;
    uint8_t _channel_count = 0;
    uint16_t _applied_rate_hz = 0;
    bool _initialized = false;
};

} // namespace Vektor
