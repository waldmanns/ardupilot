#pragma once

#include "Vektor_Signal.h"

#include <AP_Param/AP_Param.h>

#include <stdint.h>

namespace Vektor {

class PwmInput {
public:
    static constexpr uint8_t max_channels = 6;
    static constexpr uint32_t default_timeout_us = 100000;

    PwmInput();

    void reset();
    void init();
    void update(uint64_t now_us);
    void ingest_pulse(uint8_t channel, uint16_t pulse_us, uint64_t timestamp_us);

    const SignalSample<float> *channel(uint8_t index) const;
    uint16_t raw_pwm(uint8_t index) const;
    bool attached(uint8_t index) const;
    uint32_t reserved_output_mask() const { return _reserved_output_mask; }

    static float normalize_pwm(uint16_t pulse_us,
                               uint16_t minimum,
                               uint16_t trim,
                               uint16_t maximum);
    static const AP_Param::GroupInfo var_info[];

    AP_Int16 pins[max_channels];
    AP_Int16 timeout_ms;
    AP_Int16 pwm_min;
    AP_Int16 pwm_trim;
    AP_Int16 pwm_max;

private:
    void irq_handler(uint8_t pin, bool pin_high, uint32_t timestamp_us);
    bool calibration(uint16_t &minimum,
                     uint16_t &trim,
                     uint16_t &maximum) const;

    SignalSample<float> _channels[max_channels] {};
    uint16_t _raw_pwm[max_channels] {};
    uint64_t _last_pulse_us[max_channels] {};
    bool _have_pulse[max_channels] {};
    bool _value_valid[max_channels] {};
    bool _attached[max_channels] {};
    uint8_t _capture_pin[max_channels] {};
    volatile uint32_t _rise_us[max_channels] {};
    volatile uint32_t _captured_width_us[max_channels] {};
    volatile bool _new_pulse[max_channels] {};
    uint32_t _reserved_output_mask = 0;
};

} // namespace Vektor
