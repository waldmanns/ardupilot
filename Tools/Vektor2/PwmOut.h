#pragma once

#include "Config.h"

#include <stdint.h>

namespace Vektor2 {

// Intentionally tiny PWM facade. No servo-function layer and no arming state
// machine. The last values written are retained only for standard MAVLink
// SERVO_OUTPUT_RAW telemetry.
class PwmOut {
public:
    void init();

    bool write_us(uint8_t channel, uint16_t pulse_us);
    bool set_rate_hz(uint8_t channel, uint16_t rate_hz);
    void disable(uint8_t channel);
    void disable_all();

    bool enabled(uint8_t channel) const
    {
        return channel < max_pwm_channels &&
               (_enabled_mask & (1UL << channel)) != 0;
    }

    uint16_t value_us(uint8_t channel) const
    {
        return channel < max_pwm_channels ? _values_us[channel] : 0;
    }

    uint32_t enabled_mask() const { return _enabled_mask; }

private:
    uint16_t _values_us[max_pwm_channels] {};
    uint32_t _enabled_mask = 0;
};

} // namespace Vektor2
