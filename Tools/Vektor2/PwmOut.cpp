#include "PwmOut.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

namespace Vektor2 {

void PwmOut::init()
{
    if (hal.rcout == nullptr) {
        return;
    }
    hal.rcout->init();
}

bool PwmOut::set_rate_hz(uint8_t channel, uint16_t rate_hz)
{
    if (hal.rcout == nullptr || channel >= max_pwm_channels || rate_hz == 0) {
        return false;
    }

    const uint32_t mask = 1UL << channel;
    hal.rcout->set_freq(mask, rate_hz);
    return true;
}

bool PwmOut::write_us(uint8_t channel, uint16_t pulse_us)
{
    if (hal.rcout == nullptr || channel >= max_pwm_channels) {
        return false;
    }

    const uint32_t mask = 1UL << channel;
    if ((_enabled_mask & mask) == 0) {
        hal.rcout->enable_ch(channel);
        hal.rcout->set_freq(mask, default_pwm_rate_hz);
        _enabled_mask |= mask;
    }

    hal.rcout->write(channel, pulse_us);
    _values_us[channel] = pulse_us;
    return true;
}

void PwmOut::disable(uint8_t channel)
{
    if (hal.rcout == nullptr || channel >= max_pwm_channels) {
        return;
    }

    hal.rcout->disable_ch(channel);
    _enabled_mask &= ~(1UL << channel);
    _values_us[channel] = 0;
}

void PwmOut::disable_all()
{
    for (uint8_t channel = 0; channel < max_pwm_channels; channel++) {
        disable(channel);
    }
}

} // namespace Vektor2
