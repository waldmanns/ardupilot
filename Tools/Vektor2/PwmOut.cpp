#include "PwmOut.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

namespace Vektor2 {

const AP_Param::GroupInfo PwmChannelParameters::var_info[] = {
    // @Param: MIN
    // @DisplayName: PWM output minimum
    // @Description: Pulse width produced for a routed value of 1000 us.
    // @Units: us
    // @Range: 500 2500
    // @User: Standard
    AP_GROUPINFO("MIN", 1, PwmChannelParameters, min_us, 1000),

    // @Param: MAX
    // @DisplayName: PWM output maximum
    // @Description: Pulse width produced for a routed value of 2000 us.
    // @Units: us
    // @Range: 500 2500
    // @User: Standard
    AP_GROUPINFO("MAX", 2, PwmChannelParameters, max_us, 2000),

    // @Param: INV
    // @DisplayName: Invert PWM output
    // @Description: Reverse the mapping between the routed value and PWM output.
    // @Values: 0:Normal,1:Inverted
    // @User: Standard
    AP_GROUPINFO("INV", 3, PwmChannelParameters, inverted, 0),

    AP_GROUPEND
};

#define PWM_CHANNEL(n) AP_SUBGROUPINFO(channels[(n)-1], #n "_", n, PwmOut, PwmChannelParameters)
const AP_Param::GroupInfo PwmOut::var_info[] = {
    PWM_CHANNEL(1), PWM_CHANNEL(2), PWM_CHANNEL(3), PWM_CHANNEL(4),
    PWM_CHANNEL(5), PWM_CHANNEL(6), PWM_CHANNEL(7), PWM_CHANNEL(8),
    PWM_CHANNEL(9), PWM_CHANNEL(10), PWM_CHANNEL(11), PWM_CHANNEL(12),
    PWM_CHANNEL(13), PWM_CHANNEL(14), PWM_CHANNEL(15), PWM_CHANNEL(16),
    PWM_CHANNEL(17), PWM_CHANNEL(18), PWM_CHANNEL(19), PWM_CHANNEL(20),
    PWM_CHANNEL(21), PWM_CHANNEL(22), PWM_CHANNEL(23), PWM_CHANNEL(24),
    PWM_CHANNEL(25), PWM_CHANNEL(26), PWM_CHANNEL(27), PWM_CHANNEL(28),
    PWM_CHANNEL(29), PWM_CHANNEL(30), PWM_CHANNEL(31), PWM_CHANNEL(32),
    AP_GROUPEND
};
#undef PWM_CHANNEL

void PwmOut::init()
{
    if (hal.rcout == nullptr) {
        return;
    }
    hal.rcout->init();
    // Vektor2 has no safety switch or arming state machine. ChibiOS RCOutput
    // starts with its safety gate engaged and otherwise suppresses every PWM
    // pulse even though write() accepts and reports the requested value.
    hal.rcout->force_safety_off();
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

    const PwmChannelParameters& params = channels[channel];
    const int16_t min_us = params.min_us.get();
    const int16_t max_us = params.max_us.get();
    const int8_t inverted = params.inverted.get();
    if (min_us < 500 || max_us > 2500 || min_us >= max_us ||
        (inverted != 0 && inverted != 1)) {
        disable(channel);
        return false;
    }

    // Routed signals use 1000..2000 us. Clamp before scaling so both
    // endpoints and the reversed mapping remain within the configured range.
    const uint16_t clamped_us = pulse_us < 1000 ? 1000 :
                                pulse_us > 2000 ? 2000 : pulse_us;
    const uint32_t scaled = (uint32_t(clamped_us - 1000) *
                             uint32_t(max_us - min_us) + 500U) / 1000U;
    const uint16_t output_us = inverted != 0 ? uint16_t(max_us - scaled) :
                                                uint16_t(min_us + scaled);

    const uint32_t mask = 1UL << channel;
    if ((_enabled_mask & mask) == 0) {
        hal.rcout->enable_ch(channel);
        hal.rcout->set_freq(mask, default_pwm_rate_hz);
        _enabled_mask |= mask;
    }

    hal.rcout->write(channel, output_us);
    _values_us[channel] = output_us;
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
