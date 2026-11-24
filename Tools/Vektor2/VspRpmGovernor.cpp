#include "VspRpmGovernor.h"
#include "LibVSP.h"

#include <algorithm>
#include <cmath>

namespace Vektor2 {

uint16_t VspRpmGovernor::update(uint32_t now_ms, const Settings& settings, uint16_t input_us,
                               bool input_routed, const Feedback& shaft)
{
    constexpr uint16_t stop_us = 1480;
    constexpr uint16_t low_us = 1500;
    constexpr uint32_t start_hold_ms = 10000;
    const bool input_valid = input_routed && input_us >= VSP_PWM_MIN && input_us <= VSP_PWM_MAX;
    const bool requested = input_valid && input_us > stop_us;
    const bool healthy_sensor = shaft.available && std::isfinite(shaft.rpm) && shaft.rpm >= 0;
    const bool feedback_valid = healthy_sensor && shaft.rpm > 0;
    const bool configured = std::isfinite(settings.low_rpm) && settings.low_rpm > 0 &&
                            std::isfinite(settings.high_rpm) && settings.high_rpm >= settings.low_rpm &&
                            std::isfinite(settings.deadband_rpm) && settings.deadband_rpm >= 0 &&
                            std::isfinite(settings.near_rpm) && settings.near_rpm > settings.deadband_rpm &&
                            std::isfinite(settings.fast_us_per_s) && settings.fast_us_per_s <= 100 &&
                            std::isfinite(settings.slow_us_per_s) && settings.slow_us_per_s >= 1 &&
                            settings.fast_us_per_s >= settings.slow_us_per_s &&
                            settings.start_max_us > VSP_PWM_MIN && settings.start_max_us <= low_us;
    _status = (settings.enabled ? Enabled : 0) | (input_valid ? ControlValid | ThrottleValid : 0) |
              (requested ? Requested : 0) | (feedback_valid ? FeedbackValid : 0) |
              (configured ? ConfigValid : 0);

    if (!settings.enabled) {
        _state = State::Stopped;
        _throttle_us = input_valid ? input_us : VSP_PWM_MIN;
        return input_us;
    }
    if (!configured || (input_valid && !requested)) {
        _state = State::Stopped;
        _throttle_us = VSP_PWM_MIN;
        return VSP_PWM_MIN;
    }
    if (!input_valid || !healthy_sensor || _state == State::WaitForOff ||
        (_state == State::Regulating && !feedback_valid)) {
        if (requested || _state != State::Stopped) {
            _state = State::WaitForOff;
            _status |= Inhibited;
        }
        return VSP_PWM_MIN;
    }

    if (_state == State::Stopped) {
        _state = State::Acquiring;
        _throttle_us = feedback_valid ? std::max(_throttle_us, float(VSP_PWM_MIN)) : float(VSP_PWM_MIN);
        _last_ms = now_ms;
        _last_below_start_limit_ms = now_ms;
    }
    const float dt = std::min(now_ms - _last_ms, uint32_t(100)) * 0.001f;
    _last_ms = now_ms;

    if (feedback_valid) {
        _state = State::Regulating;
        _status |= Active;
        const float fraction = float(std::max(input_us, low_us) - low_us) / (VSP_PWM_MAX - low_us);
        const float target_rpm = input_us == VSP_PWM_MAX ? settings.high_rpm :
                                 settings.low_rpm + fraction * (settings.high_rpm - settings.low_rpm);
        const float error = target_rpm - shaft.rpm;
        const float distance = std::fabs(error);
        if (distance <= settings.deadband_rpm) {
            _status |= InDeadband;
        } else {
            float rate;
            if (distance <= settings.near_rpm) {
                rate = settings.slow_us_per_s * (distance - settings.deadband_rpm) /
                       (settings.near_rpm - settings.deadband_rpm);
            } else {
                const float blend = std::min((distance - settings.near_rpm) / settings.near_rpm, 1.0f);
                rate = settings.slow_us_per_s + blend * (settings.fast_us_per_s - settings.slow_us_per_s);
            }
            const float pulse = _throttle_us + std::copysign(rate * dt, error);
            _throttle_us = std::max(float(VSP_PWM_MIN), std::min(pulse, float(VSP_PWM_MAX)));
        }
    } else {
        // A stationary shaft needs motion before the sensor can provide RPM.
        // Bound the open-loop command until the first measured revolution.
        if (_throttle_us < settings.start_max_us) {
            _last_below_start_limit_ms = now_ms;
        } else if (now_ms - _last_below_start_limit_ms >= start_hold_ms) {
            _state = State::WaitForOff;
            _status |= Inhibited;
            return VSP_PWM_MIN;
        }
        _throttle_us = std::min(_throttle_us + settings.fast_us_per_s * dt, float(settings.start_max_us));
        _status |= Starting;
    }

    const uint16_t output = uint16_t(_throttle_us + 0.5f);
    if (_state == State::Regulating && (output == VSP_PWM_MIN || output == VSP_PWM_MAX)) {
        _status |= AtThrottleLimit;
    }
    return output;
}

} // namespace Vektor2
