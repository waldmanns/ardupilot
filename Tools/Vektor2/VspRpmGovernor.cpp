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
                            settings.start_us > VSP_PWM_MIN && settings.start_us <= low_us &&
                            std::isfinite(settings.start_timeout_s) &&
                            settings.start_timeout_s >= 0.1f && settings.start_timeout_s <= 60.0f;
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
        if (feedback_valid) {
            _throttle_us = std::max(_throttle_us, float(VSP_PWM_MIN));
        } else {
            _state = State::Acquiring;
            _throttle_us = settings.start_us;
            _start_ms = now_ms;
        }
    }
    const uint32_t timeout_ms = uint32_t(settings.start_timeout_s * 1000.0f + 0.5f);
    if (_state == State::Acquiring && now_ms - _start_ms >= timeout_ms) {
        _state = State::WaitForOff;
        _status |= Inhibited;
        return VSP_PWM_MIN;
    }
    if (!feedback_valid) {
        _status |= Starting;
        return uint16_t(_throttle_us);
    }

    // The first feedback sample takes over from the startup pulse without a step.
    const float dt = _state == State::Regulating ? std::min(now_ms - _last_ms, uint32_t(100)) * 0.001f : 0;
    _last_ms = now_ms;
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
    const uint16_t output = uint16_t(_throttle_us + 0.5f);
    if (output == VSP_PWM_MIN || output == VSP_PWM_MAX) {
        _status |= AtThrottleLimit;
    }
    return output;
}

} // namespace Vektor2
