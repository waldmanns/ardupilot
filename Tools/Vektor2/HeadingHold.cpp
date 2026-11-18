#include "HeadingHold.h"

#include <math.h>
#include <string.h>

namespace Vektor2 {
namespace {
float bounded(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

bool in_range(float value, float low, float high)
{
    return isfinite(value) && value >= low && value <= high;
}

bool different(float a, float b)
{
    return memcmp(&a, &b, sizeof(a)) != 0;
}

float wrap180(float angle)
{
    angle = fmodf(angle + 180, 360);
    return (angle < 0 ? angle + 360 : angle) - 180;
}

} // namespace

const char* HeadingHold::inhibit_reason() const
{
    // One short primary reason; the numeric mask retains every concurrent bit.
    // Hard faults take priority over normal manual/wait/limiting conditions.
    static const struct {
        uint16_t flag;
        const char* text;
    } reasons[] = {
        {Configuration, "Configuration invalid"},
        {Routing, "VSP cross-feed"},
        {Compass, "Compass invalid"},
        {Attitude, "Attitude invalid"},
        {Input, "Control input lost"},
        {Timing, "Loop gap >100ms"},
        {YawReset, "EKF yaw reset"},
        {Stick, "Manual override"},
        {Delay, "Waiting for idle"},
        {Travel, "Travel limited"},
        {Slew, "Slew limited"},
    };
    for (const auto& reason : reasons) {
        if (_inhibited & reason.flag) {
            return reason.text;
        }
    }
    return _state == State::Disabled ? "Disabled" : "OK";
}

bool HeadingHold::valid(const HeadingSettings& s)
{
    return (s.vsp_enabled == 0 || s.vsp_enabled == 1) && s.stick_deadband >= 0 && s.stick_deadband <= 250 &&
           s.idle_ms >= 0 && s.idle_ms <= 10000 &&
           in_range(s.p, 0, 10) && in_range(s.i, 0, 10) && in_range(s.d, 0, 10) &&
           in_range(s.imax, 0, 1) && in_range(s.heading_deadband, 0, 30) &&
           in_range(s.maximum, 0, 1) && in_range(s.slew, 0.01f, 100) &&
           in_range(s.filter_hz, 0.1f, 20);
}

bool HeadingHold::changed(const HeadingSettings& s) const
{
    if (s.enabled != _settings.enabled || s.vsp_enabled != _settings.vsp_enabled || s.stick_deadband != _settings.stick_deadband ||
        s.idle_ms != _settings.idle_ms || different(s.p, _settings.p) ||
        different(s.i, _settings.i) || different(s.d, _settings.d) ||
        different(s.imax, _settings.imax) || different(s.heading_deadband, _settings.heading_deadband) ||
        different(s.maximum, _settings.maximum) || different(s.slew, _settings.slew) ||
        different(s.filter_hz, _settings.filter_hz)) {
        return true;
    }
    return false;
}

void HeadingHold::release()
{
    _target = _error = _integral = _command = _demand = 0;
    _wait_ms = _settings.idle_ms >= 0 ? _settings.idle_ms : 0;
}

float HeadingHold::constrain(float request, float dt)
{
    const float maximum = fminf(_settings.maximum, _actuation.maximum);
    const float desired = bounded(request, -maximum, maximum);
    const float slew = fminf(_settings.slew, _actuation.slew);
    const float slewed = _command + bounded(desired - _command, -slew*dt, slew*dt);
    if (fabsf(slewed - desired) > 1.0e-6f) {
        _inhibited |= Slew;
    }
    // A reduced envelope takes precedence over slew history.
    const float result = bounded(slewed, -maximum, maximum);
    _travel_scale = fabsf(request) > maximum && fabsf(request) > 0 ? maximum / fabsf(request) : 1;
    if (fabsf(request) > maximum || fabsf(result - slewed) > 1.0e-6f) {
        _inhibited |= Travel;
    }
    return result;
}

void HeadingHold::update(uint32_t now_ms, const HeadingSettings& settings,
                         const HeadingSample& sample, const HeadingActuation& actuation)
{
    if (!settings.enabled) {
        if (_settings.enabled) {
            *this = HeadingHold{};
        }
        return;
    }

    const bool reconfigured = changed(settings) || sample.routes_changed ||
        actuation.revision != _actuation.revision || actuation.available != _actuation.available ||
        different(actuation.slew, _actuation.slew) || different(actuation.maximum, _actuation.maximum);
    const bool was_enabled = _settings.enabled;
    const uint32_t elapsed_ms = now_ms - _last_ms;
    const float dt = was_enabled && elapsed_ms <= 100 ? elapsed_ms * 0.001f : 0;
    _last_ms = now_ms;
    _settings = settings;
    _actuation = actuation;
    _inhibited = 0;
    _travel_scale = 1;
    if (reconfigured) {
        release();
        _filter_ready = false;
        _state = State::Inhibited;
    }
    if (!valid(settings) || !actuation.configuration_valid ||
        !in_range(actuation.slew, 0.01f, 100) || !in_range(actuation.maximum, 0, 1)) {
        _inhibited |= Configuration;
    }
    if (!sample.compass_valid) {
        _inhibited |= Compass;
    }
    if (!sample.attitude_valid || !isfinite(sample.heading_deg) || !isfinite(sample.rate_dps)) {
        _inhibited |= Attitude;
    }
    if (!sample.control_valid || (!actuation.available && !actuation.manual)) {
        _inhibited |= Input;
    }
    if (!sample.routes_valid) {
        _inhibited |= Routing;
    }
    if (was_enabled && elapsed_ms > 100 && !reconfigured) {
        _inhibited |= Timing;
    }
    if (sample.yaw_reset) {
        _inhibited |= YawReset;
    }
    _heading = isfinite(sample.heading_deg) ? fmodf(sample.heading_deg, 360) : 0;
    if (_heading < 0) {
        _heading += 360;
    }
    if (_inhibited != 0) {
        release();
        _state = State::Inhibited;
        _filter_ready = false;
        _rate = 0;
        return;
    }

    if (!_filter_ready) {
        _rate = sample.rate_dps;
        _filter_ready = true;
    } else {
        const float tau = 1 / (6.28318530718f * settings.filter_hz);
        _rate += dt / (tau + dt) * (sample.rate_dps - _rate);
    }
    if (actuation.manual) {
        release();
        _state = State::Manual;
        _inhibited = Stick;
        return;
    }
    if (_state != State::Waiting && _state != State::Hold) {
        _neutral_ms = now_ms;
        _state = State::Waiting;
    }
    if (_state == State::Waiting) {
        const uint32_t idle = now_ms - _neutral_ms;
        if (idle < uint16_t(settings.idle_ms)) {
            _wait_ms = settings.idle_ms - idle;
            _inhibited = Delay;
            return;
        }
        // Capture at the END of the neutral interval, including on recovery.
        _target = _heading;
        _state = State::Hold;
        _wait_ms = 0;
        return; // slew starts from zero on the following sample
    }

    _error = wrap180(_target - _heading);
    const float e = copysignf(fmaxf(0, fabsf(_error) - settings.heading_deadband), _error);
    const float old_integral = _integral;
    _integral = bounded(_integral + settings.i * e * dt, -settings.imax, settings.imax);
    const float pd = settings.p * e - settings.d * _rate;
    float request = pd + _integral;
    float command = constrain(request, dt);
    // Conditional integration accounts for the common normalized envelope and slew.
    // Always permit unwinding; do not accumulate effort beyond those constraints.
    if ((_integral - old_integral) * (request - command) > 0) {
        // Keep the deliverable part of this increment. This also lets an
        // I-only controller start moving when its increment exceeds SLEW.
        _integral = bounded(command - pd, fminf(old_integral, _integral),
                            fmaxf(old_integral, _integral));
        request = pd + _integral;
        _inhibited = 0;
        command = constrain(request, dt);
    }
    _demand = bounded(request, -settings.maximum, settings.maximum);
    _command = command;

}
} // namespace Vektor2
