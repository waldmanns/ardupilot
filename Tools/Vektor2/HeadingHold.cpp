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

bool valid_drive(const HeadingDrive& drive)
{
    for (const auto input : drive.inputs) {
        if (input < VSP_PWM_MIN || input > VSP_PWM_MAX) {
            return false;
        }
    }
    return drive.sources_valid;
}

bool valid_geometry(const HeadingDrive& drive)
{
    const auto& l = drive.limits;
    const auto& b = drive.baseline;
    const int64_t x = int64_t(b.x_coordinate) - l.x_offset;
    const int64_t y = int64_t(b.y_coordinate) - l.y_offset;
    return l.radius > 0 && l.radius <= 1000 &&
           l.x_offset >= VSP_PWM_MIN && l.x_offset <= VSP_PWM_MAX &&
           l.y_offset >= VSP_PWM_MIN && l.y_offset <= VSP_PWM_MAX &&
           b.x_coordinate >= VSP_PWM_MIN && b.x_coordinate <= VSP_PWM_MAX &&
           b.y_coordinate >= VSP_PWM_MIN && b.y_coordinate <= VSP_PWM_MAX &&
           x*x + y*y <= int64_t(l.radius)*l.radius;
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
        {Configuration, "Config/ring invalid"},
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
           in_range(s.filter_hz, 0.1f, 20) &&
           (!s.vsp_enabled || (s.invert[0] >= 0 && s.invert[0] <= 1 && s.invert[1] >= 0 && s.invert[1] <= 1));
}

bool HeadingHold::changed(const HeadingSettings& s, const HeadingDrive (&drives)[2]) const
{
    if (s.enabled != _settings.enabled || s.vsp_enabled != _settings.vsp_enabled || s.stick_deadband != _settings.stick_deadband ||
        s.idle_ms != _settings.idle_ms || different(s.p, _settings.p) ||
        different(s.i, _settings.i) || different(s.d, _settings.d) ||
        different(s.imax, _settings.imax) || different(s.heading_deadband, _settings.heading_deadband) ||
        different(s.maximum, _settings.maximum) || different(s.slew, _settings.slew) ||
        different(s.filter_hz, _settings.filter_hz)) {
        return true;
    }
    for (uint8_t n = 0; s.vsp_enabled && n < 2; n++) {
        if (s.invert[n] != _settings.invert[n] || drives[n].limits.radius != _limits[n].radius ||
            drives[n].limits.x_offset != _limits[n].x_offset ||
            drives[n].limits.y_offset != _limits[n].y_offset) {
            return true;
        }
    }
    return false;
}

void HeadingHold::release()
{
    _target = _error = _integral = _command = _demand = 0;
    _correction[0] = _correction[1] = 0;
    _wait_ms = _settings.idle_ms >= 0 ? _settings.idle_ms : 0;
}

float HeadingHold::allocate(float request, float dt, const HeadingDrive (&drives)[2])
{
    const float desired = bounded(request, -_settings.maximum, _settings.maximum);
    const float slew = _auxiliary.available ? fminf(_settings.slew, _auxiliary.slew) : _settings.slew;
    const float slewed = _command + bounded(desired - _command, -slew*dt, slew*dt);
    if (fabsf(slewed - desired) > 1.0e-6f) {
        _inhibited |= Slew;
    }
    float amplitude = fabsf(slewed);
    for (uint8_t n = 0; _settings.vsp_enabled && n < 2; n++) {
        const auto& drive = drives[n];
        const auto& l = drive.limits;
        const int64_t x = int64_t(drive.baseline.x_coordinate) - l.x_offset;
        const int32_t span = int32_t(floorf(sqrtf(float(int64_t(l.radius)*l.radius - x*x))));
        const int32_t low = l.y_offset - span < VSP_PWM_MIN ? VSP_PWM_MIN : l.y_offset - span;
        const int32_t high = l.y_offset + span > VSP_PWM_MAX ? VSP_PWM_MAX : l.y_offset + span;
        const bool positive = (slewed >= 0) != bool(_settings.invert[n]);
        const float room = float(positive ? high - drive.baseline.y_coordinate :
                                           drive.baseline.y_coordinate - low) / l.radius;
        amplitude = fminf(amplitude, room);
    }
    _travel_scale = fabsf(slewed) > 0 ? amplitude / fabsf(slewed) : 1;
    if (_travel_scale < 1 || fabsf(request) > _settings.maximum) {
        _inhibited |= Travel;
    }
    return copysignf(amplitude, slewed);
}

void HeadingHold::update(uint32_t now_ms, const HeadingSettings& settings,
                         const HeadingSample& sample, const HeadingDrive (&drives)[2])
{
    if (!settings.enabled) {
        if (_settings.enabled) {
            *this = HeadingHold{};
        }
        return;
    }

    const bool reconfigured = changed(settings, drives) || sample.routes_changed ||
        sample.auxiliary.revision != _auxiliary.revision ||
        sample.auxiliary.available != _auxiliary.available || different(sample.auxiliary.slew, _auxiliary.slew);
    const bool was_enabled = _settings.enabled;
    const uint32_t elapsed_ms = now_ms - _last_ms;
    const float dt = was_enabled && elapsed_ms <= 100 ? elapsed_ms * 0.001f : 0;
    _last_ms = now_ms;
    _settings = settings;
    _auxiliary = sample.auxiliary;
    _limits[0] = drives[0].limits;
    _limits[1] = drives[1].limits;
    _inhibited = 0;
    _correction[0] = _correction[1] = 0;
    _travel_scale = 1;
    if (reconfigured) {
        release();
        _filter_ready = false;
        _state = State::Inhibited;
    }
    if (!valid(settings) || !isfinite(_auxiliary.slew) || _auxiliary.slew <= 0 || (settings.vsp_enabled && sample.routes_valid &&
                            (!valid_geometry(drives[0]) || !valid_geometry(drives[1])))) {
        _inhibited |= Configuration;
    }
    if (!sample.compass_valid) {
        _inhibited |= Compass;
    }
    if (!sample.attitude_valid || !isfinite(sample.heading_deg) || !isfinite(sample.rate_dps)) {
        _inhibited |= Attitude;
    }
    if (!sample.control_valid || (!settings.vsp_enabled && !_auxiliary.available && !_auxiliary.manual) || (settings.vsp_enabled && sample.routes_valid &&
                                 (!valid_drive(drives[0]) || !valid_drive(drives[1])))) {
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
    if (_auxiliary.manual) {
        release();
        _state = State::Manual;
        _inhibited = Stick;
        return;
    }
    for (uint8_t n = 0; settings.vsp_enabled && n < 2; n++) {
        const auto& drive = drives[n];
        for (uint8_t axis = 0; axis < 2; axis++) {
            const int32_t offset = int32_t(drive.inputs[axis]) - 1500;
            if (offset < -settings.stick_deadband || offset > settings.stick_deadband) {
                release();
                _state = State::Manual;
                _inhibited = Stick;
                return;
            }
        }
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
    float command = allocate(request, dt, drives);
    // Conditional integration accounts for maximum, common travel AND slew.
    // Always permit unwinding; never accumulate demand the rods cannot deliver.
    if ((_integral - old_integral) * (request - command) > 0) {
        // Keep the deliverable part of this increment. This also lets an
        // I-only controller start moving when its increment exceeds SLEW.
        _integral = bounded(command - pd, fminf(old_integral, _integral),
                            fmaxf(old_integral, _integral));
        request = pd + _integral;
        _inhibited = 0;
        command = allocate(request, dt, drives);
    }
    _demand = bounded(request, -settings.maximum, settings.maximum);
    _command = command;
    for (uint8_t n = 0; settings.vsp_enabled && n < 2; n++) {
        const int32_t magnitude = int32_t(floorf(fabsf(command) * drives[n].limits.radius));
        _correction[n] = ((command >= 0) != bool(settings.invert[n])) ? magnitude : -magnitude;
    }
}
} // namespace Vektor2
