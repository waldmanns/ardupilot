#include "AzipodControl.h"
#include <math.h>

namespace Vektor2 {
namespace Components {
namespace {
constexpr float rad = 0.017453292519943295f;
float clamp(float x, float lo, float hi) { return fmaxf(lo, fminf(hi, x)); }
float wrap(float a) { return remainderf(a, 360.0f); }
float approach(float current, float target, float step)
{
    return current + clamp(target - current, -step, step);
}
int8_t sign(float x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); }
}

bool AzipodSettings::valid() const
{
    return angle_min >= -360 && angle_max <= 360 && angle_min < angle_max &&
        angle_max - angle_min < 360 && offset >= -360 && offset <= 360 &&
        (reversible == 0 || reversible == 1) && (reverse_opt == 0 || reverse_opt == 1) &&
        (unreachable == 0 || unreachable == 1) && deadband >= 0 && deadband <= 250 &&
        rate >= 1 && rate <= 720 && hysteresis >= 0 && hysteresis <= 180 &&
        reverse_delay >= 0 && reverse_delay <= 5000 && align >= 0 && align <= 45 &&
        isfinite(motor_slew) && motor_slew > 0 && motor_slew <= 100;
}

AzipodDemand azipod_map(uint16_t x, uint16_t y, int16_t deadband)
{
    if (x < 1000 || x > 2000 || y < 1000 || y > 2000 || deadband < 0 || deadband > 250) {
        return {0, 0};
    }
    const float fx = int(x) - 1500, fy = int(y) - 1500;
    const float length = hypotf(fx, fy);
    if (length <= deadband) { return {0, 0}; }
    return {atan2f(fy, fx) / rad, clamp((length - deadband) / (500 - deadband), 0, 1)};
}

AzipodSolution azipod_solve(const AzipodSettings& s, AzipodDemand demand,
                            float current, int8_t preferred_direction)
{
    AzipodSolution best{current, 0, false};
    if (!s.valid() || !isfinite(current) || !isfinite(demand.angle) ||
        !isfinite(demand.magnitude) || demand.magnitude <= 0) { return best; }
    current = clamp(current, s.angle_min, s.angle_max);
    best.angle = current;
    const float magnitude = fminf(1, demand.magnitude);
    float best_cost = 1e9f;
    float best_projection = 0;
    AzipodSolution projected{current, 0, false};
    for (int8_t dir = 1; dir >= -1; dir -= 2) {
        if (dir < 0 && !s.reversible) { continue; }
        const float base = wrap(demand.angle - s.offset - (dir < 0 ? 180 : 0));
        for (int k = -2; k <= 2; ++k) {
            const float candidate = base + k * 360.0f;
            if (candidate >= s.angle_min - 0.0001f && candidate <= s.angle_max + 0.0001f) {
                // With optimization disabled, prefer forward if reachable;
                // reverse remains mandatory when it is the only solution.
                const float penalty = s.reverse_opt ? (dir == preferred_direction ? 0 : s.hysteresis) : (dir < 0 ? 1000 : 0);
                const float bounded = clamp(candidate, s.angle_min, s.angle_max);
                const float cost = fabsf(bounded - current) + penalty;
                if (cost < best_cost) {
                    best_cost = cost;
                    best = {bounded, dir * magnitude, true};
                }
            }
            if (s.unreachable) {
                const float a = clamp(candidate, s.angle_min, s.angle_max);
                const float projection = fmaxf(0, cosf(wrap(base - a) * rad));
                if (projection > best_projection && projection > 0.000001f) {
                    best_projection = projection;
                    projected = {a, dir * magnitude * projection, false};
                }
            }
        }
    }
    return best.reachable ? best : projected;
}

void AzipodControl::configure(const AzipodSettings& s)
{
    _settings = s;
    _reasons = s.valid() ? 0 : CalibrationInvalid;
    _angle = _target = (float(s.angle_min) + s.angle_max) * 0.5f;
    _motor = 0;
    _running = _armed = _calibrating = false;
    _homing = true;
    _last_direction = 0;
    _preferred = 1;
}

void AzipodControl::stop(uint32_t now)
{
    if (sign(_motor) != 0) { _stopped_since = now; }
    _motor = 0;
}

uint16_t AzipodControl::servo_pwm() const
{
    return uint16_t(lroundf(1000 + 1000 * clamp((_angle - _settings.angle_min) /
                       (_settings.angle_max - _settings.angle_min), 0, 1)));
}
uint16_t AzipodControl::motor_pwm() const
{
    return uint16_t(lroundf(_settings.reversible ? 1500 + 500 * _motor : 1000 + 1000 * _motor));
}

void AzipodControl::calibrate(bool selected, uint16_t steering, int8_t motor_percent, bool routes_ready, uint16_t* outputs)
{
    _reasons = routes_ready ? 0 : OutputRouteMissing;
    // Normal control rehomes and requires neutral after calibration.
    _running = _armed = _calibrating = false;
    _motor = 0;
    outputs[AZ_OUT] = 0;
    outputs[MOT_OUT] = motor_stop();
    if (!_settings.valid()) { _reasons |= CalibrationInvalid; return; }
    if (selected) {
        _angle = _target = _settings.angle_min + (_settings.angle_max - _settings.angle_min) *
                           (clamp(steering, 1000, 2000) - 1000) / 1000;
        _motor = clamp(motor_percent, reversible() ? -20 : 0, 20) * 0.01f;
    }
    _target = _angle;
    _calibrating = true;
    outputs[AZ_OUT] = servo_pwm();
    outputs[MOT_OUT] = motor_pwm();
}

void AzipodControl::run(uint32_t now, uint32_t revision, const uint16_t* inputs,
                        const bool* valid, bool routes_ready, bool rc_valid, uint16_t* outputs)
{
    _calibrating = false;
    _reasons = 0;
    if (!rc_valid) { _reasons |= RcInvalid; }
    const bool input_invalid = !valid[0] || !valid[1] || inputs[0] < 1000 || inputs[0] > 2000 ||
                               inputs[1] < 1000 || inputs[1] > 2000;
    if (input_invalid) { _reasons |= InputInvalid; }
    if (!routes_ready) { _reasons |= OutputRouteMissing; }
    outputs[AZ_OUT] = outputs[MOT_OUT] = 0;
    if (!_settings.valid()) {
        _reasons |= CalibrationInvalid;
        stop(now); _running = _armed = false; return;
    }
    // A missing/unavailable steering route must never permit motor thrust.
    // A remaining motor route receives its calibrated logical stop.
    outputs[MOT_OUT] = motor_stop();
    if (!routes_ready) { stop(now); _running = _armed = false; return; }
    const uint32_t elapsed = now - _last;
    if (!_running || elapsed > 100 || revision != _revision) {
        stop(now); _armed = false; _homing = true; _home_start = now;
        _angle = _target = (float(_settings.angle_min) + _settings.angle_max) * 0.5f;
        _last_direction = 0; _preferred = 1;
        _stopped_since = now;
        _running = true; _revision = revision; _last = now;
        _reasons |= StartupCentering;
        outputs[AZ_OUT] = servo_pwm(); return;
    }
    _last = now;
    const float dt = elapsed * 0.001f;
    outputs[AZ_OUT] = servo_pwm();
    // Open-loop startup: allow a full travel time plus margin to reach center.
    // This is a timing estimate, never measured position or jam detection.
    const uint32_t home_ms = uint32_t(ceilf(1000.0f * (_settings.angle_max - _settings.angle_min) / _settings.rate)) + 250;
    if (_homing) {
        if (now - _home_start < home_ms || now - _home_start < uint32_t(_settings.reverse_delay)) { _reasons |= StartupCentering; return; }
        _homing = false;
    }
    if (!rc_valid || input_invalid) {
        _reasons |= NeutralRequired;
        stop(now); _armed = false; _target = _angle; return;
    }
    const auto demand = azipod_map(inputs[0], inputs[1], _settings.deadband);
    if (demand.magnitude <= 0.0f) {
        _armed = true; stop(now); _target = _angle; return;
    }
    if (!_armed) { _reasons |= NeutralRequired; return; }
    const auto solution = azipod_solve(_settings, demand, _angle, _preferred);
    const int8_t direction = sign(solution.thrust);
    if (!solution.reachable) {
        _reasons |= DirectionUnreachable;
        if (direction) { _reasons |= DirectionProjected; }
    }
    if (!direction) { stop(now); _target = _angle; return; }
    _preferred = direction;
    _target = solution.angle;
    // Compare previous commanded position before stepping; actual servo may lag.
    const float error = fabsf(_target - _angle);
    _angle = approach(_angle, _target, _settings.rate * dt);
    outputs[AZ_OUT] = servo_pwm();
    if (error > _settings.align) { _reasons |= AlignmentPending; }
    const int8_t motor_direction = sign(_motor);
    const bool reversing = motor_direction != 0 && motor_direction != direction;
    if (reversing) { _reasons |= ReversalDelay; }
    if (error > _settings.align || reversing) {
        stop(now); return;
    }
    if (_last_direction && direction != _last_direction &&
        now - _stopped_since < uint32_t(_settings.reverse_delay)) { _reasons |= ReversalDelay; return; }
    const float target = solution.thrust * fmaxf(0, cosf(error * rad));
    _motor = approach(_motor, target, _settings.motor_slew * dt);
    const int8_t next_motor_direction = sign(_motor);
    if (next_motor_direction != 0) { _last_direction = next_motor_direction; }
    outputs[MOT_OUT] = motor_pwm();
}
} // namespace Components
} // namespace Vektor2
