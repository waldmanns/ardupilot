#include "RollStabilization.h"

#include <math.h>
#include <string.h>

namespace Vektor2 {
namespace {
constexpr float pi = 3.14159265358979323846f;

float bounded(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

// These are stored parameter snapshots, not noisy measurements. Compare their
// representations so even small live edits reset the controller deterministically.
bool different(float first, float second)
{
    return memcmp(&first, &second, sizeof(first)) != 0;
}

bool in_range(float value, float low, float high)
{
    return isfinite(value) && value >= low && value <= high;
}

bool inside(Coordinates point, Limiter limits)
{
    // Use integer geometry for the FINAL integer command. Float rounding must
    // not allow a point just outside the mechanical ring to pass this check.
    const int64_t x = int64_t(point.x_coordinate) - limits.x_offset;
    const int64_t y = int64_t(point.y_coordinate) - limits.y_offset;
    return point.x_coordinate >= VSP_PWM_MIN && point.x_coordinate <= VSP_PWM_MAX &&
           point.y_coordinate >= VSP_PWM_MIN && point.y_coordinate <= VSP_PWM_MAX &&
           x*x + y*y <= int64_t(limits.radius)*limits.radius;
}

Coordinates displaced(Coordinates base, float amplitude, float x, float y, float radius)
{
    // Integer PWM cannot represent every angle exactly. Round to nearest us;
    // the deviation from the ideal ray is at most sqrt(0.5) us. Never quantize
    // the maneuvering baseline again when the requested addition is zero.
    return {base.x_coordinate + int32_t(lroundf(amplitude * radius * x)),
            base.y_coordinate + int32_t(lroundf(amplitude * radius * y))};
}
} // namespace

bool RollStabilization::valid(const RollSettings& s, const RollDriveSettings (&drives)[2])
{
    if (!in_range(s.p, 0, 10) || !in_range(s.d, 0, 10) ||
        !in_range(s.trim_deg, -45, 45) || !in_range(s.filter_hz, 0.1f, 20) ||
        !in_range(s.deadband, 0, 0.5f) || !in_range(s.slew, 0.01f, 100) ||
        !in_range(s.maneuver_limit, 0.01f, 1) ||
        s.on_ms < 20 || s.on_ms > 10000 || s.off_ms < 20 || s.off_ms > 10000) {
        return false;
    }
    for (const auto& drive : drives) {
        if (drive.angle_deg < 0 || drive.angle_deg > 359 || drive.sign < -1 ||
            drive.sign > 1 || !in_range(drive.maximum, 0, 1)) {
            return false;
        }
    }
    // A single configured drive is useful during commissioning. Two enabled
    // drives must have opposite effects; never double a correction by accident.
    return drives[0].sign == 0 || drives[1].sign == 0 || drives[0].sign != drives[1].sign;
}

bool RollStabilization::changed(const RollSettings& s, const RollDriveSettings (&drives)[2]) const
{
    // Compare fields, not struct padding. Live edits must not rotate an ongoing
    // burst or apply a newly changed gain to stale filter/slew state.
    if (s.enabled != _settings.enabled || different(s.p, _settings.p) || different(s.d, _settings.d) ||
        different(s.trim_deg, _settings.trim_deg) || different(s.filter_hz, _settings.filter_hz) ||
        different(s.deadband, _settings.deadband) || s.on_ms != _settings.on_ms ||
        s.off_ms != _settings.off_ms || different(s.slew, _settings.slew) ||
        different(s.maneuver_limit, _settings.maneuver_limit)) {
        return true;
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (drives[i].angle_deg != _drives[i].angle_deg || drives[i].sign != _drives[i].sign ||
            different(drives[i].maximum, _drives[i].maximum)) {
            return true;
        }
    }
    return false;
}

void RollStabilization::stop()
{
    if (_state == State::Burst) {
        _recovery_ms = _now_ms;
        _state = State::Recovery;
    }
    _amplitude = 0;
    _burst_drive = -1;
}

void RollStabilization::account_time(uint32_t elapsed_ms)
{
    // Integrate the command that was held over the PREVIOUS sample interval.
    // One-second summaries expose short bursts even on a slower telemetry link.
    // A stalled loop is not useful duty data; never fold a long gap into it.
    if (elapsed_ms > 100) {
        _window_ms = 0;
        _active_ms[0] = _active_ms[1] = 0;
        _status[0].duty = _status[1].duty = 0;
        return;
    }
    _window_ms += elapsed_ms;
    for (uint8_t i = 0; i < 2; i++) {
        if (_status[i].command > 0) {
            _active_ms[i] += elapsed_ms;
        }
    }
    if (_window_ms >= 1000) {
        for (uint8_t i = 0; i < 2; i++) {
            _status[i].duty = float(_active_ms[i]) / _window_ms;
            _active_ms[i] = 0;
        }
        _window_ms = 0;
    }
}

void RollStabilization::update(uint32_t now_ms, const RollSettings& settings,
                             const RollDriveSettings (&drives)[2], const RollSample& sample)
{
    if (!settings.enabled) {
        // Disable is a hard bypass, including an ongoing burst. No ramp tail,
        // old demand, recovery debt or filter history survives re-enabling.
        if (_settings.enabled) {
            *this = RollStabilization{};
        }
        return;
    }

    const bool reconfigured = changed(settings, drives);
    const uint32_t elapsed_ms = now_ms - _now_ms; // wrap-safe millisecond clock
    if (_settings.enabled) {
        account_time(elapsed_ms);
    }
    _now_ms = now_ms;
    _settings = settings;
    _drives[0] = drives[0];
    _drives[1] = drives[1];
    _inhibited = 0;
    _selected = -1;
    _demand = 0;
    _dt = elapsed_ms <= 100 ? elapsed_ms * 0.001f : 0;
    for (uint8_t i = 0; i < 2; i++) {
        _status[i].command = 0;
        _status[i].headroom = 0;
        _status[i].limited = 0;
        _applied[i] = false;
    }

    if (reconfigured) {
        stop();
        _filter_ready = false;
        _demand_active = false;
        _state = State::Waiting;
        _dt = 0;
    }
    if (!valid(settings, drives)) {
        _inhibited |= Configuration;
    }
    if (!sample.attitude_valid || !isfinite(sample.roll_deg) || !isfinite(sample.rate_dps)) {
        _inhibited |= Attitude;
    }
    if (!sample.control_valid) {
        _inhibited |= Input;
    }
    if (elapsed_ms > 100 && !reconfigured) {
        _inhibited |= Timing;
    }
    if (_inhibited != 0) {
        stop();
        _state = State::Waiting;
        _filter_ready = false;
        _demand_active = false;
        _roll = _rate = 0;
        return;
    }

    _roll = sample.roll_deg;
    if (!_filter_ready) {
        // Seed from the current rate: starting the LPF at zero would briefly
        // hide an already moving hull. Recovery allows fresh samples to settle.
        _rate = sample.rate_dps;
        _filter_ready = true;
        _recovery_ms = now_ms;
        _state = State::Recovery;
    } else {
        const float tau = 1.0f / (2.0f * pi * settings.filter_hz);
        _rate += (_dt / (tau + _dt)) * (sample.rate_dps - _rate);
    }
    _demand = bounded(-settings.p * (_roll - settings.trim_deg) - settings.d * _rate, -1, 1);
    const float magnitude = fabsf(_demand);
    if (magnitude > settings.deadband) {
        _demand_active = true;
    } else if (magnitude <= settings.deadband * 0.5f) {
        _demand_active = false;
    }
    if (_demand_active) {
        for (uint8_t i = 0; i < 2; i++) {
            if (drives[i].sign * _demand > 0 && drives[i].maximum > 0) {
                _selected = int8_t(i);
            }
        }
        if (_selected < 0) {
            _inhibited |= NoDrive;
        }
    } else {
        _inhibited |= Deadband;
    }

    // A pulse is not latched to the old sign or intensity. Stop at reversal,
    // loss of demand or the hard on-time bound. The shared timer prevents the
    // other VSP from bypassing mandatory maneuver-only time on the next tick.
    if (_state == State::Burst &&
        (_selected != _burst_drive || now_ms - _burst_ms >= settings.on_ms)) {
        stop();
    }
    if (_state == State::Recovery) {
        if (now_ms - _recovery_ms < settings.off_ms) {
            _inhibited |= Recovery;
        } else {
            _state = State::Ready;
        }
    }
}

Coordinates RollStabilization::apply(uint8_t drive, Coordinates base, Limiter limits,
                                    bool inputs_valid)
{
    // This check precedes EVERY calculation and output modification. VRS off
    // preserves even legacy edge cases rather than silently "fixing" them.
    if (!_settings.enabled || drive >= 2) {
        return base;
    }
    if (_applied[drive]) {
        return base; // at most one allocation per drive in one routing cycle
    }
    _applied[drive] = true;
    // An excluded drive has no VRS input requirements. Global invalidity was
    // already diagnosed by update(); do not obscure it with baseline warnings.
    if (_drives[drive].sign == 0 || _state == State::Waiting) {
        return base;
    }
    auto& diag = _status[drive];
    if (!inputs_valid) {
        diag.limited |= Input;
    }
    // Invalid baselines are not moved or re-normalized. The old limiter can
    // place an integer point slightly outside its circle through truncation;
    // in that case it has no admissible VRS baseline until maneuvering moves in.
    if (limits.radius <= 0 || limits.radius > 1000 ||
        limits.x_offset < VSP_PWM_MIN || limits.x_offset > VSP_PWM_MAX ||
        limits.y_offset < VSP_PWM_MIN || limits.y_offset > VSP_PWM_MAX || !inside(base, limits)) {
        diag.limited |= Baseline;
    }
    if (diag.limited != 0) {
        _inhibited |= diag.limited;
        if (_burst_drive == int8_t(drive)) {
            stop();
        }
        return base;
    }
    const float angle = _drives[drive].angle_deg * (pi / 180.0f);
    const float ux = cosf(angle);
    const float uy = sinf(angle);
    const float radius = float(limits.radius);
    const float mx = (base.x_coordinate - limits.x_offset) / radius;
    const float my = (base.y_coordinate - limits.y_offset) / radius;
    const float norm2 = mx*mx + my*my;
    const float projection = mx*ux + my*uy;

    // Solve |m + a*u|^2 <= 1 for the nonnegative ray coordinate a. Constraining
    // ONLY a preserves the maneuver command and the direction of the addition.
    // Radially clipping the combined vector would violate that contract.
    float headroom = fmaxf(0, -projection + sqrtf(fmaxf(0, projection*projection + 1 - norm2)));
    if (fabsf(ux) > 1.0e-6f) {
        const float edge = ux > 0 ? VSP_PWM_MAX : VSP_PWM_MIN;
        headroom = fminf(headroom, (edge - base.x_coordinate) / (radius * ux));
    }
    if (fabsf(uy) > 1.0e-6f) {
        const float edge = uy > 0 ? VSP_PWM_MAX : VSP_PWM_MIN;
        headroom = fminf(headroom, (edge - base.y_coordinate) / (radius * uy));
    }
    diag.headroom = fmaxf(0, headroom);
    if (_selected != int8_t(drive) || _dt <= 0 ||
        (_state != State::Ready && _state != State::Burst)) {
        return base;
    }

    // Linear priority fade: neutral grants full VRS authority; reaching MAN_LIM
    // grants none. This is a deliberate authority policy, separate from the
    // geometric headroom (an opposing correction may have ample headroom).
    const float priority = bounded(1 - sqrtf(norm2) / _settings.maneuver_limit, 0, 1);
    const float authority = _drives[drive].maximum * priority;
    const float requested = fminf(fabsf(_demand), authority);
    if (priority < 1) {
        diag.limited |= Maneuver;
    }
    if (headroom < requested) {
        diag.limited |= Travel;
    }
    _inhibited |= diag.limited;
    const float target = fminf(requested, headroom);
    if (target <= 0) {
        stop();
        return base;
    }
    if (_state == State::Ready) {
        _state = State::Burst;
        _burst_ms = _now_ms;
        _burst_drive = int8_t(drive);
        _amplitude = 0;
        diag.bursts++;
    }

    const float step = _settings.slew * _dt;
    _amplitude = bounded(target, fmaxf(0, _amplitude - step), _amplitude + step);
    // Geometry and maneuver authority are hard constraints. They override a
    // slow ramp-down when the pilot suddenly moves the rod or needs priority.
    _amplitude = fminf(_amplitude, fminf(headroom, authority));

    Coordinates result = displaced(base, _amplitude, ux, uy, radius);
    if (!inside(result, limits)) {
        // Rounding both coordinates can cross the ring even though the float
        // point was valid. Search down the SAME scalar ray, keeping a known
        // valid integer candidate. No independent X/Y clipping or re-rotation.
        float low = 0;
        float high = _amplitude;
        result = base;
        for (uint8_t iteration = 0; iteration < 16; iteration++) {
            const float mid = (low + high) * 0.5f;
            const Coordinates candidate = displaced(base, mid, ux, uy, radius);
            if (inside(candidate, limits)) {
                result = candidate;
                low = mid;
            } else {
                high = mid;
            }
        }
        diag.limited |= Travel;
        _inhibited |= Travel;
    }
    // Report the addition that survives integer PWM quantization, not the
    // unclipped request. This is commanded displacement, never measured thrust.
    diag.command = ((result.x_coordinate - base.x_coordinate)*ux +
                    (result.y_coordinate - base.y_coordinate)*uy) / radius;
    return result;
}

} // namespace Vektor2
