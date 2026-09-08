#include "AxisLimiter.h"

#include <math.h>

namespace {

constexpr uint32_t max_slew_dt_ms = 1000U;

float absf(const float value)
{
    return value < 0.0f ? -value : value;
}

float constrain_unit(const float value)
{
    if (!isfinite(value)) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

float sanitize_unit_positive(const float value, const float fallback)
{
    if (!isfinite(value)) {
        return fallback;
    }
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

float sanitize_slew_rate(const float value)
{
    if (!isfinite(value) || value < 0.0f) {
        return 0.0f;
    }
    return value;
}

float slew_axis(
    const float current,
    const float target,
    const float rate,
    const uint32_t dt_ms)
{
    if (rate <= 0.0f) {
        return target;
    }
    if (dt_ms == 0U) {
        return current;
    }

    const float max_delta = rate * (float(dt_ms) * 0.001f);
    const float delta = target - current;
    if (delta > max_delta) {
        return current + max_delta;
    }
    if (delta < -max_delta) {
        return current - max_delta;
    }
    return target;
}

} // namespace

MiniDP_AxisLimiterConfig MiniDP_AxisLimiter::default_config()
{
    MiniDP_AxisLimiterConfig config{};
    config.deadband = 0.0f;
    config.surge_limit = 1.0f;
    config.sway_limit = 1.0f;
    config.yaw_limit = 1.0f;
    config.surge_slew_rate = 0.0f;
    config.sway_slew_rate = 0.0f;
    config.yaw_slew_rate = 0.0f;
    return config;
}

void MiniDP_AxisLimiter::init(const uint32_t now_ms)
{
    cfg = default_config();
    reset(now_ms);
}

void MiniDP_AxisLimiter::set_config(
    const MiniDP_AxisLimiterConfig &new_config)
{
    cfg = new_config;
    cfg.deadband = sanitize_unit_positive(cfg.deadband, 0.0f);
    cfg.surge_limit = sanitize_unit_positive(cfg.surge_limit, 1.0f);
    cfg.sway_limit = sanitize_unit_positive(cfg.sway_limit, 1.0f);
    cfg.yaw_limit = sanitize_unit_positive(cfg.yaw_limit, 1.0f);
    cfg.surge_slew_rate = sanitize_slew_rate(cfg.surge_slew_rate);
    cfg.sway_slew_rate = sanitize_slew_rate(cfg.sway_slew_rate);
    cfg.yaw_slew_rate = sanitize_slew_rate(cfg.yaw_slew_rate);
    current_command.surge = fminf(cfg.surge_limit, fmaxf(-cfg.surge_limit, current_command.surge));
    current_command.sway = fminf(cfg.sway_limit, fmaxf(-cfg.sway_limit, current_command.sway));
    current_command.yaw = fminf(cfg.yaw_limit, fmaxf(-cfg.yaw_limit, current_command.yaw));
}

MiniDP_AxisCommand MiniDP_AxisLimiter::update(
    const uint32_t now_ms,
    const MiniDP_AxisCommand &target)
{
    const MiniDP_AxisCommand limited = apply_limits_only(target);

    if (!have_update) {
        current_command = limited;
        last_update_ms = now_ms;
        have_update = true;
        return current_command;
    }

    uint32_t dt_ms = now_ms - last_update_ms;
    if (dt_ms > max_slew_dt_ms) {
        dt_ms = max_slew_dt_ms;
    }
    last_update_ms = now_ms;

    current_command.surge = slew_axis(
        current_command.surge,
        limited.surge,
        cfg.surge_slew_rate,
        dt_ms);
    current_command.sway = slew_axis(
        current_command.sway,
        limited.sway,
        cfg.sway_slew_rate,
        dt_ms);
    current_command.yaw = slew_axis(
        current_command.yaw,
        limited.yaw,
        cfg.yaw_slew_rate,
        dt_ms);

    return current_command;
}

void MiniDP_AxisLimiter::reset(const uint32_t now_ms)
{
    reset(now_ms, {});
}

void MiniDP_AxisLimiter::reset(
    const uint32_t now_ms,
    const MiniDP_AxisCommand &command)
{
    current_command = apply_limits_only(command);
    last_update_ms = now_ms;
    have_update = true;
}

MiniDP_AxisCommand MiniDP_AxisLimiter::apply_limits_only(
    const MiniDP_AxisCommand &command) const
{
    MiniDP_AxisCommand limited{};
    limited.surge = apply_axis_limit(command.surge, cfg.surge_limit);
    limited.sway = apply_axis_limit(command.sway, cfg.sway_limit);
    limited.yaw = apply_axis_limit(command.yaw, cfg.yaw_limit);
    return limited;
}

float MiniDP_AxisLimiter::apply_axis_limit(
    const float value,
    const float limit) const
{
    const float constrained = constrain_unit(value);
    if (absf(constrained) < cfg.deadband) {
        return 0.0f;
    }
    if (constrained > limit) {
        return limit;
    }
    if (constrained < -limit) {
        return -limit;
    }
    return constrained;
}
