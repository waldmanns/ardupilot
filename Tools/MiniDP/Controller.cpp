#include "Controller.h"

#include <math.h>

namespace {

constexpr float pi = 3.14159265358979323846f;
constexpr float two_pi = 2.0f * pi;

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

float sanitize_non_negative(const float value)
{
    if (!isfinite(value) || value < 0.0f) {
        return 0.0f;
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

float wrap_pi(float angle_rad)
{
    while (angle_rad > pi) {
        angle_rad -= two_pi;
    }
    while (angle_rad < -pi) {
        angle_rad += two_pi;
    }
    return angle_rad;
}

float constrain_axis(const float value, const float limit)
{
    const float constrained = constrain_unit(value);
    if (constrained > limit) {
        return limit;
    }
    if (constrained < -limit) {
        return -limit;
    }
    return constrained;
}

void apply_position_shaping(
    float &error_n,
    float &error_e,
    const float deadband_m,
    const float radius_m)
{
    const float magnitude = sqrtf((error_n * error_n) + (error_e * error_e));
    if (magnitude <= deadband_m || magnitude <= 0.0f) {
        error_n = 0.0f;
        error_e = 0.0f;
        return;
    }

    if (radius_m <= deadband_m || magnitude >= radius_m) {
        return;
    }

    const float scale = (magnitude - deadband_m) / (radius_m - deadband_m);
    error_n *= scale;
    error_e *= scale;
}

} // namespace

MiniDP_ControllerConfig MiniDP_Controller::default_config()
{
    MiniDP_ControllerConfig config{};
    config.yaw_p = 0.0f;
    config.yaw_d = 0.0f;
    config.position_p = 0.0f;
    config.velocity_d = 0.0f;
    config.position_radius_m = 2.0f;
    config.position_deadband_m = 0.5f;
    config.surge_limit = 0.5f;
    config.sway_limit = 0.5f;
    config.yaw_limit = 0.5f;
    return config;
}

void MiniDP_Controller::init()
{
    cfg = default_config();
}

void MiniDP_Controller::set_config(
    const MiniDP_ControllerConfig &new_config)
{
    cfg = new_config;
    cfg.yaw_p = sanitize_non_negative(cfg.yaw_p);
    cfg.yaw_d = sanitize_non_negative(cfg.yaw_d);
    cfg.position_p = sanitize_non_negative(cfg.position_p);
    cfg.velocity_d = sanitize_non_negative(cfg.velocity_d);
    cfg.position_radius_m = sanitize_non_negative(cfg.position_radius_m);
    cfg.position_deadband_m = sanitize_non_negative(cfg.position_deadband_m);
    if (cfg.position_radius_m < cfg.position_deadband_m) {
        cfg.position_radius_m = cfg.position_deadband_m;
    }
    cfg.surge_limit = sanitize_unit_positive(cfg.surge_limit, 0.5f);
    cfg.sway_limit = sanitize_unit_positive(cfg.sway_limit, 0.5f);
    cfg.yaw_limit = sanitize_unit_positive(cfg.yaw_limit, 0.5f);
}

MiniDP_AxisCommand MiniDP_Controller::update(
    const MiniDP_Mode mode,
    const MiniDP_ModeTarget &target,
    const MiniDP_State &state) const
{
    switch (mode) {
    case MiniDP_Mode::HEADING_HOLD:
        return update_heading_hold(target, state);
    case MiniDP_Mode::DP_HOLD:
        return update_dp_hold(target, state);
    case MiniDP_Mode::MANUAL:
    case MiniDP_Mode::ACTUATOR_TEST:
    case MiniDP_Mode::FAILSAFE:
        break;
    }
    return {};
}

MiniDP_AxisCommand MiniDP_Controller::update_heading_hold(
    const MiniDP_ModeTarget &target,
    const MiniDP_State &state) const
{
    MiniDP_AxisCommand command{};
    if (!target.yaw_valid || !state.yaw_valid) {
        return command;
    }

    const float yaw_error = wrap_pi(target.yaw_rad - state.yaw_rad);
    command.yaw = constrain_axis(
        (cfg.yaw_p * yaw_error) - (cfg.yaw_d * state.yaw_rate_rad_s),
        cfg.yaw_limit);
    return command;
}

MiniDP_AxisCommand MiniDP_Controller::update_dp_hold(
    const MiniDP_ModeTarget &target,
    const MiniDP_State &state) const
{
    MiniDP_AxisCommand command = update_heading_hold(target, state);
    if (!target.position_valid ||
        !state.position_valid ||
        !state.velocity_valid ||
        !state.yaw_valid) {
        command.surge = 0.0f;
        command.sway = 0.0f;
        return command;
    }

    float error_n = target.pos_n_m - state.pos_n_m;
    float error_e = target.pos_e_m - state.pos_e_m;
    apply_position_shaping(
        error_n,
        error_e,
        cfg.position_deadband_m,
        cfg.position_radius_m);

    const float yaw_cos = cosf(state.yaw_rad);
    const float yaw_sin = sinf(state.yaw_rad);

    const float error_surge = (yaw_cos * error_n) + (yaw_sin * error_e);
    const float error_sway = (-yaw_sin * error_n) + (yaw_cos * error_e);
    const float velocity_surge =
        (yaw_cos * state.vel_n_m_s) + (yaw_sin * state.vel_e_m_s);
    const float velocity_sway =
        (-yaw_sin * state.vel_n_m_s) + (yaw_cos * state.vel_e_m_s);

    command.surge = constrain_axis(
        (cfg.position_p * error_surge) -
        (cfg.velocity_d * velocity_surge),
        cfg.surge_limit);
    command.sway = constrain_axis(
        (cfg.position_p * error_sway) -
        (cfg.velocity_d * velocity_sway),
        cfg.sway_limit);
    return command;
}
