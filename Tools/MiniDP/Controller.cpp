#include "Controller.h"
#include "ControlMath.h"

#include <math.h>

namespace {

constexpr float max_controller_dt_s = 1.0f;

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
    return fminf(value, 100.0f);
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

float constrain_float_range(
    const float value,
    const float low,
    const float high)
{
    if (!isfinite(value)) {
        return 0.0f;
    }
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

float vector_magnitude(const float x, const float y)
{
    return sqrtf((x * x) + (y * y));
}

void constrain_vector_length(float &x, float &y, const float limit)
{
    if (!isfinite(x) || !isfinite(y)) {
        x = 0.0f;
        y = 0.0f;
        return;
    }
    if (limit <= 0.0f) {
        x = 0.0f;
        y = 0.0f;
        return;
    }

    const float magnitude = vector_magnitude(x, y);
    if (magnitude <= limit || magnitude <= 0.0f) {
        return;
    }

    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
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
    config.yaw_i = 0.0f;
    config.yaw_d = 0.0f;
    config.position_p = 0.0f;
    config.position_i = 0.0f;
    config.velocity_d = 0.0f;
    config.position_radius_m = 2.0f;
    config.position_deadband_m = 0.5f;
    config.surge_limit = 0.5f;
    config.sway_limit = 0.5f;
    config.yaw_limit = 0.5f;
    config.position_i_limit = 0.2f;
    config.yaw_i_limit = 0.2f;
    return config;
}

void MiniDP_Controller::init()
{
    cfg = default_config();
    reset();
}

void MiniDP_Controller::set_config(
    const MiniDP_ControllerConfig &new_config)
{
    cfg = new_config;
    cfg.yaw_p = sanitize_non_negative(cfg.yaw_p);
    cfg.yaw_i = sanitize_non_negative(cfg.yaw_i);
    cfg.yaw_d = sanitize_non_negative(cfg.yaw_d);
    cfg.position_p = sanitize_non_negative(cfg.position_p);
    cfg.position_i = sanitize_non_negative(cfg.position_i);
    cfg.velocity_d = sanitize_non_negative(cfg.velocity_d);
    cfg.position_radius_m = sanitize_non_negative(cfg.position_radius_m);
    cfg.position_deadband_m = sanitize_non_negative(cfg.position_deadband_m);
    if (cfg.position_radius_m < cfg.position_deadband_m) {
        cfg.position_radius_m = cfg.position_deadband_m;
    }
    cfg.surge_limit = sanitize_unit_positive(cfg.surge_limit, 0.5f);
    cfg.sway_limit = sanitize_unit_positive(cfg.sway_limit, 0.5f);
    cfg.yaw_limit = sanitize_unit_positive(cfg.yaw_limit, 0.5f);
    cfg.position_i_limit = sanitize_unit_positive(cfg.position_i_limit, 0.2f);
    cfg.yaw_i_limit = sanitize_unit_positive(cfg.yaw_i_limit, 0.2f);
    if (cfg.yaw_i <= 0.0f) {
        yaw_i_output = 0.0f;
    }
    if (cfg.position_i <= 0.0f) {
        position_i_n = position_i_e = 0.0f;
    }
    yaw_i_output = constrain_float_range(
        yaw_i_output,
        -cfg.yaw_i_limit,
        cfg.yaw_i_limit);
    constrain_vector_length(position_i_n, position_i_e, cfg.position_i_limit);
}

void MiniDP_Controller::reset()
{
    yaw_i_output = 0.0f;
    position_i_n = 0.0f;
    position_i_e = 0.0f;
    last_update_us = 0;
    last_target_id = 0;
    last_mode = MiniDP_Mode::MANUAL;
    have_update = false;
    output_limited = false;
}

MiniDP_AxisCommand MiniDP_Controller::update(
    const MiniDP_Mode mode,
    const MiniDP_ModeTarget &target,
    const MiniDP_State &state)
{
    float dt_s = 0.0f;
    if (!have_update ||
        mode != last_mode ||
        target.target_id != last_target_id ||
        state.time_us < last_update_us) {
        yaw_i_output = 0.0f;
        position_i_n = 0.0f;
        position_i_e = 0.0f;
    } else {
        dt_s = float(state.time_us - last_update_us) * 1.0e-6f;
        if (dt_s > max_controller_dt_s) {
            dt_s = max_controller_dt_s;
        }
    }
    last_update_us = state.time_us;
    last_target_id = target.target_id;
    last_mode = mode;
    have_update = true;

    switch (mode) {
    case MiniDP_Mode::HEADING_HOLD:
        return update_heading_hold(target, state, dt_s);
    case MiniDP_Mode::DP_HOLD:
        return update_dp_hold(target, state, dt_s);
    case MiniDP_Mode::MANUAL:
    case MiniDP_Mode::ACTUATOR_TEST:
    case MiniDP_Mode::FAILSAFE:
        break;
    }
    return {};
}

MiniDP_AxisCommand MiniDP_Controller::update_heading_hold(
    const MiniDP_ModeTarget &target,
    const MiniDP_State &state,
    const float dt_s)
{
    MiniDP_AxisCommand command{};
    if (!target.yaw_valid || !state.yaw_valid || !isfinite(target.yaw_rad) ||
        !isfinite(state.yaw_rad) || !isfinite(state.yaw_rate_rad_s)) {
        yaw_i_output = 0.0f;
        return command;
    }

    const float yaw_error = MiniDP_Math::wrap_pi(MiniDP_Math::wrap_pi(target.yaw_rad) - MiniDP_Math::wrap_pi(state.yaw_rad));
    const float yaw_pd = cfg.yaw_p * yaw_error - cfg.yaw_d * state.yaw_rate_rad_s;
    const float yaw_increment = cfg.yaw_i * yaw_error * dt_s;
    if ((!output_limited || yaw_i_output * yaw_increment < 0.0f) &&
        (fabsf(yaw_pd + yaw_i_output + yaw_increment) <= cfg.yaw_limit ||
         (yaw_pd + yaw_i_output) * yaw_increment < 0.0f)) {
        yaw_i_output += yaw_increment;
    }
    yaw_i_output = constrain_float_range(
        yaw_i_output,
        -cfg.yaw_i_limit,
        cfg.yaw_i_limit);
    command.yaw = constrain_axis(
        (cfg.yaw_p * yaw_error) +
        yaw_i_output -
        (cfg.yaw_d * state.yaw_rate_rad_s),
        cfg.yaw_limit);
    return command;
}

MiniDP_AxisCommand MiniDP_Controller::update_dp_hold(
    const MiniDP_ModeTarget &target,
    const MiniDP_State &state,
    const float dt_s)
{
    MiniDP_AxisCommand command = update_heading_hold(target, state, dt_s);
    if (!target.position_valid ||
        !state.position_valid ||
        !state.velocity_valid ||
        !state.yaw_valid || !isfinite(state.yaw_rad) || !isfinite(target.pos_n_m) || !isfinite(target.pos_e_m) ||
        !isfinite(state.pos_n_m) || !isfinite(state.pos_e_m) ||
        !isfinite(state.vel_n_m_s) || !isfinite(state.vel_e_m_s) ||
        fabsf(target.pos_n_m) > 2.0e7f || fabsf(target.pos_e_m) > 2.0e7f ||
        fabsf(state.pos_n_m) > 2.0e7f || fabsf(state.pos_e_m) > 2.0e7f) {
        position_i_n = 0.0f;
        position_i_e = 0.0f;
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
    const float pd_n = cfg.position_p * error_n - cfg.velocity_d * state.vel_n_m_s;
    const float pd_e = cfg.position_p * error_e - cfg.velocity_d * state.vel_e_m_s;
    const float raw_x = yaw_cos * (pd_n + position_i_n) + yaw_sin * (pd_e + position_i_e);
    const float raw_y = -yaw_sin * (pd_n + position_i_n) + yaw_cos * (pd_e + position_i_e);
    if (!output_limited || position_i_n * error_n + position_i_e * error_e < 0.0f) {
        float increment_x = cfg.position_i * (yaw_cos * error_n + yaw_sin * error_e) * dt_s;
        float increment_y = cfg.position_i * (-yaw_sin * error_n + yaw_cos * error_e) * dt_s;
        if (fabsf(raw_x + increment_x) > cfg.surge_limit && raw_x * increment_x >= 0.0f) {
            increment_x = 0.0f;
        }
        if (fabsf(raw_y + increment_y) > cfg.sway_limit && raw_y * increment_y >= 0.0f) {
            increment_y = 0.0f;
        }
        position_i_n += yaw_cos * increment_x - yaw_sin * increment_y;
        position_i_e += yaw_sin * increment_x + yaw_cos * increment_y;
    }
    constrain_vector_length(position_i_n, position_i_e, cfg.position_i_limit);

    const float error_surge =
        (yaw_cos * ((cfg.position_p * error_n) + position_i_n)) +
        (yaw_sin * ((cfg.position_p * error_e) + position_i_e));
    const float error_sway =
        (-yaw_sin * ((cfg.position_p * error_n) + position_i_n)) +
        (yaw_cos * ((cfg.position_p * error_e) + position_i_e));
    const float velocity_surge =
        (yaw_cos * state.vel_n_m_s) + (yaw_sin * state.vel_e_m_s);
    const float velocity_sway =
        (-yaw_sin * state.vel_n_m_s) + (yaw_cos * state.vel_e_m_s);

    const float raw_surge = error_surge - cfg.velocity_d * velocity_surge;
    const float raw_sway = error_sway - cfg.velocity_d * velocity_sway;
    command.surge = constrain_axis(
        raw_surge,
        cfg.surge_limit);
    command.sway = constrain_axis(
        raw_sway,
        cfg.sway_limit);
    return command;
}
