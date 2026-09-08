#include "Input.h"

#include <limits.h>
#include <math.h>

namespace {

float absf(const float value)
{
    return value < 0.0f ? -value : value;
}

float constrain_unit(const float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

uint8_t sanitize_channel(const uint8_t channel)
{
    return channel <= MiniDP_InputMapper::max_rc_channels ? channel : 0U;
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

float apply_deadband_and_limit(
    const float value,
    const float deadband,
    const float limit)
{
    float constrained = constrain_unit(value);
    if (absf(constrained) < deadband) {
        constrained = 0.0f;
    }
    return constrained * limit;
}

bool read_rc_axis(
    const MiniDP_RCInputFrame &frame,
    const uint8_t channel,
    float &axis)
{
    axis = 0.0f;
    if (channel == 0U) {
        return true;
    }
    if (channel > frame.channel_count ||
        channel > MiniDP_InputMapper::max_rc_channels ||
        (frame.valid_mask & (1U << (channel - 1U))) == 0) {
        return false;
    }
    axis = frame.norm[channel - 1U];
    return true;
}

float mavlink_axis_to_unit(const int16_t value)
{
    if (value == INT16_MAX) {
        return 0.0f;
    }
    if (value > 1000) {
        return 1.0f;
    }
    if (value < -1000) {
        return -1.0f;
    }
    return float(value) * 0.001f;
}

bool rc_switch_high(
    const MiniDP_RCInputFrame &frame,
    const uint8_t channel,
    const uint16_t threshold_pwm)
{
    if (!frame.healthy ||
        channel == 0U ||
        channel > frame.channel_count ||
        channel > MiniDP_InputMapper::max_rc_channels ||
        (frame.valid_mask & (1U << (channel - 1U))) == 0) {
        return false;
    }

    return frame.pwm[channel - 1U] >= threshold_pwm;
}

bool rc_switch_low(
    const MiniDP_RCInputFrame &frame,
    const uint8_t channel,
    const uint16_t threshold_pwm)
{
    if (!frame.healthy ||
        channel == 0U ||
        channel > frame.channel_count ||
        channel > MiniDP_InputMapper::max_rc_channels ||
        (frame.valid_mask & (1U << (channel - 1U))) == 0) {
        return false;
    }

    return frame.pwm[channel - 1U] <= threshold_pwm;
}

} // namespace

MiniDP_InputConfig MiniDP_InputMapper::default_config()
{
    MiniDP_InputConfig config{};
    config.rc_surge_channel = 2U;
    config.rc_sway_channel = 1U;
    config.rc_yaw_channel = 4U;
    config.rc_kill_channel = 0U;
    config.rc_kill_pwm = 1800U;
    config.rc_arm_channel = 0U;
    config.rc_arm_pwm = 1800U;
    config.rc_disarm_channel = 0U;
    config.rc_disarm_pwm = 1200U;
    config.manual_deadband = 0.03f;
    config.manual_surge_limit = 0.5f;
    config.manual_sway_limit = 0.5f;
    config.manual_yaw_limit = 0.5f;
    config.mavlink_manual_timeout_ms = 500U;
    return config;
}

void MiniDP_InputMapper::init()
{
    cfg = default_config();
    last_mavlink_axes = {};
    last_mavlink_manual_ms = 0;
    have_mavlink_manual = false;
}

void MiniDP_InputMapper::set_config(const MiniDP_InputConfig &new_config)
{
    cfg = new_config;
    cfg.rc_surge_channel = sanitize_channel(cfg.rc_surge_channel);
    cfg.rc_sway_channel = sanitize_channel(cfg.rc_sway_channel);
    cfg.rc_yaw_channel = sanitize_channel(cfg.rc_yaw_channel);
    cfg.rc_kill_channel = sanitize_channel(cfg.rc_kill_channel);
    cfg.rc_arm_channel = sanitize_channel(cfg.rc_arm_channel);
    cfg.rc_disarm_channel = sanitize_channel(cfg.rc_disarm_channel);
    if (cfg.rc_kill_pwm < 800U || cfg.rc_kill_pwm > 2200U) {
        cfg.rc_kill_pwm = 1800U;
    }
    if (cfg.rc_arm_pwm < 800U || cfg.rc_arm_pwm > 2200U) {
        cfg.rc_arm_pwm = 1800U;
    }
    if (cfg.rc_disarm_pwm < 800U || cfg.rc_disarm_pwm > 2200U) {
        cfg.rc_disarm_pwm = 1200U;
    }

    cfg.manual_deadband =
        sanitize_unit_positive(cfg.manual_deadband, 0.03f);
    cfg.manual_surge_limit =
        sanitize_unit_positive(cfg.manual_surge_limit, 0.5f);
    cfg.manual_sway_limit =
        sanitize_unit_positive(cfg.manual_sway_limit, 0.5f);
    cfg.manual_yaw_limit =
        sanitize_unit_positive(cfg.manual_yaw_limit, 0.5f);

    if (cfg.mavlink_manual_timeout_ms < 100U) {
        cfg.mavlink_manual_timeout_ms = 100U;
    } else if (cfg.mavlink_manual_timeout_ms > 5000U) {
        cfg.mavlink_manual_timeout_ms = 5000U;
    }
}

MiniDP_AxisCommand MiniDP_InputMapper::apply_manual_limits(
    const MiniDP_AxisCommand &axes) const
{
    MiniDP_AxisCommand limited{};
    limited.surge = apply_deadband_and_limit(
        axes.surge,
        cfg.manual_deadband,
        cfg.manual_surge_limit);
    limited.sway = apply_deadband_and_limit(
        axes.sway,
        cfg.manual_deadband,
        cfg.manual_sway_limit);
    limited.yaw = apply_deadband_and_limit(
        axes.yaw,
        cfg.manual_deadband,
        cfg.manual_yaw_limit);
    return limited;
}

bool MiniDP_InputMapper::rc_kill_active(
    const MiniDP_RCInputFrame &frame) const
{
    return rc_switch_high(frame, cfg.rc_kill_channel, cfg.rc_kill_pwm);
}

bool MiniDP_InputMapper::rc_arm_active(
    const MiniDP_RCInputFrame &frame) const
{
    return rc_switch_high(frame, cfg.rc_arm_channel, cfg.rc_arm_pwm);
}

bool MiniDP_InputMapper::rc_disarm_active(
    const MiniDP_RCInputFrame &frame) const
{
    return rc_switch_low(frame, cfg.rc_disarm_channel, cfg.rc_disarm_pwm);
}

MiniDP_ManualCommand MiniDP_InputMapper::map_rc(
    const MiniDP_RCInputFrame &frame,
    const uint32_t now_ms) const
{
    MiniDP_ManualCommand command{};
    command.source = MiniDP_ManualSource::RC;
    command.timestamp_ms = now_ms;
    command.kill = rc_kill_active(frame);

    if (!frame.healthy || command.kill) {
        return command;
    }

    MiniDP_AxisCommand axes{};
    if (!read_rc_axis(frame, cfg.rc_surge_channel, axes.surge) ||
        !read_rc_axis(frame, cfg.rc_sway_channel, axes.sway) ||
        !read_rc_axis(frame, cfg.rc_yaw_channel, axes.yaw)) {
        return command;
    }

    command.valid = true;
    command.axes = apply_manual_limits(axes);
    return command;
}

void MiniDP_InputMapper::record_mavlink_manual_control(
    const uint32_t now_ms,
    const int16_t x,
    const int16_t y,
    const int16_t r)
{
    last_mavlink_axes.surge = mavlink_axis_to_unit(x);
    last_mavlink_axes.sway = mavlink_axis_to_unit(y);
    last_mavlink_axes.yaw = mavlink_axis_to_unit(r);
    last_mavlink_manual_ms = now_ms;
    have_mavlink_manual = true;
}

MiniDP_ManualCommand MiniDP_InputMapper::mavlink_manual_command(
    const uint32_t now_ms) const
{
    MiniDP_ManualCommand command{};
    command.source = MiniDP_ManualSource::MAVLINK;
    command.timestamp_ms = last_mavlink_manual_ms;

    if (!have_mavlink_manual ||
        now_ms - last_mavlink_manual_ms > cfg.mavlink_manual_timeout_ms) {
        return command;
    }

    command.valid = true;
    command.axes = apply_manual_limits(last_mavlink_axes);
    return command;
}

const char *MiniDP_InputMapper::source_name(const MiniDP_ManualSource source)
{
    switch (source) {
    case MiniDP_ManualSource::NONE:
        return "NONE";
    case MiniDP_ManualSource::RC:
        return "RC";
    case MiniDP_ManualSource::MAVLINK:
        return "MAVLINK";
    }
    return "UNKNOWN";
}
