#pragma once

#include "Output.h"

#include <stdint.h>

enum class MiniDP_ManualSource : uint8_t {
    NONE = 0,
    RC,
    MAVLINK,
};

struct MiniDP_InputConfig {
    uint8_t rc_surge_channel;
    uint8_t rc_sway_channel;
    uint8_t rc_yaw_channel;
    uint8_t rc_kill_channel;
    uint16_t rc_kill_pwm;

    float manual_deadband;
    float manual_surge_limit;
    float manual_sway_limit;
    float manual_yaw_limit;
    uint32_t mavlink_manual_timeout_ms;
};

struct MiniDP_RCInputFrame {
    bool healthy;
    uint8_t channel_count;
    float norm[16];
    uint16_t pwm[16];
};

struct MiniDP_ManualCommand {
    bool valid;
    MiniDP_ManualSource source;
    MiniDP_AxisCommand axes;
    bool kill;
    uint32_t timestamp_ms;
};

class MiniDP_InputMapper {
public:
    static constexpr uint8_t max_rc_channels = 16;

    void init();
    void set_config(const MiniDP_InputConfig &new_config);

    const MiniDP_InputConfig &config() const { return cfg; }

    MiniDP_ManualCommand map_rc(
        const MiniDP_RCInputFrame &frame,
        uint32_t now_ms) const;
    bool rc_kill_active(const MiniDP_RCInputFrame &frame) const;

    void record_mavlink_manual_control(
        uint32_t now_ms,
        int16_t x,
        int16_t y,
        int16_t r);
    MiniDP_ManualCommand mavlink_manual_command(uint32_t now_ms) const;

    static MiniDP_InputConfig default_config();
    static const char *source_name(MiniDP_ManualSource source);

private:
    MiniDP_InputConfig cfg{};
    MiniDP_AxisCommand last_mavlink_axes{};
    uint32_t last_mavlink_manual_ms = 0;
    bool have_mavlink_manual = false;

    MiniDP_AxisCommand apply_manual_limits(
        const MiniDP_AxisCommand &axes) const;
};
