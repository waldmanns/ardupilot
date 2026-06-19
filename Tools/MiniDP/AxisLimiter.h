#pragma once

#include "Output.h"

#include <stdint.h>

struct MiniDP_AxisLimiterConfig {
    float deadband;
    float surge_limit;
    float sway_limit;
    float yaw_limit;
    float surge_slew_rate;
    float sway_slew_rate;
    float yaw_slew_rate;
};

class MiniDP_AxisLimiter {
public:
    void init(uint32_t now_ms);
    void set_config(const MiniDP_AxisLimiterConfig &new_config);

    const MiniDP_AxisLimiterConfig &config() const { return cfg; }
    const MiniDP_AxisCommand &command() const { return current_command; }

    MiniDP_AxisCommand update(
        uint32_t now_ms,
        const MiniDP_AxisCommand &target);
    void reset(uint32_t now_ms);
    void reset(uint32_t now_ms, const MiniDP_AxisCommand &command);

    static MiniDP_AxisLimiterConfig default_config();

private:
    MiniDP_AxisLimiterConfig cfg{};
    MiniDP_AxisCommand current_command{};
    uint32_t last_update_ms = 0;
    bool have_update = false;

    MiniDP_AxisCommand apply_limits_only(
        const MiniDP_AxisCommand &command) const;
    float apply_axis_limit(float value, float limit) const;
};
