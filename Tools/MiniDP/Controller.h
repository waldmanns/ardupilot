#pragma once

#include "Mode.h"
#include "Output.h"
#include "State.h"

struct MiniDP_ControllerConfig {
    float yaw_p;
    float yaw_i;
    float yaw_d;
    float position_p;
    float position_i;
    float velocity_d;
    float position_radius_m;
    float position_deadband_m;
    float surge_limit;
    float sway_limit;
    float yaw_limit;
    float position_i_limit;
    float yaw_i_limit;
};

class MiniDP_Controller {
public:
    void init();
    void set_config(const MiniDP_ControllerConfig &new_config);

    const MiniDP_ControllerConfig &config() const { return cfg; }

    MiniDP_AxisCommand update(
        MiniDP_Mode mode,
        const MiniDP_ModeTarget &target,
        const MiniDP_State &state);
    void reset();
    void set_output_limited(bool limited) { output_limited = limited; }

    static MiniDP_ControllerConfig default_config();

private:
    MiniDP_ControllerConfig cfg{};
    float yaw_i_output = 0.0f;
    float position_i_n = 0.0f;
    float position_i_e = 0.0f;
    uint64_t last_update_us = 0;
    uint32_t last_target_id = 0;
    MiniDP_Mode last_mode = MiniDP_Mode::MANUAL;
    bool have_update = false;
    bool output_limited = false;

    MiniDP_AxisCommand update_heading_hold(
        const MiniDP_ModeTarget &target,
        const MiniDP_State &state,
        float dt_s);
    MiniDP_AxisCommand update_dp_hold(
        const MiniDP_ModeTarget &target,
        const MiniDP_State &state,
        float dt_s);
};
