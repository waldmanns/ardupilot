#pragma once

#include "Mode.h"
#include "Output.h"
#include "State.h"

struct MiniDP_ControllerConfig {
    float yaw_p;
    float yaw_d;
    float position_p;
    float velocity_d;
    float position_radius_m;
    float position_deadband_m;
    float surge_limit;
    float sway_limit;
    float yaw_limit;
};

class MiniDP_Controller {
public:
    void init();
    void set_config(const MiniDP_ControllerConfig &new_config);

    const MiniDP_ControllerConfig &config() const { return cfg; }

    MiniDP_AxisCommand update(
        MiniDP_Mode mode,
        const MiniDP_ModeTarget &target,
        const MiniDP_State &state) const;

    static MiniDP_ControllerConfig default_config();

private:
    MiniDP_ControllerConfig cfg{};

    MiniDP_AxisCommand update_heading_hold(
        const MiniDP_ModeTarget &target,
        const MiniDP_State &state) const;
    MiniDP_AxisCommand update_dp_hold(
        const MiniDP_ModeTarget &target,
        const MiniDP_State &state) const;
};
