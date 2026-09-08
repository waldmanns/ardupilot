#include <AP_gtest.h>
#include "Controller.h"
#include "ControlMath.h"
#include "Output.h"

// Deterministic planar vessel model. Forces come from the allocated thrusters,
// including steering rate and reverse folding, rather than controller axes.
// Unit mass/inertia and linear drag make this a control regression, not a hull
// identification or a prediction of real vessel performance.
static void stationkeeping(const int16_t frame_type)
{
    MiniDP_Controller controller;
    controller.init();
    auto cfg = controller.config();
    cfg.position_p = 0.15f;
    cfg.position_i = 0.015f;
    cfg.velocity_d = 0.8f;
    cfg.yaw_p = 0.6f;
    cfg.yaw_i = 0.015f;
    cfg.yaw_d = 0.8f;
    cfg.position_deadband_m = 0.05f;
    cfg.position_radius_m = 0.2f;
    controller.set_config(cfg);
    MiniDP_OutputManager output;
    output.init(frame_type);
    MiniDP_State state{};
    state.position_valid = state.velocity_valid = state.yaw_valid = true;
    state.pos_n_m = 8;
    state.pos_e_m = -5;
    state.yaw_rad = 0.6f;
    MiniDP_ModeTarget target{};
    target.position_valid = target.yaw_valid = true;
    target.target_id = 1;
    constexpr float dt = 0.02f;
    for (unsigned step = 0; step < 12000; step++) {
        state.time_us += 20000;
        const auto cmd = controller.update(MiniDP_Mode::DP_HOLD, target, state);
        const auto &f = output.update(MiniDP_OutputState::ARMED_ACTIVE, cmd, dt);
        controller.set_output_limited(f.saturated);
        float fx, fy, torque;
        if (frame_type == 901) {
            fx = f.actuator[0].demand + f.actuator[1].demand;
            fy = f.actuator[2].demand + f.actuator[3].demand;
            torque = f.actuator[0].demand - f.actuator[1].demand +
                f.actuator[2].demand - f.actuator[3].demand;
        } else {
            fx = f.thruster[0].force_x + f.thruster[1].force_x;
            const float aft_y = f.thruster[0].force_y + f.thruster[1].force_y;
            fy = aft_y + f.thruster[2].force_y;
            torque = -aft_y + f.thruster[2].force_y +
                f.thruster[0].force_x - f.thruster[1].force_x;
        }
        const float c = cosf(state.yaw_rad), s = sinf(state.yaw_rad);
        // Constant external force and moment require integral compensation.
        state.vel_n_m_s += (c * fx - s * fy + 0.03f - 0.4f * state.vel_n_m_s) * dt;
        state.vel_e_m_s += (s * fx + c * fy - 0.02f - 0.4f * state.vel_e_m_s) * dt;
        state.yaw_rate_rad_s += (torque + 0.01f - 0.5f * state.yaw_rate_rad_s) * dt;
        state.pos_n_m += state.vel_n_m_s * dt;
        state.pos_e_m += state.vel_e_m_s * dt;
        state.yaw_rad = MiniDP_Math::wrap_pi(state.yaw_rad + state.yaw_rate_rad_s * dt);
        ASSERT_TRUE(isfinite(state.pos_n_m));
        ASSERT_LT(hypotf(state.pos_n_m, state.pos_e_m), 15);
        if (step > 10000) {
            EXPECT_LT(hypotf(state.pos_n_m, state.pos_e_m), 0.3f);
            EXPECT_LT(fabsf(state.yaw_rad), 0.05f);
            EXPECT_LT(hypotf(state.vel_n_m_s, state.vel_e_m_s), 0.05f);
        }
    }
}

TEST(MiniDPDynamics, Frame901RejectsSteadyDisturbance) { stationkeeping(901); }
TEST(MiniDPDynamics, Frame902RejectsSteadyDisturbance) { stationkeeping(902); }

AP_GTEST_MAIN()
