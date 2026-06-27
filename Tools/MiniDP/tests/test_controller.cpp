#include <AP_gtest.h>

#include "Controller.h"

namespace {

MiniDP_State valid_state()
{
    MiniDP_State state{};
    state.time_us = 1000U;
    state.yaw_valid = true;
    state.position_valid = true;
    state.velocity_valid = true;
    state.yaw_rad = 0.0f;
    return state;
}

MiniDP_ModeTarget valid_target()
{
    MiniDP_ModeTarget target{};
    target.yaw_valid = true;
    target.position_valid = true;
    target.target_id = 1U;
    return target;
}

} // namespace

TEST(MiniDPController, DefaultsProduceQuietCommands)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_State state = valid_state();
    state.yaw_rad = -1.0f;
    state.pos_n_m = -5.0f;
    MiniDP_ModeTarget target = valid_target();
    target.yaw_rad = 1.0f;
    target.pos_n_m = 5.0f;

    const MiniDP_AxisCommand command =
        controller.update(MiniDP_Mode::DP_HOLD, target, state);

    EXPECT_FLOAT_EQ(command.surge, 0.0f);
    EXPECT_FLOAT_EQ(command.sway, 0.0f);
    EXPECT_FLOAT_EQ(command.yaw, 0.0f);
}

TEST(MiniDPController, HeadingHoldWrapsYawErrorAndDampsRate)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_ControllerConfig config = controller.config();
    config.yaw_p = 1.0f;
    config.yaw_d = 0.5f;
    config.yaw_limit = 0.75f;
    controller.set_config(config);

    MiniDP_State state = valid_state();
    state.yaw_rad = 3.0f;
    state.yaw_rate_rad_s = 0.2f;
    MiniDP_ModeTarget target = valid_target();
    target.yaw_rad = -3.0f;

    const MiniDP_AxisCommand command =
        controller.update(MiniDP_Mode::HEADING_HOLD, target, state);

    EXPECT_FLOAT_EQ(command.surge, 0.0f);
    EXPECT_FLOAT_EQ(command.sway, 0.0f);
    EXPECT_NEAR(command.yaw, 0.18318534f, 1.0e-6f);
}

TEST(MiniDPController, HeadingHoldClampsYawCommand)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_ControllerConfig config = controller.config();
    config.yaw_p = 10.0f;
    config.yaw_limit = 0.25f;
    controller.set_config(config);

    MiniDP_State state = valid_state();
    MiniDP_ModeTarget target = valid_target();
    target.yaw_rad = 1.0f;

    const MiniDP_AxisCommand command =
        controller.update(MiniDP_Mode::HEADING_HOLD, target, state);

    EXPECT_FLOAT_EQ(command.yaw, 0.25f);
}

TEST(MiniDPController, HeadingHoldIntegratesYawErrorWithLimit)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_ControllerConfig config = controller.config();
    config.yaw_i = 0.1f;
    config.yaw_i_limit = 0.15f;
    config.yaw_limit = 1.0f;
    controller.set_config(config);

    MiniDP_State state = valid_state();
    MiniDP_ModeTarget target = valid_target();
    target.yaw_rad = 1.0f;

    MiniDP_AxisCommand command =
        controller.update(MiniDP_Mode::HEADING_HOLD, target, state);
    EXPECT_FLOAT_EQ(command.yaw, 0.0f);

    state.time_us += 1000000U;
    command = controller.update(MiniDP_Mode::HEADING_HOLD, target, state);
    EXPECT_NEAR(command.yaw, 0.1f, 1.0e-6f);

    state.time_us += 1000000U;
    command = controller.update(MiniDP_Mode::HEADING_HOLD, target, state);
    EXPECT_NEAR(command.yaw, 0.15f, 1.0e-6f);
}

TEST(MiniDPController, DPHoldTransformsPositionAndVelocityToBodyAxes)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_ControllerConfig config = controller.config();
    config.position_p = 0.1f;
    config.velocity_d = 0.2f;
    config.surge_limit = 1.0f;
    config.sway_limit = 1.0f;
    controller.set_config(config);

    MiniDP_State state = valid_state();
    state.yaw_rad = 1.57079632679f;
    state.pos_n_m = 0.0f;
    state.pos_e_m = 0.0f;
    state.vel_n_m_s = 0.0f;
    state.vel_e_m_s = 0.0f;

    MiniDP_ModeTarget target = valid_target();
    target.pos_n_m = 2.0f;
    target.pos_e_m = 0.0f;

    const MiniDP_AxisCommand command =
        controller.update(MiniDP_Mode::DP_HOLD, target, state);

    EXPECT_NEAR(command.surge, 0.0f, 1.0e-6f);
    EXPECT_NEAR(command.sway, -0.2f, 1.0e-6f);
}

TEST(MiniDPController, DPHoldSoftensPositionInsideConfiguredRadius)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_ControllerConfig config = controller.config();
    config.position_p = 1.0f;
    config.velocity_d = 0.0f;
    config.position_radius_m = 2.0f;
    config.position_deadband_m = 0.5f;
    config.surge_limit = 1.0f;
    config.sway_limit = 1.0f;
    controller.set_config(config);

    MiniDP_State state = valid_state();
    state.pos_n_m = 0.0f;
    state.pos_e_m = 0.0f;
    state.vel_n_m_s = 0.0f;
    state.vel_e_m_s = 0.0f;

    MiniDP_ModeTarget target = valid_target();
    target.pos_n_m = 1.0f;

    const MiniDP_AxisCommand softened =
        controller.update(MiniDP_Mode::DP_HOLD, target, state);
    EXPECT_NEAR(softened.surge, 0.33333334f, 1.0e-6f);

    target.pos_n_m = 0.25f;
    const MiniDP_AxisCommand deadbanded =
        controller.update(MiniDP_Mode::DP_HOLD, target, state);
    EXPECT_FLOAT_EQ(deadbanded.surge, 0.0f);
}

TEST(MiniDPController, DPHoldIntegratesPositionErrorWithVectorLimit)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_ControllerConfig config = controller.config();
    config.position_p = 0.0f;
    config.position_i = 0.1f;
    config.position_i_limit = 0.25f;
    config.surge_limit = 1.0f;
    config.sway_limit = 1.0f;
    controller.set_config(config);

    MiniDP_State state = valid_state();
    state.pos_n_m = 0.0f;
    state.pos_e_m = 0.0f;
    state.vel_n_m_s = 0.0f;
    state.vel_e_m_s = 0.0f;

    MiniDP_ModeTarget target = valid_target();
    target.pos_n_m = 2.0f;
    target.pos_e_m = 0.0f;

    MiniDP_AxisCommand command =
        controller.update(MiniDP_Mode::DP_HOLD, target, state);
    EXPECT_FLOAT_EQ(command.surge, 0.0f);

    state.time_us += 1000000U;
    command = controller.update(MiniDP_Mode::DP_HOLD, target, state);
    EXPECT_NEAR(command.surge, 0.2f, 1.0e-6f);

    state.time_us += 1000000U;
    command = controller.update(MiniDP_Mode::DP_HOLD, target, state);
    EXPECT_NEAR(command.surge, 0.25f, 1.0e-6f);
}

TEST(MiniDPController, IntegratorsResetWhenTargetChanges)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_ControllerConfig config = controller.config();
    config.yaw_i = 0.1f;
    config.position_i = 0.1f;
    config.yaw_i_limit = 0.5f;
    config.position_i_limit = 0.5f;
    config.surge_limit = 1.0f;
    config.yaw_limit = 1.0f;
    controller.set_config(config);

    MiniDP_State state = valid_state();
    MiniDP_ModeTarget target = valid_target();
    target.yaw_rad = 1.0f;
    target.pos_n_m = 2.0f;

    (void)controller.update(MiniDP_Mode::DP_HOLD, target, state);
    state.time_us += 1000000U;
    MiniDP_AxisCommand command =
        controller.update(MiniDP_Mode::DP_HOLD, target, state);
    EXPECT_GT(command.surge, 0.0f);
    EXPECT_GT(command.yaw, 0.0f);

    state.time_us += 1000000U;
    target.target_id++;
    command = controller.update(MiniDP_Mode::DP_HOLD, target, state);
    EXPECT_FLOAT_EQ(command.surge, 0.0f);
    EXPECT_FLOAT_EQ(command.yaw, 0.0f);
}

TEST(MiniDPController, MissingStateZerosOnlyUnavailableAxes)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_ControllerConfig config = controller.config();
    config.yaw_p = 0.5f;
    config.position_p = 0.5f;
    controller.set_config(config);

    MiniDP_State state = valid_state();
    state.position_valid = false;
    MiniDP_ModeTarget target = valid_target();
    target.yaw_rad = 0.5f;
    target.pos_n_m = 1.0f;

    const MiniDP_AxisCommand command =
        controller.update(MiniDP_Mode::DP_HOLD, target, state);

    EXPECT_FLOAT_EQ(command.surge, 0.0f);
    EXPECT_FLOAT_EQ(command.sway, 0.0f);
    EXPECT_FLOAT_EQ(command.yaw, 0.25f);
}

TEST(MiniDPController, SanitizesInvalidConfig)
{
    MiniDP_Controller controller;
    controller.init();

    MiniDP_ControllerConfig config = controller.config();
    config.yaw_p = -1.0f;
    config.yaw_i = -1.5f;
    config.yaw_d = -2.0f;
    config.position_p = -3.0f;
    config.position_i = -3.5f;
    config.velocity_d = -4.0f;
    config.position_radius_m = -5.0f;
    config.position_deadband_m = 3.0f;
    config.surge_limit = -1.0f;
    config.sway_limit = 2.0f;
    config.yaw_limit = 0.25f;
    config.position_i_limit = NAN;
    config.yaw_i_limit = -1.0f;
    controller.set_config(config);

    EXPECT_FLOAT_EQ(controller.config().yaw_p, 0.0f);
    EXPECT_FLOAT_EQ(controller.config().yaw_i, 0.0f);
    EXPECT_FLOAT_EQ(controller.config().yaw_d, 0.0f);
    EXPECT_FLOAT_EQ(controller.config().position_p, 0.0f);
    EXPECT_FLOAT_EQ(controller.config().position_i, 0.0f);
    EXPECT_FLOAT_EQ(controller.config().velocity_d, 0.0f);
    EXPECT_FLOAT_EQ(controller.config().position_radius_m, 3.0f);
    EXPECT_FLOAT_EQ(controller.config().position_deadband_m, 3.0f);
    EXPECT_FLOAT_EQ(controller.config().surge_limit, 0.0f);
    EXPECT_FLOAT_EQ(controller.config().sway_limit, 1.0f);
    EXPECT_FLOAT_EQ(controller.config().yaw_limit, 0.25f);
    EXPECT_FLOAT_EQ(controller.config().position_i_limit, 0.2f);
    EXPECT_FLOAT_EQ(controller.config().yaw_i_limit, 0.0f);
}

AP_GTEST_MAIN()
