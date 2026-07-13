#include <AP_gtest.h>

#include "Output.h"

TEST(MiniDPOutput, DefaultsStartElectricallyQuiet)
{
    MiniDP_OutputManager output;
    output.init();

    const MiniDP_OutputFrame &frame = output.frame();
    EXPECT_EQ(output.frame_type(), MiniDP_OutputManager::default_frame_type);
    EXPECT_STREQ(
        MiniDP_OutputManager::frame_type_name(output.frame_type()),
        "OMNI_PLUS");
    EXPECT_EQ(frame.state, MiniDP_OutputState::DISARMED);
    EXPECT_EQ(frame.active_pwm_count, 0U);
    for (uint8_t i = 0; i < MiniDP_OutputManager::max_actuators; i++) {
        EXPECT_FALSE(frame.actuator[i].pwm_enabled);
    }
}

TEST(MiniDPOutput, InvalidFrameTypeFallsBackToOmniPlus901)
{
    MiniDP_OutputManager output;
    output.init(123);

    EXPECT_EQ(output.frame_type(), int16_t(MiniDP_FrameType::OMNI_PLUS));
    EXPECT_FALSE(output.set_frame_type(123));
    EXPECT_EQ(output.frame_type(), int16_t(MiniDP_FrameType::OMNI_PLUS));
}

TEST(MiniDPOutput, NeutralAndFailsafeUseTrimByDefault)
{
    MiniDP_OutputManager output;
    output.init();

    MiniDP_AxisCommand zero{};
    const MiniDP_OutputFrame &neutral =
        output.update(MiniDP_OutputState::ARMED_NEUTRAL, zero);
    EXPECT_EQ(neutral.active_pwm_count, 4U);
    for (uint8_t i = 0; i < 4; i++) {
        EXPECT_TRUE(neutral.actuator[i].pwm_enabled);
        EXPECT_EQ(neutral.actuator[i].pwm_us, 1500U);
    }
    EXPECT_FALSE(neutral.actuator[4].pwm_enabled);
    EXPECT_FALSE(neutral.actuator[5].pwm_enabled);

    const MiniDP_OutputFrame &failsafe =
        output.update(MiniDP_OutputState::FAILSAFE, zero);
    EXPECT_EQ(failsafe.active_pwm_count, 4U);
    for (uint8_t i = 0; i < 4; i++) {
        EXPECT_TRUE(failsafe.actuator[i].pwm_enabled);
        EXPECT_EQ(failsafe.actuator[i].pwm_us, 1500U);
    }
}

TEST(MiniDPOutput, KillDisablesConfiguredActuatorsByDefault)
{
    MiniDP_OutputManager output;
    output.init();

    MiniDP_AxisCommand zero{};
    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::KILL, zero);
    EXPECT_EQ(frame.active_pwm_count, 0U);
    for (uint8_t i = 0; i < MiniDP_OutputManager::max_actuators; i++) {
        EXPECT_FALSE(frame.actuator[i].pwm_enabled);
    }
}

TEST(MiniDPOutput, Frame901OmniPlusMapsMotor1ToMotor4)
{
    MiniDP_OutputManager output;
    output.init(int16_t(MiniDP_FrameType::OMNI_PLUS));

    EXPECT_STREQ(MiniDP_OutputManager::motor_name(0), "Motor1");
    EXPECT_STREQ(MiniDP_OutputManager::motor_name(1), "Motor2");
    EXPECT_STREQ(MiniDP_OutputManager::motor_name(2), "Motor3");
    EXPECT_STREQ(MiniDP_OutputManager::motor_name(3), "Motor4");
    EXPECT_TRUE(output.actuator_config(0).enabled);
    EXPECT_TRUE(output.actuator_config(1).enabled);
    EXPECT_TRUE(output.actuator_config(2).enabled);
    EXPECT_TRUE(output.actuator_config(3).enabled);
    EXPECT_FALSE(output.actuator_config(4).enabled);
    EXPECT_FALSE(output.actuator_config(5).enabled);

    MiniDP_AxisCommand command{};
    command.surge = 0.50f;
    command.sway = -0.25f;
    command.yaw = 0.25f;

    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);

    ASSERT_FALSE(frame.saturated);
    EXPECT_FLOAT_EQ(frame.actuator[0].demand, 0.75f);
    EXPECT_FLOAT_EQ(frame.actuator[1].demand, 0.25f);
    EXPECT_FLOAT_EQ(frame.actuator[2].demand, 0.0f);
    EXPECT_FLOAT_EQ(frame.actuator[3].demand, -0.50f);
    EXPECT_EQ(frame.actuator[0].pwm_us, 1800U);
    EXPECT_EQ(frame.actuator[1].pwm_us, 1600U);
    EXPECT_EQ(frame.actuator[2].pwm_us, 1500U);
    EXPECT_EQ(frame.actuator[3].pwm_us, 1300U);
}

TEST(MiniDPOutput, Frame901ReportsFourFixedThrusters)
{
    MiniDP_OutputManager output;
    output.init(int16_t(MiniDP_FrameType::OMNI_PLUS));

    MiniDP_AxisCommand command{};
    command.surge = 0.50f;
    command.sway = -0.25f;
    command.yaw = 0.25f;

    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);

    ASSERT_EQ(frame.configured_thruster_count, 4U);
    for (uint8_t i = 0; i < 4U; i++) {
        const MiniDP_ThrusterOutput &thruster = frame.thruster[i];
        EXPECT_TRUE(thruster.configured);
        EXPECT_EQ(thruster.type, MiniDP_ThrusterType::FIXED);
        EXPECT_EQ(thruster.output1, i);
        EXPECT_EQ(thruster.output2, UINT8_MAX);
        EXPECT_FLOAT_EQ(thruster.value, frame.actuator[i].demand);
        EXPECT_FLOAT_EQ(thruster.value2, 0.0f);
        EXPECT_EQ(thruster.pwm1_us, frame.actuator[i].pwm_us);
        EXPECT_EQ(thruster.pwm2_us, 0U);
    }
}

TEST(MiniDPOutput, Frame901ScrewPositionAdjustsDifferentialYaw)
{
    MiniDP_OutputManager output;
    output.init(int16_t(MiniDP_FrameType::OMNI_PLUS));

    MiniDP_AxisCommand command{};
    command.yaw = 0.5f;

    const MiniDP_OutputFrame &aft =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);
    EXPECT_FLOAT_EQ(aft.actuator[0].demand, 0.5f);
    EXPECT_FLOAT_EQ(aft.actuator[1].demand, -0.5f);

    MiniDP_FrameGeometryConfig geometry =
        MiniDP_OutputManager::default_frame_geometry_config();
    geometry.screw_position = MiniDP_ScrewPosition::FORWARD;
    output.set_frame_geometry(geometry);
    const MiniDP_OutputFrame &forward =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);
    EXPECT_FLOAT_EQ(forward.actuator[0].demand, -0.5f);
    EXPECT_FLOAT_EQ(forward.actuator[1].demand, 0.5f);

    geometry.screw_position = MiniDP_ScrewPosition::CENTER;
    output.set_frame_geometry(geometry);
    const MiniDP_OutputFrame &center =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);
    EXPECT_FLOAT_EQ(center.actuator[0].demand, 0.0f);
    EXPECT_FLOAT_EQ(center.actuator[1].demand, 0.0f);

    geometry.screw_position = MiniDP_ScrewPosition::AFT;
    geometry.screw_yaw_scale = 0.5f;
    output.set_frame_geometry(geometry);
    const MiniDP_OutputFrame &scaled =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);
    EXPECT_FLOAT_EQ(scaled.actuator[0].demand, 0.25f);
    EXPECT_FLOAT_EQ(scaled.actuator[1].demand, -0.25f);
}

TEST(MiniDPOutput, Frame902DualAzimuthBowMapsMotor1ToMotor5)
{
    MiniDP_OutputManager output;
    output.init(int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW));

    EXPECT_EQ(output.frame_type(), int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW));
    EXPECT_STREQ(
        MiniDP_OutputManager::frame_type_name(output.frame_type()),
        "DUAL_AZ_180_BOW");
    for (uint8_t i = 0; i < 5; i++) {
        EXPECT_TRUE(output.actuator_config(i).enabled);
    }
    EXPECT_FALSE(output.actuator_config(5).enabled);

    const MiniDP_OutputFrame &safe = output.frame();
    EXPECT_EQ(safe.configured_thruster_count, 3U);
    EXPECT_EQ(safe.thruster[0].type, MiniDP_ThrusterType::AZIMUTH_180);
    EXPECT_EQ(safe.thruster[0].output1, 0U);
    EXPECT_EQ(safe.thruster[0].output2, 1U);
    EXPECT_EQ(safe.thruster[1].type, MiniDP_ThrusterType::AZIMUTH_180);
    EXPECT_EQ(safe.thruster[1].output1, 2U);
    EXPECT_EQ(safe.thruster[1].output2, 3U);
    EXPECT_EQ(safe.thruster[2].type, MiniDP_ThrusterType::FIXED);
    EXPECT_EQ(safe.thruster[2].output1, 4U);
}

TEST(MiniDPOutput, Frame902AzipodParametersLimitEachPodIndependently)
{
    MiniDP_OutputManager output;
    output.init(int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW));

    MiniDP_AzipodConfig config{};
    config.angle_min_rad = -0.7853982f;
    config.angle_max_rad = 0.7853982f;
    config.allow_reverse_fold = false;
    ASSERT_TRUE(output.set_azipod_config(0U, config));

    MiniDP_AxisCommand command{};
    command.sway = 1.0f;
    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);

    EXPECT_NEAR(frame.thruster[0].value2, 0.7853982f, 0.0001f);
    EXPECT_TRUE(
        (frame.thruster[0].flags &
         MiniDP_OutputManager::thruster_flag_angle_limited) != 0U);
    EXPECT_NEAR(frame.actuator[1].demand, 0.5f, 0.0001f);
    EXPECT_NEAR(frame.thruster[1].value2, 1.5707963f, 0.0001f);
    EXPECT_FALSE(
        (frame.thruster[1].flags &
         MiniDP_OutputManager::thruster_flag_angle_limited) != 0U);
}

TEST(MiniDPOutput, Frame902AzipodSpanIsLimitedTo180Degrees)
{
    MiniDP_OutputManager output;
    output.init(int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW));

    MiniDP_AzipodConfig config{};
    config.angle_min_rad = -3.1415927f;
    config.angle_max_rad = 3.1415927f;
    config.allow_reverse_fold = true;
    ASSERT_TRUE(output.set_azipod_config(0U, config));

    const MiniDP_AzipodConfig &sanitized = output.azipod_config(0U);
    EXPECT_NEAR(sanitized.angle_min_rad, -1.5707963f, 0.0001f);
    EXPECT_NEAR(sanitized.angle_max_rad, 1.5707963f, 0.0001f);
    EXPECT_TRUE(sanitized.allow_reverse_fold);
    EXPECT_FALSE(output.set_azipod_config(2U, config));
}

TEST(MiniDPOutput, Frame902ForwardSurgeSplitsAcrossAftPods)
{
    MiniDP_OutputManager output;
    output.init(int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW));

    MiniDP_AxisCommand command{};
    command.surge = 1.0f;

    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);

    EXPECT_FALSE(frame.saturated);
    EXPECT_EQ(frame.active_pwm_count, 5U);
    EXPECT_FLOAT_EQ(frame.actuator[0].demand, 0.5f);
    EXPECT_FLOAT_EQ(frame.actuator[1].demand, 0.0f);
    EXPECT_FLOAT_EQ(frame.actuator[2].demand, 0.5f);
    EXPECT_FLOAT_EQ(frame.actuator[3].demand, 0.0f);
    EXPECT_FLOAT_EQ(frame.actuator[4].demand, 0.0f);
    EXPECT_EQ(frame.actuator[0].pwm_us, 1700U);
    EXPECT_EQ(frame.actuator[1].pwm_us, 1500U);
    EXPECT_EQ(frame.actuator[2].pwm_us, 1700U);
    EXPECT_EQ(frame.actuator[3].pwm_us, 1500U);
    EXPECT_EQ(frame.actuator[4].pwm_us, 1500U);

    EXPECT_FLOAT_EQ(frame.thruster[0].value, 0.5f);
    EXPECT_FLOAT_EQ(frame.thruster[0].value2, 0.0f);
    EXPECT_FLOAT_EQ(frame.thruster[0].force_x, 0.5f);
    EXPECT_FLOAT_EQ(frame.thruster[0].force_y, 0.0f);
    EXPECT_EQ(frame.thruster[0].pwm1_us, 1700U);
    EXPECT_EQ(frame.thruster[0].pwm2_us, 1500U);
}

TEST(MiniDPOutput, Frame902ReverseSurgeFoldsAzimuthAndReversesThrust)
{
    MiniDP_OutputManager output;
    output.init(int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW));

    MiniDP_AxisCommand command{};
    command.surge = -1.0f;

    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);

    EXPECT_FLOAT_EQ(frame.actuator[0].demand, -0.5f);
    EXPECT_FLOAT_EQ(frame.actuator[1].demand, 0.0f);
    EXPECT_FLOAT_EQ(frame.actuator[2].demand, -0.5f);
    EXPECT_FLOAT_EQ(frame.actuator[3].demand, 0.0f);
    EXPECT_EQ(frame.actuator[0].pwm_us, 1300U);
    EXPECT_EQ(frame.actuator[1].pwm_us, 1500U);
    EXPECT_EQ(frame.actuator[2].pwm_us, 1300U);
    EXPECT_EQ(frame.actuator[3].pwm_us, 1500U);
    EXPECT_TRUE(
        (frame.thruster[0].flags &
         MiniDP_OutputManager::thruster_flag_reverse_folded) != 0);
    EXPECT_TRUE(
        (frame.thruster[1].flags &
         MiniDP_OutputManager::thruster_flag_reverse_folded) != 0);
    EXPECT_FLOAT_EQ(frame.thruster[0].value, -0.5f);
    EXPECT_FLOAT_EQ(frame.thruster[0].value2, 0.0f);
    EXPECT_FLOAT_EQ(frame.thruster[0].force_x, -0.5f);
}

TEST(MiniDPOutput, Frame902SwayUsesPodAzimuthAndBowThruster)
{
    MiniDP_OutputManager output;
    output.init(int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW));

    MiniDP_AxisCommand command{};
    command.sway = 1.0f;

    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);

    EXPECT_FLOAT_EQ(frame.actuator[0].demand, 0.25f);
    EXPECT_FLOAT_EQ(frame.actuator[2].demand, 0.25f);
    EXPECT_FLOAT_EQ(frame.actuator[4].demand, 0.5f);
    EXPECT_EQ(frame.actuator[0].pwm_us, 1600U);
    EXPECT_EQ(frame.actuator[1].pwm_us, 1900U);
    EXPECT_EQ(frame.actuator[2].pwm_us, 1600U);
    EXPECT_EQ(frame.actuator[3].pwm_us, 1900U);
    EXPECT_EQ(frame.actuator[4].pwm_us, 1700U);
    EXPECT_NEAR(frame.thruster[0].value2, 1.5707963f, 0.0001f);
    EXPECT_NEAR(frame.thruster[1].value2, 1.5707963f, 0.0001f);
    EXPECT_FLOAT_EQ(frame.thruster[2].value, 0.5f);
}

TEST(MiniDPOutput, SaturationScalesAllActuatorsTogether)
{
    MiniDP_OutputManager output;
    output.init();

    MiniDP_AxisCommand command{};
    command.surge = 1.0f;
    command.yaw = 1.0f;

    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);

    ASSERT_TRUE(frame.saturated);
    EXPECT_TRUE(frame.actuator[0].saturated);
    EXPECT_FLOAT_EQ(frame.actuator[0].demand, 1.0f);
    EXPECT_FLOAT_EQ(frame.actuator[1].demand, 0.0f);
    EXPECT_FLOAT_EQ(frame.actuator[2].demand, 0.5f);
    EXPECT_FLOAT_EQ(frame.actuator[3].demand, -0.5f);
    EXPECT_EQ(frame.actuator[0].pwm_us, 1900U);
    EXPECT_EQ(frame.actuator[2].pwm_us, 1700U);
    EXPECT_EQ(frame.actuator[3].pwm_us, 1300U);
}

TEST(MiniDPOutput, SafeActionsArePerActuator)
{
    MiniDP_OutputManager output;
    output.init();

    MiniDP_ActuatorConfig config = output.actuator_config(0);
    config.failsafe_action = MiniDP_OutputSafeAction::SEND_FAILSAFE;
    config.pwm_failsafe = 1234U;
    ASSERT_TRUE(output.set_actuator_config(0, config));

    config = output.actuator_config(1);
    config.failsafe_action = MiniDP_OutputSafeAction::DISABLE_PWM;
    ASSERT_TRUE(output.set_actuator_config(1, config));

    MiniDP_AxisCommand zero{};
    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::FAILSAFE, zero);

    EXPECT_TRUE(frame.actuator[0].pwm_enabled);
    EXPECT_EQ(frame.actuator[0].pwm_us, 1234U);
    EXPECT_FALSE(frame.actuator[1].pwm_enabled);
    EXPECT_TRUE(frame.actuator[2].pwm_enabled);
    EXPECT_EQ(frame.actuator[2].pwm_us, 1500U);
}

TEST(MiniDPOutput, ReversedActuatorFlipsPwmDirection)
{
    MiniDP_OutputManager output;
    output.init();

    MiniDP_ActuatorConfig config = output.actuator_config(0);
    config.reversed = true;
    ASSERT_TRUE(output.set_actuator_config(0, config));

    MiniDP_AxisCommand command{};
    command.surge = 0.5f;

    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::ARMED_ACTIVE, command);

    EXPECT_FLOAT_EQ(frame.actuator[0].demand, 0.5f);
    EXPECT_EQ(frame.actuator[0].pwm_us, 1300U);
}

TEST(MiniDPOutput, ActuatorCanTargetConfiguredPwmChannel)
{
    MiniDP_OutputManager output;
    output.init();

    MiniDP_ActuatorConfig config = output.actuator_config(0);
    config.pwm_channel = 5U;
    ASSERT_TRUE(output.set_actuator_config(0, config));

    MiniDP_AxisCommand zero{};
    const MiniDP_OutputFrame &frame =
        output.update(MiniDP_OutputState::ARMED_NEUTRAL, zero);

    EXPECT_TRUE(frame.actuator[0].pwm_enabled);
    EXPECT_EQ(frame.actuator[0].pwm_channel, 5U);
    EXPECT_EQ(frame.actuator[0].pwm_us, 1500U);
}

TEST(MiniDPOutput, RejectsDuplicateEnabledPwmChannel)
{
    MiniDP_OutputManager output;
    output.init();

    MiniDP_ActuatorConfig config = output.actuator_config(1);
    config.pwm_channel = 0U;
    EXPECT_FALSE(output.set_actuator_config(1, config));

    config.enabled = false;
    EXPECT_TRUE(output.set_actuator_config(1, config));
}

TEST(MiniDPOutput, ActuatorTestDrivesOnlyRequestedActuator)
{
    MiniDP_OutputManager output;
    output.init();

    MiniDP_ActuatorTestCommand command{};
    command.active = true;
    command.actuator_index = 1U;
    command.demand = -0.5f;

    const MiniDP_OutputFrame &frame = output.update_actuator_test(command);

    EXPECT_EQ(frame.state, MiniDP_OutputState::ACTUATOR_TEST);
    EXPECT_EQ(frame.active_pwm_count, 1U);
    EXPECT_FALSE(frame.actuator[0].pwm_enabled);
    EXPECT_TRUE(frame.actuator[1].pwm_enabled);
    EXPECT_EQ(frame.actuator[1].pwm_us, 1300U);
    EXPECT_FALSE(frame.actuator[2].pwm_enabled);
}

TEST(MiniDPOutput, ActuatorTestPwmClampsToActuatorConfig)
{
    MiniDP_OutputManager output;
    output.init();

    MiniDP_ActuatorTestCommand command{};
    command.active = true;
    command.actuator_index = 0U;
    command.use_pwm = true;
    command.pwm_us = 2100U;

    const MiniDP_OutputFrame &frame = output.update_actuator_test(command);

    EXPECT_EQ(frame.active_pwm_count, 1U);
    EXPECT_EQ(frame.actuator[0].pwm_us, 1900U);
}

AP_GTEST_MAIN()
