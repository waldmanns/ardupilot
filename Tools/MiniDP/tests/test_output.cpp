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
