#include <AP_gtest.h>

#include "Input.h"

#include <limits.h>

namespace {

MiniDP_RCInputFrame healthy_frame(const uint8_t channel_count = 8U)
{
    MiniDP_RCInputFrame frame{};
    frame.healthy = true;
    frame.channel_count = channel_count;
    for (uint8_t i = 0; i < MiniDP_InputMapper::max_rc_channels; i++) {
        frame.pwm[i] = 1500U;
    }
    return frame;
}

} // namespace

TEST(MiniDPInput, DefaultsMapRcSurgeSwayYawThroughLimits)
{
    MiniDP_InputMapper input;
    input.init();

    MiniDP_RCInputFrame frame = healthy_frame();
    frame.norm[1] = 0.8f;   // RC2 surge
    frame.norm[0] = -0.4f;  // RC1 sway
    frame.norm[3] = 0.3f;   // RC4 yaw

    const MiniDP_ManualCommand command = input.map_rc(frame, 100U);

    ASSERT_TRUE(command.valid);
    EXPECT_EQ(command.source, MiniDP_ManualSource::RC);
    EXPECT_FLOAT_EQ(command.axes.surge, 0.4f);
    EXPECT_FLOAT_EQ(command.axes.sway, -0.2f);
    EXPECT_FLOAT_EQ(command.axes.yaw, 0.15f);
}

TEST(MiniDPInput, DeadbandZerosSmallManualInputs)
{
    MiniDP_InputMapper input;
    input.init();

    MiniDP_RCInputFrame frame = healthy_frame();
    frame.norm[1] = 0.02f;
    frame.norm[0] = -0.02f;
    frame.norm[3] = 0.02f;

    const MiniDP_ManualCommand command = input.map_rc(frame, 100U);

    ASSERT_TRUE(command.valid);
    EXPECT_FLOAT_EQ(command.axes.surge, 0.0f);
    EXPECT_FLOAT_EQ(command.axes.sway, 0.0f);
    EXPECT_FLOAT_EQ(command.axes.yaw, 0.0f);
}

TEST(MiniDPInput, RejectsRcWhenMappedChannelMissing)
{
    MiniDP_InputMapper input;
    input.init();

    MiniDP_RCInputFrame frame = healthy_frame(3U);
    frame.norm[1] = 0.8f;

    const MiniDP_ManualCommand command = input.map_rc(frame, 100U);

    EXPECT_FALSE(command.valid);
}

TEST(MiniDPInput, ConfiguredRcKillBlocksCommand)
{
    MiniDP_InputMapper input;
    input.init();

    MiniDP_InputConfig config = input.config();
    config.rc_kill_channel = 5U;
    config.rc_kill_pwm = 1700U;
    input.set_config(config);

    MiniDP_RCInputFrame frame = healthy_frame();
    frame.pwm[4] = 1800U;
    frame.norm[1] = 0.8f;

    const MiniDP_ManualCommand command = input.map_rc(frame, 100U);

    EXPECT_TRUE(input.rc_kill_active(frame));
    EXPECT_TRUE(command.kill);
    EXPECT_FALSE(command.valid);
}

TEST(MiniDPInput, ConfiguredRcArmAndDisarmSwitchesReportActive)
{
    MiniDP_InputMapper input;
    input.init();

    MiniDP_InputConfig config = input.config();
    config.rc_arm_channel = 6U;
    config.rc_arm_pwm = 1700U;
    config.rc_disarm_channel = 7U;
    config.rc_disarm_pwm = 1300U;
    input.set_config(config);

    MiniDP_RCInputFrame frame = healthy_frame();
    EXPECT_FALSE(input.rc_arm_active(frame));
    EXPECT_FALSE(input.rc_disarm_active(frame));

    frame.pwm[5] = 1800U;
    EXPECT_TRUE(input.rc_arm_active(frame));

    frame.pwm[6] = 1200U;
    EXPECT_TRUE(input.rc_disarm_active(frame));
}

TEST(MiniDPInput, MavlinkManualMapsAxesAndTimesOut)
{
    MiniDP_InputMapper input;
    input.init();

    input.record_mavlink_manual_control(100U, 600, -1000, 250);

    MiniDP_ManualCommand command = input.mavlink_manual_command(500U);
    ASSERT_TRUE(command.valid);
    EXPECT_EQ(command.source, MiniDP_ManualSource::MAVLINK);
    EXPECT_FLOAT_EQ(command.axes.surge, 0.3f);
    EXPECT_FLOAT_EQ(command.axes.sway, -0.5f);
    EXPECT_FLOAT_EQ(command.axes.yaw, 0.125f);

    command = input.mavlink_manual_command(601U);
    EXPECT_FALSE(command.valid);
}

TEST(MiniDPInput, MavlinkManualClampsAndIgnoresInvalidAxis)
{
    MiniDP_InputMapper input;
    input.init();

    input.record_mavlink_manual_control(100U, 1500, INT16_MAX, -1500);

    const MiniDP_ManualCommand command = input.mavlink_manual_command(100U);

    ASSERT_TRUE(command.valid);
    EXPECT_FLOAT_EQ(command.axes.surge, 0.5f);
    EXPECT_FLOAT_EQ(command.axes.sway, 0.0f);
    EXPECT_FLOAT_EQ(command.axes.yaw, -0.5f);
}


TEST(MiniDPInput, SparseOverridesCannotUseStaleReceiverChannels)
{
    MiniDP_InputMapper input;
    input.init();
    auto frame = healthy_frame(16);
    frame.valid_mask = (1U << 0) | (1U << 1);
    EXPECT_FALSE(input.map_rc(frame, 100).valid);
    frame.valid_mask |= (1U << 3);
    EXPECT_TRUE(input.map_rc(frame, 100).valid);
    auto config = input.config();
    config.rc_arm_channel = 16;
    input.set_config(config);
    frame.pwm[15] = 2000;
    EXPECT_FALSE(input.rc_arm_active(frame));
}

AP_GTEST_MAIN()
