#include <AP_gtest.h>

#include "Mode.h"

static MiniDP_State valid_state()
{
    MiniDP_State state{};
    state.time_us = 1000;
    state.yaw_valid = true;
    state.position_valid = true;
    state.velocity_valid = true;
    state.origin_valid = true;
    state.ekf_healthy = true;
    state.yaw_rad = 1.25f;
    state.pos_n_m = 12.0f;
    state.pos_e_m = -4.0f;
    state.origin_id = 3;
    state.reset_counter = 7;
    return state;
}

TEST(MiniDPMode, StartsInManual)
{
    MiniDP_ModeManager manager;
    manager.init(42);

    EXPECT_TRUE(manager.initialised());
    EXPECT_EQ(manager.mode(), MiniDP_Mode::MANUAL);
    EXPECT_EQ(manager.last_transition().sequence, 1U);
    EXPECT_EQ(manager.last_transition().reason, MiniDP_ModeReason::STARTUP);
    EXPECT_FALSE(manager.target().yaw_valid);
}

TEST(MiniDPMode, HeadingHoldRequiresYawAndLatchesTarget)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_State state{};

    const auto rejected = manager.request_mode(
        MiniDP_Mode::HEADING_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state);
    EXPECT_FALSE(rejected.accepted);
    EXPECT_EQ(rejected.rejection, MiniDP_ModeReject::YAW_INVALID);

    state = valid_state();
    const auto accepted = manager.request_mode(
        MiniDP_Mode::HEADING_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state);
    EXPECT_TRUE(accepted.accepted);
    EXPECT_TRUE(accepted.changed);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::HEADING_HOLD);
    EXPECT_TRUE(manager.target().yaw_valid);
    EXPECT_FALSE(manager.target().position_valid);
    EXPECT_FLOAT_EQ(manager.target().yaw_rad, state.yaw_rad);
}

TEST(MiniDPMode, DPHoldRequiresCompleteStateAndLatchesTarget)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_State state = valid_state();
    state.velocity_valid = false;

    const auto rejected = manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state);
    EXPECT_FALSE(rejected.accepted);
    EXPECT_EQ(rejected.rejection, MiniDP_ModeReject::VELOCITY_INVALID);

    state.velocity_valid = true;
    const auto accepted = manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state);
    EXPECT_TRUE(accepted.accepted);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::DP_HOLD);
    EXPECT_TRUE(manager.target().yaw_valid);
    EXPECT_TRUE(manager.target().position_valid);
    EXPECT_FLOAT_EQ(manager.target().pos_n_m, state.pos_n_m);
    EXPECT_FLOAT_EQ(manager.target().pos_e_m, state.pos_e_m);
    EXPECT_EQ(manager.target().origin_id, state.origin_id);
    EXPECT_EQ(manager.target().reset_counter, state.reset_counter);
}

TEST(MiniDPMode, DPFallsBackToHeadingOnEstimatorReset)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_State state = valid_state();
    ASSERT_TRUE(manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state).accepted);

    state.time_us++;
    state.reset_counter++;
    state.yaw_rad = -0.5f;
    manager.update(state);

    EXPECT_EQ(manager.mode(), MiniDP_Mode::HEADING_HOLD);
    EXPECT_EQ(
        manager.last_transition().reason,
        MiniDP_ModeReason::ESTIMATOR_RESET);
    EXPECT_FLOAT_EQ(manager.target().yaw_rad, state.yaw_rad);
    EXPECT_FALSE(manager.target().position_valid);
}

TEST(MiniDPMode, HeadingHoldRelatchesYawOnEstimatorReset)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_State state = valid_state();
    ASSERT_TRUE(manager.request_mode(
        MiniDP_Mode::HEADING_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state).accepted);
    const uint32_t previous_target_id = manager.target().target_id;

    state.time_us++;
    state.reset_counter++;
    state.yaw_rad = -0.75f;
    manager.update(state);

    EXPECT_EQ(manager.mode(), MiniDP_Mode::HEADING_HOLD);
    EXPECT_EQ(
        manager.last_transition().reason,
        MiniDP_ModeReason::ESTIMATOR_RESET);
    EXPECT_GT(manager.target().target_id, previous_target_id);
    EXPECT_FLOAT_EQ(manager.target().yaw_rad, state.yaw_rad);
}

TEST(MiniDPMode, DPFallsBackToHeadingWhenPositionIsLost)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_State state = valid_state();
    ASSERT_TRUE(manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state).accepted);

    state.time_us++;
    state.position_valid = false;
    manager.update(state);

    EXPECT_EQ(manager.mode(), MiniDP_Mode::HEADING_HOLD);
    EXPECT_EQ(
        manager.last_transition().reason,
        MiniDP_ModeReason::STATE_INVALID);
}

TEST(MiniDPMode, YawLossEntersLatchedFailsafe)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_State state = valid_state();
    ASSERT_TRUE(manager.request_mode(
        MiniDP_Mode::HEADING_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state).accepted);

    state.time_us++;
    state.yaw_valid = false;
    manager.update(state);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::FAILSAFE);

    const auto override_rejected = manager.manual_override(state);
    EXPECT_FALSE(override_rejected.accepted);
    EXPECT_EQ(
        override_rejected.rejection,
        MiniDP_ModeReject::FAILSAFE_LATCHED);

    const auto direct_dp_rejected = manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::FAILSAFE_CLEARED,
        valid_state());
    EXPECT_FALSE(direct_dp_rejected.accepted);
    EXPECT_EQ(
        direct_dp_rejected.rejection,
        MiniDP_ModeReject::FAILSAFE_LATCHED);

    const auto cleared = manager.request_mode(
        MiniDP_Mode::MANUAL,
        MiniDP_ModeReason::FAILSAFE_CLEARED,
        state);
    EXPECT_TRUE(cleared.accepted);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::MANUAL);
}

TEST(MiniDPMode, ManualOverrideClearsControlTarget)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    const MiniDP_State state = valid_state();
    ASSERT_TRUE(manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state).accepted);

    const auto result = manager.manual_override(state);
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::MANUAL);
    EXPECT_FALSE(manager.target().yaw_valid);
    EXPECT_FALSE(manager.target().position_valid);
}

TEST(MiniDPMode, ActuatorTestRequiresExplicitAuthorization)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    const MiniDP_State state = valid_state();

    const auto rejected = manager.request_mode(
        MiniDP_Mode::ACTUATOR_TEST,
        MiniDP_ModeReason::ACTUATOR_TEST_REQUEST,
        state);
    EXPECT_FALSE(rejected.accepted);
    EXPECT_EQ(
        rejected.rejection,
        MiniDP_ModeReject::ACTUATOR_TEST_NOT_AUTHORIZED);

    const auto accepted = manager.request_mode(
        MiniDP_Mode::ACTUATOR_TEST,
        MiniDP_ModeReason::ACTUATOR_TEST_REQUEST,
        state,
        true);
    EXPECT_TRUE(accepted.accepted);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::ACTUATOR_TEST);
}

AP_GTEST_MAIN()
