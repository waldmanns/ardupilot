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
    state.gps_hacc_m = 1.0f;
    state.gps_sacc_m = 0.2f;
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

TEST(MiniDPMode, DPHoldRequiresConfiguredGpsQuality)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_ModeConfig config{};
    config.dp_hacc_max_m = 3.0f;
    config.dp_sacc_max_m = 1.0f;
    manager.set_config(config);

    MiniDP_State state = valid_state();
    state.gps_hacc_m = 4.0f;

    const auto rejected_hacc = manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state);
    EXPECT_FALSE(rejected_hacc.accepted);
    EXPECT_EQ(
        rejected_hacc.rejection,
        MiniDP_ModeReject::GPS_QUALITY_INVALID);

    state.gps_hacc_m = 2.0f;
    state.gps_sacc_m = NAN;
    const auto rejected_unknown_sacc = manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state);
    EXPECT_FALSE(rejected_unknown_sacc.accepted);
    EXPECT_EQ(
        rejected_unknown_sacc.rejection,
        MiniDP_ModeReject::GPS_QUALITY_INVALID);

    config.dp_sacc_max_m = 0.0f;
    manager.set_config(config);
    const auto accepted = manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state);
    EXPECT_TRUE(accepted.accepted);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::DP_HOLD);
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

TEST(MiniDPMode, DPHoldCanRelatchCurrentPositionOnRequest)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_State state = valid_state();
    ASSERT_TRUE(manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state).accepted);

    const uint32_t first_target_id = manager.target().target_id;
    state.time_us++;
    state.pos_n_m = 20.0f;
    state.pos_e_m = 30.0f;
    const auto unchanged = manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::MAVLINK_REQUEST,
        state);
    EXPECT_TRUE(unchanged.accepted);
    EXPECT_FALSE(unchanged.changed);
    EXPECT_EQ(manager.target().target_id, first_target_id);
    EXPECT_FLOAT_EQ(manager.target().pos_n_m, 12.0f);
    EXPECT_FLOAT_EQ(manager.target().pos_e_m, -4.0f);

    state.time_us++;
    const auto retargeted = manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::MAVLINK_REQUEST,
        state,
        false,
        true);
    EXPECT_TRUE(retargeted.accepted);
    EXPECT_TRUE(retargeted.changed);
    EXPECT_GT(manager.target().target_id, first_target_id);
    EXPECT_FLOAT_EQ(manager.target().pos_n_m, 20.0f);
    EXPECT_FLOAT_EQ(manager.target().pos_e_m, 30.0f);
}

TEST(MiniDPMode, ExplicitDPTargetEntersHold)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    const MiniDP_State state = valid_state();

    MiniDP_ModeTarget target{};
    target.yaw_valid = true;
    target.position_valid = true;
    target.yaw_rad = 0.25f;
    target.pos_n_m = 42.0f;
    target.pos_e_m = -8.0f;
    target.origin_id = state.origin_id;
    target.reset_counter = state.reset_counter;

    const auto accepted = manager.request_dp_target(
        target,
        MiniDP_ModeReason::MAVLINK_REQUEST,
        state);

    EXPECT_TRUE(accepted.accepted);
    EXPECT_TRUE(accepted.changed);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::DP_HOLD);
    EXPECT_TRUE(manager.target().yaw_valid);
    EXPECT_TRUE(manager.target().position_valid);
    EXPECT_FLOAT_EQ(manager.target().yaw_rad, 0.25f);
    EXPECT_FLOAT_EQ(manager.target().pos_n_m, 42.0f);
    EXPECT_FLOAT_EQ(manager.target().pos_e_m, -8.0f);
    EXPECT_EQ(manager.target().origin_id, state.origin_id);
    EXPECT_EQ(manager.target().reset_counter, state.reset_counter);
    EXPECT_EQ(manager.last_transition().from, MiniDP_Mode::MANUAL);
    EXPECT_EQ(manager.last_transition().to, MiniDP_Mode::DP_HOLD);
}

TEST(MiniDPMode, ExplicitDPTargetUpdatesCurrentHoldPoint)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_State state = valid_state();

    MiniDP_ModeTarget first{};
    first.yaw_valid = true;
    first.position_valid = true;
    first.yaw_rad = state.yaw_rad;
    first.pos_n_m = 10.0f;
    first.pos_e_m = 20.0f;
    first.origin_id = state.origin_id;
    first.reset_counter = state.reset_counter;
    ASSERT_TRUE(manager.request_dp_target(
        first,
        MiniDP_ModeReason::MAVLINK_REQUEST,
        state).accepted);
    const uint32_t first_target_id = manager.target().target_id;

    state.time_us++;
    MiniDP_ModeTarget second = first;
    second.pos_n_m = -5.0f;
    second.pos_e_m = 15.0f;
    second.yaw_rad = -0.75f;
    const auto updated = manager.request_dp_target(
        second,
        MiniDP_ModeReason::MAVLINK_REQUEST,
        state);

    EXPECT_TRUE(updated.accepted);
    EXPECT_TRUE(updated.changed);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::DP_HOLD);
    EXPECT_GT(manager.target().target_id, first_target_id);
    EXPECT_FLOAT_EQ(manager.target().pos_n_m, -5.0f);
    EXPECT_FLOAT_EQ(manager.target().pos_e_m, 15.0f);
    EXPECT_FLOAT_EQ(manager.target().yaw_rad, -0.75f);
    EXPECT_EQ(manager.last_transition().from, MiniDP_Mode::DP_HOLD);
    EXPECT_EQ(manager.last_transition().to, MiniDP_Mode::DP_HOLD);
}

TEST(MiniDPMode, ExplicitDPTargetRejectsInvalidTarget)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    const MiniDP_State state = valid_state();

    MiniDP_ModeTarget target{};
    target.yaw_valid = true;
    target.position_valid = true;
    target.yaw_rad = state.yaw_rad;
    target.pos_n_m = 2.0f;
    target.pos_e_m = 3.0f;
    target.origin_id = state.origin_id + 1U;
    target.reset_counter = state.reset_counter;

    const auto wrong_origin = manager.request_dp_target(
        target,
        MiniDP_ModeReason::MAVLINK_REQUEST,
        state);
    EXPECT_FALSE(wrong_origin.accepted);
    EXPECT_EQ(wrong_origin.rejection, MiniDP_ModeReject::TARGET_INVALID);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::MANUAL);

    target.origin_id = state.origin_id;
    target.pos_n_m = NAN;
    const auto nan_target = manager.request_dp_target(
        target,
        MiniDP_ModeReason::MAVLINK_REQUEST,
        state);
    EXPECT_FALSE(nan_target.accepted);
    EXPECT_EQ(nan_target.rejection, MiniDP_ModeReject::TARGET_INVALID);
    EXPECT_EQ(manager.mode(), MiniDP_Mode::MANUAL);
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

TEST(MiniDPMode, DPFallsBackToHeadingWhenGpsQualityDegrades)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    MiniDP_ModeConfig config{};
    config.dp_hacc_max_m = 3.0f;
    config.dp_sacc_max_m = 1.0f;
    manager.set_config(config);

    MiniDP_State state = valid_state();
    ASSERT_TRUE(manager.request_mode(
        MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST,
        state).accepted);

    state.time_us++;
    state.gps_sacc_m = 1.5f;
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


TEST(MiniDPMode, RepeatedIdenticalTargetPreservesIdentity)
{
    MiniDP_ModeManager manager;
    manager.init(0);
    const auto state = valid_state();
    ASSERT_TRUE(manager.request_mode(MiniDP_Mode::DP_HOLD,
        MiniDP_ModeReason::USER_REQUEST, state).accepted);
    auto target = manager.target();
    const uint32_t id = target.target_id;
    for (unsigned i = 0; i < 100; i++) {
        auto result = manager.request_dp_target(target,
            MiniDP_ModeReason::MAVLINK_REQUEST, state);
        EXPECT_TRUE(result.accepted);
        EXPECT_FALSE(result.changed);
        EXPECT_EQ(manager.target().target_id, id);
    }
    target.yaw_rad = NAN;
    EXPECT_FALSE(manager.request_dp_target(target,
        MiniDP_ModeReason::MAVLINK_REQUEST, state).accepted);
    EXPECT_EQ(manager.target().target_id, id);
}

AP_GTEST_MAIN()
