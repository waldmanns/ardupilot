#include <AP_gtest.h>

#include "Authority.h"

static MiniDP_AuthorityStatus healthy_status()
{
    MiniDP_AuthorityStatus status{};
    status.time_us = 1000;
    status.rc_healthy = true;
    status.mavlink_healthy = true;
    status.mavlink_manual_authorized = true;
    status.mavlink_target_authorized = true;
    status.actuator_test_authorized = true;
    return status;
}

static MiniDP_AuthorityPolicy safe_policy()
{
    MiniDP_AuthorityPolicy policy{};
    policy.rc_fallback_on_mavlink_loss = true;
    policy.target_link_loss_action = MiniDP_TargetLinkLossAction::FAILSAFE;
    return policy;
}

TEST(MiniDPAuthority, StartsWithNoOwner)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(42);

    EXPECT_TRUE(arbiter.initialised());
    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::NONE);
    EXPECT_EQ(arbiter.last_transition().sequence, 1U);
    EXPECT_EQ(
        arbiter.last_transition().reason,
        MiniDP_AuthorityReason::STARTUP);
}

TEST(MiniDPAuthority, RCRequiresHealthyLink)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    MiniDP_AuthorityStatus status = healthy_status();
    status.rc_healthy = false;

    const auto rejected = arbiter.request_owner(
        MiniDP_ControlOwner::RC,
        MiniDP_AuthorityReason::RC_REQUEST,
        status);
    EXPECT_FALSE(rejected.accepted);
    EXPECT_EQ(rejected.rejection, MiniDP_AuthorityReject::RC_UNHEALTHY);

    status.rc_healthy = true;
    const auto accepted = arbiter.request_owner(
        MiniDP_ControlOwner::RC,
        MiniDP_AuthorityReason::RC_REQUEST,
        status);
    EXPECT_TRUE(accepted.accepted);
    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::RC);
}

TEST(MiniDPAuthority, MavlinkPermissionsAreSeparate)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    MiniDP_AuthorityStatus status = healthy_status();
    status.mavlink_manual_authorized = false;

    const auto manual_rejected = arbiter.request_owner(
        MiniDP_ControlOwner::MAVLINK_MANUAL,
        MiniDP_AuthorityReason::MAVLINK_MANUAL_REQUEST,
        status);
    EXPECT_FALSE(manual_rejected.accepted);
    EXPECT_EQ(
        manual_rejected.rejection,
        MiniDP_AuthorityReject::MAVLINK_MANUAL_UNAUTHORIZED);

    const auto target_accepted = arbiter.request_owner(
        MiniDP_ControlOwner::MAVLINK_TARGET,
        MiniDP_AuthorityReason::MAVLINK_TARGET_REQUEST,
        status);
    EXPECT_TRUE(target_accepted.accepted);
    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::MAVLINK_TARGET);
}

TEST(MiniDPAuthority, RCTakeoverMustBeEnabled)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    MiniDP_AuthorityStatus status = healthy_status();
    ASSERT_TRUE(arbiter.request_owner(
        MiniDP_ControlOwner::MAVLINK_MANUAL,
        MiniDP_AuthorityReason::MAVLINK_MANUAL_REQUEST,
        status).accepted);

    const auto rejected = arbiter.request_owner(
        MiniDP_ControlOwner::RC,
        MiniDP_AuthorityReason::RC_TAKEOVER,
        status);
    EXPECT_FALSE(rejected.accepted);
    EXPECT_EQ(
        rejected.rejection,
        MiniDP_AuthorityReject::RC_TAKEOVER_DISABLED);

    MiniDP_AuthorityPolicy policy = safe_policy();
    policy.rc_takeover_from_mavlink = true;
    arbiter.set_policy(policy);
    const auto accepted = arbiter.request_owner(
        MiniDP_ControlOwner::RC,
        MiniDP_AuthorityReason::RC_TAKEOVER,
        status);
    EXPECT_TRUE(accepted.accepted);
    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::RC);
}

TEST(MiniDPAuthority, RCLossDefaultsToFailsafe)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    arbiter.set_policy(safe_policy());
    MiniDP_AuthorityStatus status = healthy_status();
    ASSERT_TRUE(arbiter.request_owner(
        MiniDP_ControlOwner::RC,
        MiniDP_AuthorityReason::RC_REQUEST,
        status).accepted);

    status.time_us++;
    status.rc_healthy = false;
    arbiter.update(status);

    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::FAILSAFE);
    EXPECT_EQ(
        arbiter.last_transition().reason,
        MiniDP_AuthorityReason::RC_LINK_LOSS);
}

TEST(MiniDPAuthority, RCLossCanExplicitlyTransferToMavlink)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    MiniDP_AuthorityPolicy policy = safe_policy();
    policy.mavlink_manual_takeover_on_rc_loss = true;
    arbiter.set_policy(policy);
    MiniDP_AuthorityStatus status = healthy_status();
    ASSERT_TRUE(arbiter.request_owner(
        MiniDP_ControlOwner::RC,
        MiniDP_AuthorityReason::RC_REQUEST,
        status).accepted);

    status.time_us++;
    status.rc_healthy = false;
    arbiter.update(status);

    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::MAVLINK_MANUAL);
    EXPECT_EQ(
        arbiter.last_transition().reason,
        MiniDP_AuthorityReason::RC_LOSS_MAVLINK_TAKEOVER);
}

TEST(MiniDPAuthority, MavlinkManualLossFallsBackToRC)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    arbiter.set_policy(safe_policy());
    MiniDP_AuthorityStatus status = healthy_status();
    ASSERT_TRUE(arbiter.request_owner(
        MiniDP_ControlOwner::MAVLINK_MANUAL,
        MiniDP_AuthorityReason::MAVLINK_MANUAL_REQUEST,
        status).accepted);

    status.time_us++;
    status.mavlink_healthy = false;
    arbiter.update(status);

    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::RC);
    EXPECT_EQ(
        arbiter.last_transition().reason,
        MiniDP_AuthorityReason::MAVLINK_LINK_LOSS);
}

TEST(MiniDPAuthority, TargetLinkCanHoldAndRecordRecovery)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    MiniDP_AuthorityPolicy policy = safe_policy();
    policy.target_link_loss_action = MiniDP_TargetLinkLossAction::HOLD_TARGET;
    arbiter.set_policy(policy);
    MiniDP_AuthorityStatus status = healthy_status();
    ASSERT_TRUE(arbiter.request_owner(
        MiniDP_ControlOwner::MAVLINK_TARGET,
        MiniDP_AuthorityReason::MAVLINK_TARGET_REQUEST,
        status).accepted);

    status.time_us++;
    status.mavlink_healthy = false;
    arbiter.update(status);
    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::MAVLINK_TARGET);
    EXPECT_EQ(
        arbiter.last_transition().reason,
        MiniDP_AuthorityReason::MAVLINK_TARGET_HOLD);
    const uint32_t held_sequence = arbiter.last_transition().sequence;

    status.time_us++;
    arbiter.update(status);
    EXPECT_EQ(arbiter.last_transition().sequence, held_sequence);

    status.time_us++;
    status.mavlink_healthy = true;
    arbiter.update(status);
    EXPECT_EQ(
        arbiter.last_transition().reason,
        MiniDP_AuthorityReason::MAVLINK_LINK_RESTORED);
}

TEST(MiniDPAuthority, TargetLinkFallbackRequiresHealthyRC)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    MiniDP_AuthorityPolicy policy = safe_policy();
    policy.target_link_loss_action =
        MiniDP_TargetLinkLossAction::FALLBACK_TO_RC;
    arbiter.set_policy(policy);
    MiniDP_AuthorityStatus status = healthy_status();
    ASSERT_TRUE(arbiter.request_owner(
        MiniDP_ControlOwner::MAVLINK_TARGET,
        MiniDP_AuthorityReason::MAVLINK_TARGET_REQUEST,
        status).accepted);

    status.time_us++;
    status.mavlink_healthy = false;
    arbiter.update(status);
    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::RC);

    MiniDP_AuthorityArbiter no_rc_arbiter;
    no_rc_arbiter.init(0);
    no_rc_arbiter.set_policy(policy);
    status = healthy_status();
    ASSERT_TRUE(no_rc_arbiter.request_owner(
        MiniDP_ControlOwner::MAVLINK_TARGET,
        MiniDP_AuthorityReason::MAVLINK_TARGET_REQUEST,
        status).accepted);
    status.time_us++;
    status.mavlink_healthy = false;
    status.rc_healthy = false;
    no_rc_arbiter.update(status);
    EXPECT_EQ(no_rc_arbiter.owner(), MiniDP_ControlOwner::FAILSAFE);
}

TEST(MiniDPAuthority, RCKillAlwaysWinsAndFailsafeIsLatched)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    MiniDP_AuthorityStatus status = healthy_status();
    ASSERT_TRUE(arbiter.request_owner(
        MiniDP_ControlOwner::MAVLINK_MANUAL,
        MiniDP_AuthorityReason::MAVLINK_MANUAL_REQUEST,
        status).accepted);

    status.time_us++;
    status.rc_kill = true;
    arbiter.update(status);
    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::FAILSAFE);
    EXPECT_EQ(
        arbiter.last_transition().reason,
        MiniDP_AuthorityReason::RC_KILL);

    status.rc_kill = false;
    const auto request_rejected = arbiter.request_owner(
        MiniDP_ControlOwner::RC,
        MiniDP_AuthorityReason::RC_REQUEST,
        status);
    EXPECT_FALSE(request_rejected.accepted);
    EXPECT_EQ(
        request_rejected.rejection,
        MiniDP_AuthorityReject::FAILSAFE_LATCHED);

    const auto cleared = arbiter.clear_failsafe(status);
    EXPECT_TRUE(cleared.accepted);
    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::NONE);
}

TEST(MiniDPAuthority, ExternalFailsafeLatchesAuthority)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    MiniDP_AuthorityStatus status = healthy_status();
    ASSERT_TRUE(arbiter.request_owner(
        MiniDP_ControlOwner::RC,
        MiniDP_AuthorityReason::RC_REQUEST,
        status).accepted);

    status.time_us++;
    status.failsafe_active = true;
    arbiter.update(status);

    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::FAILSAFE);
    EXPECT_EQ(
        arbiter.last_transition().reason,
        MiniDP_AuthorityReason::FAILSAFE_TRIGGERED);
}

TEST(MiniDPAuthority, TestOwnerIsExclusiveAndRevocationIsSafe)
{
    MiniDP_AuthorityArbiter arbiter;
    arbiter.init(0);
    MiniDP_AuthorityStatus status = healthy_status();
    ASSERT_TRUE(arbiter.request_owner(
        MiniDP_ControlOwner::TEST,
        MiniDP_AuthorityReason::TEST_REQUEST,
        status).accepted);

    const auto rc_rejected = arbiter.request_owner(
        MiniDP_ControlOwner::RC,
        MiniDP_AuthorityReason::RC_REQUEST,
        status);
    EXPECT_FALSE(rc_rejected.accepted);
    EXPECT_EQ(rc_rejected.rejection, MiniDP_AuthorityReject::TEST_ACTIVE);

    status.time_us++;
    status.actuator_test_authorized = false;
    arbiter.update(status);
    EXPECT_EQ(arbiter.owner(), MiniDP_ControlOwner::NONE);
    EXPECT_EQ(
        arbiter.last_transition().reason,
        MiniDP_AuthorityReason::TEST_AUTH_REVOKED);
}

AP_GTEST_MAIN()
