#include <AP_gtest.h>

#include "Arming.h"

static MiniDP_State valid_state()
{
    MiniDP_State state{};
    state.gps_fix_type = uint8_t(AP_GPS::GPS_OK_FIX_3D);
    state.gps_hacc_m = 1.0f;
    state.gps_sacc_m = 0.2f;
    state.yaw_valid = true;
    state.origin_valid = true;
    state.position_valid = true;
    state.velocity_valid = true;
    state.ekf_healthy = true;
    return state;
}

TEST(MiniDPArming, StartsDisarmedAndRequiresArmByDefault)
{
    MiniDP_Arming arming;
    arming.init();

    EXPECT_FALSE(arming.armed());
    EXPECT_FALSE(arming.outputs_allowed());
    EXPECT_TRUE(arming.config().arming_required);
    EXPECT_EQ(
        arming.config().gps_requirement,
        MiniDP_ArmGpsRequirement::NONE);
}

TEST(MiniDPArming, CanArmWithoutGpsWhenConfigured)
{
    MiniDP_Arming arming;
    arming.init();

    MiniDP_State state{};
    const MiniDP_ArmResult result =
        arming.arm(state, false, false, false);

    EXPECT_TRUE(result.accepted);
    EXPECT_TRUE(result.changed);
    EXPECT_TRUE(arming.armed());
    EXPECT_TRUE(arming.outputs_allowed());
}

TEST(MiniDPArming, CanRequireGpsFixAndQuality)
{
    MiniDP_Arming arming;
    arming.init();
    MiniDP_ArmingConfig config = arming.config();
    config.gps_requirement = MiniDP_ArmGpsRequirement::FIX;
    config.gps_hacc_max_m = 3.0f;
    config.gps_sacc_max_m_s = 1.0f;
    arming.set_config(config);

    MiniDP_State state = valid_state();
    state.gps_fix_type = uint8_t(AP_GPS::NO_FIX);
    const MiniDP_ArmResult no_fix =
        arming.arm(state, false, false, false);
    EXPECT_FALSE(no_fix.accepted);
    EXPECT_EQ(no_fix.rejection, MiniDP_ArmReject::GPS_FIX);

    state = valid_state();
    state.gps_hacc_m = 4.0f;
    const MiniDP_ArmResult poor_quality =
        arming.arm(state, false, false, false);
    EXPECT_FALSE(poor_quality.accepted);
    EXPECT_EQ(poor_quality.rejection, MiniDP_ArmReject::GPS_QUALITY);

    state.gps_hacc_m = 2.0f;
    const MiniDP_ArmResult accepted =
        arming.arm(state, false, false, false);
    EXPECT_TRUE(accepted.accepted);
    EXPECT_TRUE(arming.armed());
}

TEST(MiniDPArming, CanRequireDpReadyState)
{
    MiniDP_Arming arming;
    arming.init();
    MiniDP_ArmingConfig config = arming.config();
    config.gps_requirement = MiniDP_ArmGpsRequirement::DP_READY;
    arming.set_config(config);

    MiniDP_State state = valid_state();
    state.position_valid = false;
    const MiniDP_ArmResult rejected =
        arming.arm(state, false, false, false);
    EXPECT_FALSE(rejected.accepted);
    EXPECT_EQ(rejected.rejection, MiniDP_ArmReject::STATE_INVALID);

    state.position_valid = true;
    const MiniDP_ArmResult accepted =
        arming.arm(state, false, false, false);
    EXPECT_TRUE(accepted.accepted);
}

TEST(MiniDPArming, ForceSkipsGpsButNotKillOrFailsafe)
{
    MiniDP_Arming arming;
    arming.init();
    MiniDP_ArmingConfig config = arming.config();
    config.gps_requirement = MiniDP_ArmGpsRequirement::DP_READY;
    arming.set_config(config);

    MiniDP_State state{};
    EXPECT_FALSE(arming.arm(state, true, false, true).accepted);
    EXPECT_FALSE(arming.arm(state, false, true, true).accepted);

    const MiniDP_ArmResult accepted =
        arming.arm(state, false, false, true);
    EXPECT_TRUE(accepted.accepted);
    EXPECT_TRUE(arming.armed());
}

TEST(MiniDPArming, DisarmIsIdempotent)
{
    MiniDP_Arming arming;
    arming.init();
    ASSERT_TRUE(arming.arm(valid_state(), false, false, false).accepted);

    const MiniDP_ArmResult disarmed = arming.disarm();
    EXPECT_TRUE(disarmed.accepted);
    EXPECT_TRUE(disarmed.changed);
    EXPECT_FALSE(arming.armed());

    const MiniDP_ArmResult already_disarmed = arming.disarm();
    EXPECT_TRUE(already_disarmed.accepted);
    EXPECT_FALSE(already_disarmed.changed);
}

AP_GTEST_MAIN()
