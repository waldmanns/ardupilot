#include <AP_gtest.h>

#include "AxisLimiter.h"

#include <math.h>

TEST(MiniDPAxisLimiter, DefaultsPassThroughUnitCommands)
{
    MiniDP_AxisLimiter limiter;
    limiter.init(0U);

    const MiniDP_AxisCommand output =
        limiter.update(10U, {0.5f, -0.75f, 1.25f});

    EXPECT_FLOAT_EQ(output.surge, 0.5f);
    EXPECT_FLOAT_EQ(output.sway, -0.75f);
    EXPECT_FLOAT_EQ(output.yaw, 1.0f);
}

TEST(MiniDPAxisLimiter, PerAxisLimitsClampCommand)
{
    MiniDP_AxisLimiter limiter;
    limiter.init(0U);

    MiniDP_AxisLimiterConfig config = limiter.config();
    config.surge_limit = 0.25f;
    config.sway_limit = 0.5f;
    config.yaw_limit = 0.75f;
    limiter.set_config(config);

    const MiniDP_AxisCommand output =
        limiter.update(10U, {1.0f, -1.0f, 1.0f});

    EXPECT_FLOAT_EQ(output.surge, 0.25f);
    EXPECT_FLOAT_EQ(output.sway, -0.5f);
    EXPECT_FLOAT_EQ(output.yaw, 0.75f);
}

TEST(MiniDPAxisLimiter, DeadbandZerosSmallCommands)
{
    MiniDP_AxisLimiter limiter;
    limiter.init(0U);

    MiniDP_AxisLimiterConfig config = limiter.config();
    config.deadband = 0.1f;
    limiter.set_config(config);

    const MiniDP_AxisCommand output =
        limiter.update(10U, {0.05f, -0.09f, 0.11f});

    EXPECT_FLOAT_EQ(output.surge, 0.0f);
    EXPECT_FLOAT_EQ(output.sway, 0.0f);
    EXPECT_FLOAT_EQ(output.yaw, 0.11f);
}

TEST(MiniDPAxisLimiter, SlewRateLimitsRateOfChange)
{
    MiniDP_AxisLimiter limiter;
    limiter.init(0U);

    MiniDP_AxisLimiterConfig config = limiter.config();
    config.surge_slew_rate = 0.5f;
    config.sway_slew_rate = 1.0f;
    config.yaw_slew_rate = 2.0f;
    limiter.set_config(config);

    MiniDP_AxisCommand output =
        limiter.update(500U, {1.0f, -1.0f, 1.0f});
    EXPECT_FLOAT_EQ(output.surge, 0.25f);
    EXPECT_FLOAT_EQ(output.sway, -0.5f);
    EXPECT_FLOAT_EQ(output.yaw, 1.0f);

    output = limiter.update(750U, {1.0f, -1.0f, -1.0f});
    EXPECT_FLOAT_EQ(output.surge, 0.375f);
    EXPECT_FLOAT_EQ(output.sway, -0.75f);
    EXPECT_FLOAT_EQ(output.yaw, 0.5f);
}

TEST(MiniDPAxisLimiter, ResetClearsStoredCommand)
{
    MiniDP_AxisLimiter limiter;
    limiter.init(0U);

    MiniDP_AxisLimiterConfig config = limiter.config();
    config.surge_slew_rate = 0.5f;
    limiter.set_config(config);

    (void)limiter.update(1000U, {1.0f, 0.0f, 0.0f});
    limiter.reset(2000U);

    EXPECT_FLOAT_EQ(limiter.command().surge, 0.0f);

    const MiniDP_AxisCommand output =
        limiter.update(2200U, {1.0f, 0.0f, 0.0f});
    EXPECT_FLOAT_EQ(output.surge, 0.1f);
}

TEST(MiniDPAxisLimiter, SanitizesInvalidConfig)
{
    MiniDP_AxisLimiter limiter;
    limiter.init(0U);

    MiniDP_AxisLimiterConfig config = limiter.config();
    config.deadband = NAN;
    config.surge_limit = -1.0f;
    config.sway_limit = 2.0f;
    config.yaw_limit = INFINITY;
    config.surge_slew_rate = -1.0f;
    limiter.set_config(config);

    EXPECT_FLOAT_EQ(limiter.config().deadband, 0.0f);
    EXPECT_FLOAT_EQ(limiter.config().surge_limit, 0.0f);
    EXPECT_FLOAT_EQ(limiter.config().sway_limit, 1.0f);
    EXPECT_FLOAT_EQ(limiter.config().yaw_limit, 1.0f);
    EXPECT_FLOAT_EQ(limiter.config().surge_slew_rate, 0.0f);
}


TEST(MiniDPAxisLimiter, RefreshingConfigPreservesSubDeadbandSlewProgress)
{
    MiniDP_AxisLimiter limiter;
    limiter.init(0);
    auto config = limiter.config();
    config.deadband = 0.1f;
    config.surge_slew_rate = 0.5f;
    for (uint32_t ms = 2; ms <= 1000; ms += 2) {
        limiter.set_config(config);
        limiter.update(ms, {1, 0, 0});
    }
    EXPECT_NEAR(limiter.command().surge, 0.5f, 1.0e-5f);
}

AP_GTEST_MAIN()
