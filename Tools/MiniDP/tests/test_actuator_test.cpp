#include <AP_gtest.h>

#include "ActuatorTest.h"
#include <math.h>

TEST(MiniDPActuatorTest, StartsPercentTest)
{
    MiniDP_ActuatorTest test;

    MiniDP_ActuatorTestRequest request{};
    request.actuator_index = 2;
    request.throttle_type =
        uint8_t(MiniDP_ActuatorTestThrottleType::PERCENT);
    request.throttle_value = -25.0f;
    request.timeout_s = 1.5f;

    const auto result = test.start(100U, request);

    EXPECT_TRUE(result.accepted);
    EXPECT_TRUE(test.active());
    EXPECT_EQ(test.end_ms(), 1600U);
    EXPECT_TRUE(test.command().active);
    EXPECT_EQ(test.command().actuator_index, 2U);
    EXPECT_FALSE(test.command().use_pwm);
    EXPECT_FLOAT_EQ(test.command().demand, -0.25f);
}

TEST(MiniDPActuatorTest, StartsPwmTest)
{
    MiniDP_ActuatorTest test;

    MiniDP_ActuatorTestRequest request{};
    request.actuator_index = 0;
    request.throttle_type = uint8_t(MiniDP_ActuatorTestThrottleType::PWM);
    request.throttle_value = 1601.4f;
    request.timeout_s = 1.0f;

    const auto result = test.start(100U, request);

    EXPECT_TRUE(result.accepted);
    EXPECT_TRUE(test.command().use_pwm);
    EXPECT_EQ(test.command().pwm_us, 1601U);
}

TEST(MiniDPActuatorTest, RejectsBadRequests)
{
    MiniDP_ActuatorTest test;
    MiniDP_ActuatorTestRequest request{};
    request.actuator_index = 0;
    request.throttle_type =
        uint8_t(MiniDP_ActuatorTestThrottleType::PERCENT);
    request.throttle_value = 20.0f;
    request.timeout_s = 1.0f;

    request.actuator_index = MiniDP_OutputManager::max_actuators;
    EXPECT_EQ(
        test.start(0, request).rejection,
        MiniDP_ActuatorTestReject::INVALID_ACTUATOR);

    request.actuator_index = 0;
    request.throttle_type = 99;
    EXPECT_EQ(
        test.start(0, request).rejection,
        MiniDP_ActuatorTestReject::INVALID_THROTTLE_TYPE);

    request.throttle_type =
        uint8_t(MiniDP_ActuatorTestThrottleType::PERCENT);
    request.throttle_value = 101.0f;
    EXPECT_EQ(
        test.start(0, request).rejection,
        MiniDP_ActuatorTestReject::INVALID_THROTTLE_VALUE);

    request.throttle_type = uint8_t(MiniDP_ActuatorTestThrottleType::PWM);
    request.throttle_value = 799.0f;
    EXPECT_EQ(
        test.start(0, request).rejection,
        MiniDP_ActuatorTestReject::INVALID_THROTTLE_VALUE);

    request.throttle_value = 1500.0f;
    request.timeout_s = 0.0f;
    EXPECT_EQ(
        test.start(0, request).rejection,
        MiniDP_ActuatorTestReject::INVALID_TIMEOUT);
}

TEST(MiniDPActuatorTest, TimesOut)
{
    MiniDP_ActuatorTest test;
    MiniDP_ActuatorTestRequest request{};
    request.actuator_index = 0;
    request.throttle_type =
        uint8_t(MiniDP_ActuatorTestThrottleType::PERCENT);
    request.throttle_value = 20.0f;
    request.timeout_s = 0.1f;

    ASSERT_TRUE(test.start(100U, request).accepted);

    test.update(199U, true);
    EXPECT_TRUE(test.active());

    test.update(200U, true);
    EXPECT_FALSE(test.active());
    EXPECT_EQ(
        test.last_stop_reason(),
        MiniDP_ActuatorTestStopReason::TIMEOUT);
}

TEST(MiniDPActuatorTest, UnsafeStopsImmediately)
{
    MiniDP_ActuatorTest test;
    MiniDP_ActuatorTestRequest request{};
    request.actuator_index = 0;
    request.throttle_type =
        uint8_t(MiniDP_ActuatorTestThrottleType::PERCENT);
    request.throttle_value = 20.0f;
    request.timeout_s = 1.0f;

    ASSERT_TRUE(test.start(100U, request).accepted);
    test.update(101U, false);

    EXPECT_FALSE(test.active());
    EXPECT_EQ(
        test.last_stop_reason(),
        MiniDP_ActuatorTestStopReason::UNSAFE);
}

TEST(MiniDPActuatorTest, ReplacementRecordsStopReason)
{
    MiniDP_ActuatorTest test;
    MiniDP_ActuatorTestRequest request{};
    request.actuator_index = 0;
    request.throttle_type =
        uint8_t(MiniDP_ActuatorTestThrottleType::PERCENT);
    request.throttle_value = 20.0f;
    request.timeout_s = 1.0f;

    ASSERT_TRUE(test.start(100U, request).accepted);
    request.actuator_index = 1;
    ASSERT_TRUE(test.start(200U, request).accepted);

    EXPECT_TRUE(test.active());
    EXPECT_EQ(test.command().actuator_index, 1U);
    EXPECT_EQ(
        test.last_stop_reason(),
        MiniDP_ActuatorTestStopReason::REPLACED);
}


TEST(MiniDPActuatorTest, NonfiniteRequestCannotReplaceRunningTest)
{
    MiniDP_ActuatorTest test;
    MiniDP_ActuatorTestRequest request{};
    request.timeout_s = 1.0f;
    request.throttle_value = 25.0f;
    ASSERT_TRUE(test.start(100, request).accepted);
    const float bad[] = {NAN, INFINITY, -INFINITY};
    for (float value : bad) {
        request.timeout_s = value;
        EXPECT_FALSE(test.start(200, request).accepted);
        request.timeout_s = 1.0f;
        request.throttle_value = value;
        EXPECT_FALSE(test.start(200, request).accepted);
        request.throttle_value = 25.0f;
        EXPECT_TRUE(test.active());
        EXPECT_EQ(test.end_ms(), 1100U);
        EXPECT_FLOAT_EQ(test.command().demand, 0.25f);
    }
}

AP_GTEST_MAIN()
