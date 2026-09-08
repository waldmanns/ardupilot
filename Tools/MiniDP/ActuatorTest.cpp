#include "ActuatorTest.h"

#include <math.h>

namespace {

float absf(const float value)
{
    return value < 0.0f ? -value : value;
}

uint32_t timeout_ms_from_seconds(const float timeout_s)
{
    if (!isfinite(timeout_s) || timeout_s <= 0.0f) {
        return 0;
    }

    const float requested_ms = fminf(timeout_s, float(MiniDP_ActuatorTest::max_timeout_ms) * 0.001f) * 1000.0f;
    if (requested_ms >= float(MiniDP_ActuatorTest::max_timeout_ms)) {
        return MiniDP_ActuatorTest::max_timeout_ms;
    }
    if (requested_ms < 1.0f) {
        return 1U;
    }
    return uint32_t(requested_ms + 0.5f);
}

} // namespace

MiniDP_ActuatorTestStartResult MiniDP_ActuatorTest::start(
    const uint32_t now_ms,
    const MiniDP_ActuatorTestRequest &request)
{
    if (request.actuator_index >= MiniDP_OutputManager::max_actuators) {
        return {false, MiniDP_ActuatorTestReject::INVALID_ACTUATOR};
    }

    const uint32_t requested_timeout_ms =
        timeout_ms_from_seconds(request.timeout_s);
    if (requested_timeout_ms == 0U) {
        return {false, MiniDP_ActuatorTestReject::INVALID_TIMEOUT};
    }

    if (!isfinite(request.throttle_value)) {
        return {false, MiniDP_ActuatorTestReject::INVALID_THROTTLE_VALUE};
    }

    MiniDP_ActuatorTestCommand new_command{};
    new_command.active = true;
    new_command.actuator_index = request.actuator_index;

    switch (MiniDP_ActuatorTestThrottleType(request.throttle_type)) {
    case MiniDP_ActuatorTestThrottleType::PERCENT:
        if (absf(request.throttle_value) > 100.0f) {
            return {false, MiniDP_ActuatorTestReject::INVALID_THROTTLE_VALUE};
        }
        new_command.use_pwm = false;
        new_command.demand = request.throttle_value * 0.01f;
        break;

    case MiniDP_ActuatorTestThrottleType::PWM:
        if (request.throttle_value < 800.0f ||
            request.throttle_value > 2200.0f) {
            return {false, MiniDP_ActuatorTestReject::INVALID_THROTTLE_VALUE};
        }
        new_command.use_pwm = true;
        new_command.pwm_us = uint16_t(request.throttle_value + 0.5f);
        break;

    default:
        return {false, MiniDP_ActuatorTestReject::INVALID_THROTTLE_TYPE};
    }

    if (is_active) {
        stop_reason = MiniDP_ActuatorTestStopReason::REPLACED;
    } else {
        stop_reason = MiniDP_ActuatorTestStopReason::NONE;
    }
    is_active = true;
    start_ms = now_ms;
    timeout_ms = requested_timeout_ms;
    output_command = new_command;

    return {true, MiniDP_ActuatorTestReject::NONE};
}

void MiniDP_ActuatorTest::update(
    const uint32_t now_ms,
    const bool safe_to_continue)
{
    if (!is_active) {
        return;
    }

    if (!safe_to_continue) {
        cancel(MiniDP_ActuatorTestStopReason::UNSAFE);
        return;
    }

    if (now_ms - start_ms >= timeout_ms) {
        cancel(MiniDP_ActuatorTestStopReason::TIMEOUT);
    }
}

void MiniDP_ActuatorTest::cancel(const MiniDP_ActuatorTestStopReason reason)
{
    is_active = false;
    output_command = {};
    stop_reason = reason;
}

const char *MiniDP_ActuatorTest::reject_name(
    const MiniDP_ActuatorTestReject rejection)
{
    switch (rejection) {
    case MiniDP_ActuatorTestReject::NONE:
        return "none";
    case MiniDP_ActuatorTestReject::NOT_AUTHORIZED:
        return "not-authorized";
    case MiniDP_ActuatorTestReject::NOT_ARMED:
        return "not-armed";
    case MiniDP_ActuatorTestReject::AUTHORITY_REJECTED:
        return "authority-rejected";
    case MiniDP_ActuatorTestReject::MODE_REJECTED:
        return "mode-rejected";
    case MiniDP_ActuatorTestReject::INVALID_MOTOR_COUNT:
        return "invalid-motor-count";
    case MiniDP_ActuatorTestReject::INVALID_TEST_ORDER:
        return "invalid-test-order";
    case MiniDP_ActuatorTestReject::INVALID_ACTUATOR:
        return "invalid-actuator";
    case MiniDP_ActuatorTestReject::INVALID_THROTTLE_TYPE:
        return "invalid-throttle-type";
    case MiniDP_ActuatorTestReject::INVALID_THROTTLE_VALUE:
        return "invalid-throttle-value";
    case MiniDP_ActuatorTestReject::INVALID_TIMEOUT:
        return "invalid-timeout";
    }
    return "unknown";
}

const char *MiniDP_ActuatorTest::stop_reason_name(
    const MiniDP_ActuatorTestStopReason reason)
{
    switch (reason) {
    case MiniDP_ActuatorTestStopReason::NONE:
        return "none";
    case MiniDP_ActuatorTestStopReason::TIMEOUT:
        return "timeout";
    case MiniDP_ActuatorTestStopReason::UNSAFE:
        return "unsafe";
    case MiniDP_ActuatorTestStopReason::REPLACED:
        return "replaced";
    case MiniDP_ActuatorTestStopReason::CANCELLED:
        return "cancelled";
    }
    return "unknown";
}
