#pragma once

#include "Output.h"

#include <stdint.h>

enum class MiniDP_ActuatorTestThrottleType : uint8_t {
    PERCENT = 0,
    PWM = 1,
};

enum class MiniDP_ActuatorTestReject : uint8_t {
    NONE = 0,
    NOT_AUTHORIZED,
    AUTHORITY_REJECTED,
    MODE_REJECTED,
    INVALID_MOTOR_COUNT,
    INVALID_TEST_ORDER,
    INVALID_ACTUATOR,
    INVALID_THROTTLE_TYPE,
    INVALID_THROTTLE_VALUE,
    INVALID_TIMEOUT,
};

enum class MiniDP_ActuatorTestStopReason : uint8_t {
    NONE = 0,
    TIMEOUT,
    UNSAFE,
    REPLACED,
    CANCELLED,
};

struct MiniDP_ActuatorTestRequest {
    uint8_t actuator_index;
    uint8_t throttle_type;
    float throttle_value;
    float timeout_s;
};

struct MiniDP_ActuatorTestStartResult {
    bool accepted;
    MiniDP_ActuatorTestReject rejection;
};

class MiniDP_ActuatorTest {
public:
    static constexpr uint32_t max_timeout_ms = 30000U;

    MiniDP_ActuatorTestStartResult start(
        uint32_t now_ms,
        const MiniDP_ActuatorTestRequest &request);
    void update(uint32_t now_ms, bool safe_to_continue);
    void cancel(MiniDP_ActuatorTestStopReason reason);

    bool active() const { return is_active; }
    uint32_t end_ms() const { return start_ms + timeout_ms; }
    MiniDP_ActuatorTestStopReason last_stop_reason() const { return stop_reason; }
    const MiniDP_ActuatorTestCommand &command() const { return output_command; }

    static const char *reject_name(MiniDP_ActuatorTestReject rejection);
    static const char *stop_reason_name(MiniDP_ActuatorTestStopReason reason);

private:
    bool is_active = false;
    uint32_t start_ms = 0;
    uint32_t timeout_ms = 0;
    MiniDP_ActuatorTestCommand output_command{};
    MiniDP_ActuatorTestStopReason stop_reason =
        MiniDP_ActuatorTestStopReason::NONE;
};
