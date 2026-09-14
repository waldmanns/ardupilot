#pragma once

#include "Vektor_Protocol.h"

#include <stdint.h>

/*
 * Programming-time names for the built-in Vektor Serial Protocol schema.
 *
 * GET, SET, SUBSCRIBE, and ROUTE_* carry the stable u32 ID, not the text path
 * or AP_Param name.  These references provide all three where applicable.
 * A client should still use DESCRIBE as the runtime source of truth because a
 * newer firmware can expose a different schema or board capability set.
 */
namespace Vektor {
namespace SerialCatalog {

struct Reference {
    const char *path;
    uint32_t id;
    Protocol::PrimitiveType type;

    constexpr explicit Reference(
        const char *canonical_path,
        Protocol::PrimitiveType primitive_type = Protocol::PrimitiveType::INVALID) :
        path(canonical_path),
        id(Protocol::fnv1a32_constexpr(canonical_path)),
        type(primitive_type)
    {}
};

struct ParameterReference {
    const char *path;
    const char *ap_param_name;
    uint32_t id;
    Protocol::PrimitiveType type;

    constexpr ParameterReference(const char *canonical_path,
                                 const char *short_name,
                                 Protocol::PrimitiveType primitive_type) :
        path(canonical_path),
        ap_param_name(short_name),
        id(Protocol::fnv1a32_constexpr(canonical_path)),
        type(primitive_type)
    {}
};

namespace Component {

static constexpr Reference PROTOCOL("component/system/0");
static constexpr Reference RUNTIME("component/system/1");
static constexpr Reference ATTITUDE("component/attitude/0");
static constexpr Reference VSP("component/vsp/1");
static constexpr Reference RCIN("component/rcin/0");
static constexpr Reference PWM_INPUT("component/pwm_input/0");
static constexpr Reference PWM_OUTPUT("component/pwm_output/0");

static constexpr const Reference *ALL[] = {
    &PROTOCOL,
    &RUNTIME,
    &ATTITUDE,
    &VSP,
    &RCIN,
    &PWM_INPUT,
    &PWM_OUTPUT,
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

} // namespace Component

namespace Parameter {

static constexpr ParameterReference SYS_OPTIONS(
    "component/system/0/parameter/sys_options",
    "SYS_OPTIONS",
    Protocol::PrimitiveType::I32);
static constexpr ParameterReference SYS_DESC_PAGE(
    "component/system/0/parameter/sys_desc_page",
    "SYS_DESC_PAGE",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference SYS_PROTOCOL_BAUD(
    "component/system/0/parameter/sys_protocol_baud",
    "SYS_PROTO_BAUD",
    Protocol::PrimitiveType::I32);
static constexpr ParameterReference RCIN_UART_PORT(
    "component/rcin/0/parameter/uart_port",
    "RCIN_PORT",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference RCIN_TIMEOUT_MS(
    "component/rcin/0/parameter/timeout_ms",
    "RCIN_TIMEOUT",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference RCIN_PROTOCOL_MASK(
    "component/rcin/0/parameter/protocol_mask",
    "RC_PROTOCOLS",
    Protocol::PrimitiveType::I32);

#define VEKTOR_PWMIN_PIN_PARAMETER(CHANNEL)                                  \
    static constexpr ParameterReference PWMIN_CHANNEL_##CHANNEL##_PIN(       \
        "component/pwm_input/0/parameter/channel_" #CHANNEL "_pin",       \
        "PWIN" #CHANNEL "_PIN",                                           \
        Protocol::PrimitiveType::I16)

VEKTOR_PWMIN_PIN_PARAMETER(1);
VEKTOR_PWMIN_PIN_PARAMETER(2);
VEKTOR_PWMIN_PIN_PARAMETER(3);
VEKTOR_PWMIN_PIN_PARAMETER(4);
VEKTOR_PWMIN_PIN_PARAMETER(5);
VEKTOR_PWMIN_PIN_PARAMETER(6);

#undef VEKTOR_PWMIN_PIN_PARAMETER

static constexpr ParameterReference PWMIN_TIMEOUT_MS(
    "component/pwm_input/0/parameter/timeout_ms",
    "PWIN_TIMEOUT",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference PWMIN_MINIMUM_US(
    "component/pwm_input/0/parameter/minimum_us",
    "PWIN_MIN",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference PWMIN_TRIM_US(
    "component/pwm_input/0/parameter/trim_us",
    "PWIN_TRIM",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference PWMIN_MAXIMUM_US(
    "component/pwm_input/0/parameter/maximum_us",
    "PWIN_MAX",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference PWMOUT_RATE_HZ(
    "component/pwm_output/0/parameter/rate_hz",
    "PWM_RATE",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference PWMOUT_MINIMUM_US(
    "component/pwm_output/0/parameter/minimum_us",
    "PWM_MIN",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference PWMOUT_TRIM_US(
    "component/pwm_output/0/parameter/trim_us",
    "PWM_TRIM",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference PWMOUT_MAXIMUM_US(
    "component/pwm_output/0/parameter/maximum_us",
    "PWM_MAX",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference PWMOUT_REVERSE_MASK(
    "component/pwm_output/0/parameter/reverse_mask",
    "PWM_REVERSE",
    Protocol::PrimitiveType::I16);
static constexpr ParameterReference PWMOUT_FAILSAFE_US(
    "component/pwm_output/0/parameter/failsafe_us",
    "PWM_FAILSAFE",
    Protocol::PrimitiveType::I16);

static constexpr const ParameterReference *PWMIN_CHANNEL_PINS[] = {
    &PWMIN_CHANNEL_1_PIN,
    &PWMIN_CHANNEL_2_PIN,
    &PWMIN_CHANNEL_3_PIN,
    &PWMIN_CHANNEL_4_PIN,
    &PWMIN_CHANNEL_5_PIN,
    &PWMIN_CHANNEL_6_PIN,
};
static constexpr uint8_t PWMIN_CHANNEL_PIN_COUNT =
    sizeof(PWMIN_CHANNEL_PINS) / sizeof(PWMIN_CHANNEL_PINS[0]);

static constexpr const ParameterReference *ALL[] = {
    &SYS_OPTIONS,
    &SYS_DESC_PAGE,
    &SYS_PROTOCOL_BAUD,
    &RCIN_UART_PORT,
    &RCIN_TIMEOUT_MS,
    &RCIN_PROTOCOL_MASK,
    &PWMIN_CHANNEL_1_PIN,
    &PWMIN_CHANNEL_2_PIN,
    &PWMIN_CHANNEL_3_PIN,
    &PWMIN_CHANNEL_4_PIN,
    &PWMIN_CHANNEL_5_PIN,
    &PWMIN_CHANNEL_6_PIN,
    &PWMIN_TIMEOUT_MS,
    &PWMIN_MINIMUM_US,
    &PWMIN_TRIM_US,
    &PWMIN_MAXIMUM_US,
    &PWMOUT_RATE_HZ,
    &PWMOUT_MINIMUM_US,
    &PWMOUT_TRIM_US,
    &PWMOUT_MAXIMUM_US,
    &PWMOUT_REVERSE_MASK,
    &PWMOUT_FAILSAFE_US,
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

} // namespace Parameter

namespace Observable {

static constexpr Reference RX_FRAMES(
    "component/system/0/observable/rx_frames", Protocol::PrimitiveType::U32);
static constexpr Reference RX_DROPS(
    "component/system/0/observable/rx_drops", Protocol::PrimitiveType::U32);
static constexpr Reference TX_DROPS(
    "component/system/0/observable/tx_drops", Protocol::PrimitiveType::U32);
static constexpr Reference UPTIME_MS(
    "component/system/1/observable/uptime_ms", Protocol::PrimitiveType::U32);
static constexpr Reference LOOP_COUNT(
    "component/system/1/observable/loop_count", Protocol::PrimitiveType::U32);
static constexpr Reference LOOP_DT_US(
    "component/system/1/observable/loop_dt_us", Protocol::PrimitiveType::U32);
static constexpr Reference LOOP_WORK_US(
    "component/system/1/observable/loop_work_us", Protocol::PrimitiveType::U32);
static constexpr Reference LOOP_MAX_WORK_US(
    "component/system/1/observable/loop_max_work_us",
    Protocol::PrimitiveType::U32);
static constexpr Reference SERVICE_RATE_HZ(
    "component/system/1/observable/service_rate_hz",
    Protocol::PrimitiveType::U16);
static constexpr Reference ATTITUDE_ROLL_DEG(
    "component/attitude/0/observable/roll_deg",
    Protocol::PrimitiveType::FLOAT32);
static constexpr Reference ATTITUDE_PITCH_DEG(
    "component/attitude/0/observable/pitch_deg",
    Protocol::PrimitiveType::FLOAT32);
static constexpr Reference ATTITUDE_YAW_DEG(
    "component/attitude/0/observable/yaw_deg",
    Protocol::PrimitiveType::FLOAT32);
static constexpr Reference ATTITUDE_QUATERNION(
    "component/attitude/0/observable/quaternion",
    Protocol::PrimitiveType::QUATERNIONF);
static constexpr Reference ATTITUDE_BODY_RATES_RAD_S(
    "component/attitude/0/observable/body_rates_rad_s",
    Protocol::PrimitiveType::VECTOR3F);

#define VEKTOR_RCIN_PWM_OBSERVABLE(CHANNEL)                                 \
    static constexpr Reference RCIN_CHANNEL_##CHANNEL##_US(                 \
        "component/rcin/0/observable/channel_" #CHANNEL "_us",            \
        Protocol::PrimitiveType::U16)

VEKTOR_RCIN_PWM_OBSERVABLE(1);
VEKTOR_RCIN_PWM_OBSERVABLE(2);
VEKTOR_RCIN_PWM_OBSERVABLE(3);
VEKTOR_RCIN_PWM_OBSERVABLE(4);
VEKTOR_RCIN_PWM_OBSERVABLE(5);
VEKTOR_RCIN_PWM_OBSERVABLE(6);
VEKTOR_RCIN_PWM_OBSERVABLE(7);
VEKTOR_RCIN_PWM_OBSERVABLE(8);
VEKTOR_RCIN_PWM_OBSERVABLE(9);
VEKTOR_RCIN_PWM_OBSERVABLE(10);
VEKTOR_RCIN_PWM_OBSERVABLE(11);
VEKTOR_RCIN_PWM_OBSERVABLE(12);
VEKTOR_RCIN_PWM_OBSERVABLE(13);
VEKTOR_RCIN_PWM_OBSERVABLE(14);
VEKTOR_RCIN_PWM_OBSERVABLE(15);
VEKTOR_RCIN_PWM_OBSERVABLE(16);

#undef VEKTOR_RCIN_PWM_OBSERVABLE

static constexpr const Reference *ATTITUDE[] = {
    &ATTITUDE_ROLL_DEG,
    &ATTITUDE_PITCH_DEG,
    &ATTITUDE_YAW_DEG,
    &ATTITUDE_QUATERNION,
    &ATTITUDE_BODY_RATES_RAD_S,
};
static constexpr uint8_t ATTITUDE_COUNT =
    sizeof(ATTITUDE) / sizeof(ATTITUDE[0]);

static constexpr const Reference *RCIN_CHANNELS_US[] = {
    &RCIN_CHANNEL_1_US,
    &RCIN_CHANNEL_2_US,
    &RCIN_CHANNEL_3_US,
    &RCIN_CHANNEL_4_US,
    &RCIN_CHANNEL_5_US,
    &RCIN_CHANNEL_6_US,
    &RCIN_CHANNEL_7_US,
    &RCIN_CHANNEL_8_US,
    &RCIN_CHANNEL_9_US,
    &RCIN_CHANNEL_10_US,
    &RCIN_CHANNEL_11_US,
    &RCIN_CHANNEL_12_US,
    &RCIN_CHANNEL_13_US,
    &RCIN_CHANNEL_14_US,
    &RCIN_CHANNEL_15_US,
    &RCIN_CHANNEL_16_US,
};
static constexpr uint8_t RCIN_CHANNEL_US_COUNT =
    sizeof(RCIN_CHANNELS_US) / sizeof(RCIN_CHANNELS_US[0]);

static constexpr const Reference *ALL[] = {
    &RX_FRAMES,
    &RX_DROPS,
    &TX_DROPS,
    &UPTIME_MS,
    &LOOP_COUNT,
    &LOOP_DT_US,
    &LOOP_WORK_US,
    &LOOP_MAX_WORK_US,
    &SERVICE_RATE_HZ,
    &ATTITUDE_ROLL_DEG,
    &ATTITUDE_PITCH_DEG,
    &ATTITUDE_YAW_DEG,
    &ATTITUDE_QUATERNION,
    &ATTITUDE_BODY_RATES_RAD_S,
    &RCIN_CHANNEL_1_US,
    &RCIN_CHANNEL_2_US,
    &RCIN_CHANNEL_3_US,
    &RCIN_CHANNEL_4_US,
    &RCIN_CHANNEL_5_US,
    &RCIN_CHANNEL_6_US,
    &RCIN_CHANNEL_7_US,
    &RCIN_CHANNEL_8_US,
    &RCIN_CHANNEL_9_US,
    &RCIN_CHANNEL_10_US,
    &RCIN_CHANNEL_11_US,
    &RCIN_CHANNEL_12_US,
    &RCIN_CHANNEL_13_US,
    &RCIN_CHANNEL_14_US,
    &RCIN_CHANNEL_15_US,
    &RCIN_CHANNEL_16_US,
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

} // namespace Observable

namespace Input {

static constexpr Reference VSP_X(
    "component/vsp/1/input/x", Protocol::PrimitiveType::FLOAT32);
static constexpr Reference VSP_Y(
    "component/vsp/1/input/y", Protocol::PrimitiveType::FLOAT32);

#define VEKTOR_PWMOUT_INPUT(CHANNEL)                                         \
    static constexpr Reference PWMOUT_CHANNEL_##CHANNEL(                     \
        "component/pwm_output/0/input/channel_" #CHANNEL,                  \
        Protocol::PrimitiveType::FLOAT32)

VEKTOR_PWMOUT_INPUT(1);
VEKTOR_PWMOUT_INPUT(2);
VEKTOR_PWMOUT_INPUT(3);
VEKTOR_PWMOUT_INPUT(4);
VEKTOR_PWMOUT_INPUT(5);
VEKTOR_PWMOUT_INPUT(6);
VEKTOR_PWMOUT_INPUT(7);
VEKTOR_PWMOUT_INPUT(8);
VEKTOR_PWMOUT_INPUT(9);
VEKTOR_PWMOUT_INPUT(10);
VEKTOR_PWMOUT_INPUT(11);
VEKTOR_PWMOUT_INPUT(12);

#undef VEKTOR_PWMOUT_INPUT

static constexpr const Reference *PWMOUT_CHANNELS[] = {
    &PWMOUT_CHANNEL_1,
    &PWMOUT_CHANNEL_2,
    &PWMOUT_CHANNEL_3,
    &PWMOUT_CHANNEL_4,
    &PWMOUT_CHANNEL_5,
    &PWMOUT_CHANNEL_6,
    &PWMOUT_CHANNEL_7,
    &PWMOUT_CHANNEL_8,
    &PWMOUT_CHANNEL_9,
    &PWMOUT_CHANNEL_10,
    &PWMOUT_CHANNEL_11,
    &PWMOUT_CHANNEL_12,
};
static constexpr uint8_t PWMOUT_CHANNEL_COUNT =
    sizeof(PWMOUT_CHANNELS) / sizeof(PWMOUT_CHANNELS[0]);

static constexpr const Reference *ALL[] = {
    &VSP_X,
    &VSP_Y,
    &PWMOUT_CHANNEL_1,
    &PWMOUT_CHANNEL_2,
    &PWMOUT_CHANNEL_3,
    &PWMOUT_CHANNEL_4,
    &PWMOUT_CHANNEL_5,
    &PWMOUT_CHANNEL_6,
    &PWMOUT_CHANNEL_7,
    &PWMOUT_CHANNEL_8,
    &PWMOUT_CHANNEL_9,
    &PWMOUT_CHANNEL_10,
    &PWMOUT_CHANNEL_11,
    &PWMOUT_CHANNEL_12,
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

} // namespace Input

namespace Output {

static constexpr Reference VSP_SERVO_A(
    "component/vsp/1/output/servo_a", Protocol::PrimitiveType::FLOAT32);
static constexpr Reference VSP_SERVO_B(
    "component/vsp/1/output/servo_b", Protocol::PrimitiveType::FLOAT32);

#define VEKTOR_RCIN_OUTPUT(CHANNEL)                                          \
    static constexpr Reference RCIN_CHANNEL_##CHANNEL(                       \
        "component/rcin/0/output/channel_" #CHANNEL,                       \
        Protocol::PrimitiveType::FLOAT32)

VEKTOR_RCIN_OUTPUT(1);
VEKTOR_RCIN_OUTPUT(2);
VEKTOR_RCIN_OUTPUT(3);
VEKTOR_RCIN_OUTPUT(4);
VEKTOR_RCIN_OUTPUT(5);
VEKTOR_RCIN_OUTPUT(6);
VEKTOR_RCIN_OUTPUT(7);
VEKTOR_RCIN_OUTPUT(8);
VEKTOR_RCIN_OUTPUT(9);
VEKTOR_RCIN_OUTPUT(10);
VEKTOR_RCIN_OUTPUT(11);
VEKTOR_RCIN_OUTPUT(12);
VEKTOR_RCIN_OUTPUT(13);
VEKTOR_RCIN_OUTPUT(14);
VEKTOR_RCIN_OUTPUT(15);
VEKTOR_RCIN_OUTPUT(16);

#undef VEKTOR_RCIN_OUTPUT

#define VEKTOR_PWMIN_OUTPUT(CHANNEL)                                         \
    static constexpr Reference PWMIN_CHANNEL_##CHANNEL(                      \
        "component/pwm_input/0/output/channel_" #CHANNEL,                  \
        Protocol::PrimitiveType::FLOAT32)

VEKTOR_PWMIN_OUTPUT(1);
VEKTOR_PWMIN_OUTPUT(2);
VEKTOR_PWMIN_OUTPUT(3);
VEKTOR_PWMIN_OUTPUT(4);
VEKTOR_PWMIN_OUTPUT(5);
VEKTOR_PWMIN_OUTPUT(6);

#undef VEKTOR_PWMIN_OUTPUT

#define VEKTOR_PWMOUT_OUTPUT(CHANNEL)                                       \
    static constexpr Reference PWMOUT_CHANNEL_##CHANNEL(                    \
        "component/pwm_output/0/output/channel_" #CHANNEL,                 \
        Protocol::PrimitiveType::U16)

VEKTOR_PWMOUT_OUTPUT(1);
VEKTOR_PWMOUT_OUTPUT(2);
VEKTOR_PWMOUT_OUTPUT(3);
VEKTOR_PWMOUT_OUTPUT(4);
VEKTOR_PWMOUT_OUTPUT(5);
VEKTOR_PWMOUT_OUTPUT(6);
VEKTOR_PWMOUT_OUTPUT(7);
VEKTOR_PWMOUT_OUTPUT(8);
VEKTOR_PWMOUT_OUTPUT(9);
VEKTOR_PWMOUT_OUTPUT(10);
VEKTOR_PWMOUT_OUTPUT(11);
VEKTOR_PWMOUT_OUTPUT(12);

#undef VEKTOR_PWMOUT_OUTPUT

static constexpr const Reference *RCIN_CHANNELS[] = {
    &RCIN_CHANNEL_1,
    &RCIN_CHANNEL_2,
    &RCIN_CHANNEL_3,
    &RCIN_CHANNEL_4,
    &RCIN_CHANNEL_5,
    &RCIN_CHANNEL_6,
    &RCIN_CHANNEL_7,
    &RCIN_CHANNEL_8,
    &RCIN_CHANNEL_9,
    &RCIN_CHANNEL_10,
    &RCIN_CHANNEL_11,
    &RCIN_CHANNEL_12,
    &RCIN_CHANNEL_13,
    &RCIN_CHANNEL_14,
    &RCIN_CHANNEL_15,
    &RCIN_CHANNEL_16,
};
static constexpr uint8_t RCIN_CHANNEL_COUNT =
    sizeof(RCIN_CHANNELS) / sizeof(RCIN_CHANNELS[0]);

static constexpr const Reference *PWMIN_CHANNELS[] = {
    &PWMIN_CHANNEL_1,
    &PWMIN_CHANNEL_2,
    &PWMIN_CHANNEL_3,
    &PWMIN_CHANNEL_4,
    &PWMIN_CHANNEL_5,
    &PWMIN_CHANNEL_6,
};
static constexpr uint8_t PWMIN_CHANNEL_COUNT =
    sizeof(PWMIN_CHANNELS) / sizeof(PWMIN_CHANNELS[0]);

static constexpr const Reference *PWMOUT_CHANNELS[] = {
    &PWMOUT_CHANNEL_1,
    &PWMOUT_CHANNEL_2,
    &PWMOUT_CHANNEL_3,
    &PWMOUT_CHANNEL_4,
    &PWMOUT_CHANNEL_5,
    &PWMOUT_CHANNEL_6,
    &PWMOUT_CHANNEL_7,
    &PWMOUT_CHANNEL_8,
    &PWMOUT_CHANNEL_9,
    &PWMOUT_CHANNEL_10,
    &PWMOUT_CHANNEL_11,
    &PWMOUT_CHANNEL_12,
};
static constexpr uint8_t PWMOUT_CHANNEL_COUNT =
    sizeof(PWMOUT_CHANNELS) / sizeof(PWMOUT_CHANNELS[0]);

static constexpr const Reference *ALL[] = {
    &VSP_SERVO_A,
    &VSP_SERVO_B,
    &RCIN_CHANNEL_1,
    &RCIN_CHANNEL_2,
    &RCIN_CHANNEL_3,
    &RCIN_CHANNEL_4,
    &RCIN_CHANNEL_5,
    &RCIN_CHANNEL_6,
    &RCIN_CHANNEL_7,
    &RCIN_CHANNEL_8,
    &RCIN_CHANNEL_9,
    &RCIN_CHANNEL_10,
    &RCIN_CHANNEL_11,
    &RCIN_CHANNEL_12,
    &RCIN_CHANNEL_13,
    &RCIN_CHANNEL_14,
    &RCIN_CHANNEL_15,
    &RCIN_CHANNEL_16,
    &PWMIN_CHANNEL_1,
    &PWMIN_CHANNEL_2,
    &PWMIN_CHANNEL_3,
    &PWMIN_CHANNEL_4,
    &PWMIN_CHANNEL_5,
    &PWMIN_CHANNEL_6,
    &PWMOUT_CHANNEL_1,
    &PWMOUT_CHANNEL_2,
    &PWMOUT_CHANNEL_3,
    &PWMOUT_CHANNEL_4,
    &PWMOUT_CHANNEL_5,
    &PWMOUT_CHANNEL_6,
    &PWMOUT_CHANNEL_7,
    &PWMOUT_CHANNEL_8,
    &PWMOUT_CHANNEL_9,
    &PWMOUT_CHANNEL_10,
    &PWMOUT_CHANNEL_11,
    &PWMOUT_CHANNEL_12,
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

} // namespace Output

// Ready-made groups for constructing SUBSCRIBE payloads.  Each group is an
// ordered array of field references; append each .id in order and decode
// TELEMETRY values in that same order.
namespace Stream {

static constexpr const Reference *ATTITUDE[] = {
    &Observable::ATTITUDE_ROLL_DEG,
    &Observable::ATTITUDE_PITCH_DEG,
    &Observable::ATTITUDE_YAW_DEG,
    &Observable::ATTITUDE_QUATERNION,
    &Observable::ATTITUDE_BODY_RATES_RAD_S,
};
static constexpr uint8_t ATTITUDE_COUNT =
    sizeof(ATTITUDE) / sizeof(ATTITUDE[0]);

static constexpr const Reference *RC_INPUT_NORMALIZED[] = {
    &Output::RCIN_CHANNEL_1,
    &Output::RCIN_CHANNEL_2,
    &Output::RCIN_CHANNEL_3,
    &Output::RCIN_CHANNEL_4,
    &Output::RCIN_CHANNEL_5,
    &Output::RCIN_CHANNEL_6,
    &Output::RCIN_CHANNEL_7,
    &Output::RCIN_CHANNEL_8,
    &Output::RCIN_CHANNEL_9,
    &Output::RCIN_CHANNEL_10,
    &Output::RCIN_CHANNEL_11,
    &Output::RCIN_CHANNEL_12,
    &Output::RCIN_CHANNEL_13,
    &Output::RCIN_CHANNEL_14,
    &Output::RCIN_CHANNEL_15,
    &Output::RCIN_CHANNEL_16,
};
static constexpr uint8_t RC_INPUT_NORMALIZED_COUNT =
    sizeof(RC_INPUT_NORMALIZED) / sizeof(RC_INPUT_NORMALIZED[0]);

static constexpr const Reference *RC_INPUT_PWM_US[] = {
    &Observable::RCIN_CHANNEL_1_US,
    &Observable::RCIN_CHANNEL_2_US,
    &Observable::RCIN_CHANNEL_3_US,
    &Observable::RCIN_CHANNEL_4_US,
    &Observable::RCIN_CHANNEL_5_US,
    &Observable::RCIN_CHANNEL_6_US,
    &Observable::RCIN_CHANNEL_7_US,
    &Observable::RCIN_CHANNEL_8_US,
    &Observable::RCIN_CHANNEL_9_US,
    &Observable::RCIN_CHANNEL_10_US,
    &Observable::RCIN_CHANNEL_11_US,
    &Observable::RCIN_CHANNEL_12_US,
    &Observable::RCIN_CHANNEL_13_US,
    &Observable::RCIN_CHANNEL_14_US,
    &Observable::RCIN_CHANNEL_15_US,
    &Observable::RCIN_CHANNEL_16_US,
};
static constexpr uint8_t RC_INPUT_PWM_US_COUNT =
    sizeof(RC_INPUT_PWM_US) / sizeof(RC_INPUT_PWM_US[0]);

static constexpr const Reference *RC_OUTPUT_PWM_US[] = {
    &Output::PWMOUT_CHANNEL_1,
    &Output::PWMOUT_CHANNEL_2,
    &Output::PWMOUT_CHANNEL_3,
    &Output::PWMOUT_CHANNEL_4,
    &Output::PWMOUT_CHANNEL_5,
    &Output::PWMOUT_CHANNEL_6,
    &Output::PWMOUT_CHANNEL_7,
    &Output::PWMOUT_CHANNEL_8,
    &Output::PWMOUT_CHANNEL_9,
    &Output::PWMOUT_CHANNEL_10,
    &Output::PWMOUT_CHANNEL_11,
    &Output::PWMOUT_CHANNEL_12,
};
static constexpr uint8_t RC_OUTPUT_PWM_US_COUNT =
    sizeof(RC_OUTPUT_PWM_US) / sizeof(RC_OUTPUT_PWM_US[0]);

static constexpr const Reference *RC_OUTPUT_COMMAND_NORMALIZED[] = {
    &Input::PWMOUT_CHANNEL_1,
    &Input::PWMOUT_CHANNEL_2,
    &Input::PWMOUT_CHANNEL_3,
    &Input::PWMOUT_CHANNEL_4,
    &Input::PWMOUT_CHANNEL_5,
    &Input::PWMOUT_CHANNEL_6,
    &Input::PWMOUT_CHANNEL_7,
    &Input::PWMOUT_CHANNEL_8,
    &Input::PWMOUT_CHANNEL_9,
    &Input::PWMOUT_CHANNEL_10,
    &Input::PWMOUT_CHANNEL_11,
    &Input::PWMOUT_CHANNEL_12,
};
static constexpr uint8_t RC_OUTPUT_COMMAND_NORMALIZED_COUNT =
    sizeof(RC_OUTPUT_COMMAND_NORMALIZED) /
    sizeof(RC_OUTPUT_COMMAND_NORMALIZED[0]);

static constexpr const Reference *ATTITUDE_RC_IO[] = {
    &Observable::ATTITUDE_ROLL_DEG,
    &Observable::ATTITUDE_PITCH_DEG,
    &Observable::ATTITUDE_YAW_DEG,
    &Observable::ATTITUDE_QUATERNION,
    &Observable::ATTITUDE_BODY_RATES_RAD_S,
    &Observable::RCIN_CHANNEL_1_US,
    &Observable::RCIN_CHANNEL_2_US,
    &Observable::RCIN_CHANNEL_3_US,
    &Observable::RCIN_CHANNEL_4_US,
    &Observable::RCIN_CHANNEL_5_US,
    &Observable::RCIN_CHANNEL_6_US,
    &Observable::RCIN_CHANNEL_7_US,
    &Observable::RCIN_CHANNEL_8_US,
    &Observable::RCIN_CHANNEL_9_US,
    &Observable::RCIN_CHANNEL_10_US,
    &Observable::RCIN_CHANNEL_11_US,
    &Observable::RCIN_CHANNEL_12_US,
    &Observable::RCIN_CHANNEL_13_US,
    &Observable::RCIN_CHANNEL_14_US,
    &Observable::RCIN_CHANNEL_15_US,
    &Observable::RCIN_CHANNEL_16_US,
    &Output::PWMOUT_CHANNEL_1,
    &Output::PWMOUT_CHANNEL_2,
    &Output::PWMOUT_CHANNEL_3,
    &Output::PWMOUT_CHANNEL_4,
    &Output::PWMOUT_CHANNEL_5,
    &Output::PWMOUT_CHANNEL_6,
    &Output::PWMOUT_CHANNEL_7,
    &Output::PWMOUT_CHANNEL_8,
    &Output::PWMOUT_CHANNEL_9,
    &Output::PWMOUT_CHANNEL_10,
    &Output::PWMOUT_CHANNEL_11,
    &Output::PWMOUT_CHANNEL_12,
};
static constexpr uint8_t ATTITUDE_RC_IO_COUNT =
    sizeof(ATTITUDE_RC_IO) / sizeof(ATTITUDE_RC_IO[0]);

} // namespace Stream

static constexpr uint16_t FIELD_COUNT =
    Parameter::COUNT + Observable::COUNT + Input::COUNT + Output::COUNT;

} // namespace SerialCatalog
} // namespace Vektor
