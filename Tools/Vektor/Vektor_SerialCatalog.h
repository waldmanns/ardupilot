#pragma once

#include "Vektor_Protocol.h"

#include <stdint.h>

/*
 * Programming-time names for the built-in Vektor Serial Protocol schema.
 *
 * Vektor_SerialCatalog.def is the canonical path/type binding manifest. These
 * declarations and their complete lookup arrays are generated from it by the
 * preprocessor, so adding a field cannot leave a hand-maintained ALL list
 * behind. Runtime clients must still use DESCRIBE because board capability
 * filtering and newer firmware can expose a different descriptor surface.
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

#define VEKTOR_COMPONENT(SYMBOL, PATH, ...) static constexpr Reference SYMBOL(PATH);
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...)
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...)
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT

static constexpr const Reference *ALL[] = {
#define VEKTOR_COMPONENT(SYMBOL, PATH, ...) &SYMBOL,
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...)
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...)
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

} // namespace Component

namespace Parameter {

#define VEKTOR_COMPONENT(SYMBOL, PATH, ...)
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...)                       \
    static constexpr ParameterReference SYMBOL(                             \
        PATH, AP_NAME, Protocol::PrimitiveType::TYPE);
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...)
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT

static constexpr const ParameterReference *ALL[] = {
#define VEKTOR_COMPONENT(SYMBOL, PATH, ...)
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...) &SYMBOL,
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...)
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

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

} // namespace Parameter

namespace Observable {

#define VEKTOR_COMPONENT(SYMBOL, PATH, ...)
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...)
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...)                               \
    static constexpr Reference SYMBOL(PATH, Protocol::PrimitiveType::TYPE);
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...)
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT

static constexpr const Reference *ALL[] = {
#define VEKTOR_COMPONENT(SYMBOL, PATH, ...)
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...)
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...) &SYMBOL,
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...)
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

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

} // namespace Observable

namespace Input {

#define VEKTOR_COMPONENT(SYMBOL, PATH, ...)
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...)
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...)                                    \
    static constexpr Reference SYMBOL(PATH, Protocol::PrimitiveType::TYPE);
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...)
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT

static constexpr const Reference *ALL[] = {
#define VEKTOR_COMPONENT(SYMBOL, PATH, ...)
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...)
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...) &SYMBOL,
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...)
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

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

} // namespace Input

namespace Output {

#define VEKTOR_COMPONENT(SYMBOL, PATH, ...)
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...)
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...)                                   \
    static constexpr Reference SYMBOL(PATH, Protocol::PrimitiveType::TYPE);
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT

static constexpr const Reference *ALL[] = {
#define VEKTOR_COMPONENT(SYMBOL, PATH, ...)
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, ...)
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, ...)
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, ...) &SYMBOL,
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT
};
static constexpr uint16_t COUNT = sizeof(ALL) / sizeof(ALL[0]);

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
