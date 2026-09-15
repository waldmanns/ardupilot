#include "Vektor_Capability.h"
#include "Vektor_PwmInput.h"
#include "Vektor_PwmOutput.h"

#include <AP_HAL/AP_HAL_Boards.h>

/*
 * hwdef supplies the physical board facts.  Values which are safe to infer
 * use the generated HAL macros; connector-specific values default to zero so
 * an unannotated board never advertises hardware merely because its MCU has a
 * matching peripheral.
 */
#ifndef VEKTOR_PRODUCT_NAME
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define VEKTOR_PRODUCT_NAME CHIBIOS_BOARD_NAME
#else
#define VEKTOR_PRODUCT_NAME HAL_BOARD_NAME
#endif
#endif

#ifndef VEKTOR_HARDWARE_REVISION
#define VEKTOR_HARDWARE_REVISION "unknown"
#endif

#ifndef VEKTOR_MCU_NAME
#define VEKTOR_MCU_NAME "unknown"
#endif

#ifndef VEKTOR_BOARD_ID
#ifdef APJ_BOARD_ID
#define VEKTOR_BOARD_ID APJ_BOARD_ID
#else
#define VEKTOR_BOARD_ID 0
#endif
#endif

#ifndef VEKTOR_PWM_OUTPUT_COUNT
#ifdef HAL_PWM_COUNT
#define VEKTOR_PWM_OUTPUT_COUNT HAL_PWM_COUNT
#else
#define VEKTOR_PWM_OUTPUT_COUNT 0
#endif
#endif

#ifndef VEKTOR_PWM_INPUT_COUNT
#define VEKTOR_PWM_INPUT_COUNT 0
#endif

#ifndef VEKTOR_FLEX_TIMER_CHANNEL_COUNT
#define VEKTOR_FLEX_TIMER_CHANNEL_COUNT 0
#endif

#ifndef VEKTOR_DEDICATED_RECEIVER_ROW_COUNT
#define VEKTOR_DEDICATED_RECEIVER_ROW_COUNT 0
#endif

#ifndef VEKTOR_UART_ENDPOINT_COUNT
#define VEKTOR_UART_ENDPOINT_COUNT 0
#endif

#ifndef VEKTOR_CAN_PORT_COUNT
#ifdef HAL_NUM_CAN_IFACES
#define VEKTOR_CAN_PORT_COUNT HAL_NUM_CAN_IFACES
#else
#define VEKTOR_CAN_PORT_COUNT 0
#endif
#endif

#ifndef VEKTOR_ADC_OBSERVABLE_COUNT
#define VEKTOR_ADC_OBSERVABLE_COUNT 0
#endif

#ifndef VEKTOR_HAS_NATIVE_USB
#if defined(HAL_USE_SERIAL_USB) && HAL_USE_SERIAL_USB
#define VEKTOR_HAS_NATIVE_USB 1
#else
#define VEKTOR_HAS_NATIVE_USB 0
#endif
#endif

#ifndef VEKTOR_HAS_ONBOARD_IMU
#ifdef HAL_INS_PROBE_LIST
#define VEKTOR_HAS_ONBOARD_IMU 1
#else
#define VEKTOR_HAS_ONBOARD_IMU 0
#endif
#endif

#ifndef VEKTOR_HAS_ONBOARD_COMPASS
#ifdef HAL_MAG_PROBE_LIST
#define VEKTOR_HAS_ONBOARD_COMPASS 1
#else
#define VEKTOR_HAS_ONBOARD_COMPASS 0
#endif
#endif

#ifndef VEKTOR_IMU_NAME
#define VEKTOR_IMU_NAME "Onboard IMU"
#endif

#ifndef VEKTOR_COMPASS_NAME
#define VEKTOR_COMPASS_NAME "Onboard Compass"
#endif

#ifndef VEKTOR_HAS_SD_STORAGE
#if (defined(HAL_WITH_SPI_SDCARD) && HAL_WITH_SPI_SDCARD) || \
    (defined(HAL_USE_SDC) && HAL_USE_SDC)
#define VEKTOR_HAS_SD_STORAGE 1
#else
#define VEKTOR_HAS_SD_STORAGE 0
#endif
#endif

#ifndef VEKTOR_HAS_DATAFLASH_STORAGE
#if defined(HAL_WITH_SPI_DATAFLASH) && HAL_WITH_SPI_DATAFLASH
#define VEKTOR_HAS_DATAFLASH_STORAGE 1
#else
#define VEKTOR_HAS_DATAFLASH_STORAGE 0
#endif
#endif

#ifndef VEKTOR_STORAGE_AREA_COUNT
#define VEKTOR_STORAGE_AREA_COUNT \
    (VEKTOR_HAS_SD_STORAGE + VEKTOR_HAS_DATAFLASH_STORAGE)
#endif

#ifndef VEKTOR_PWM_TIMER_GROUP_COUNT
#define VEKTOR_PWM_TIMER_GROUP_COUNT 0
#endif

#ifndef VEKTOR_FLEX_TIMER_GROUP_COUNT
#define VEKTOR_FLEX_TIMER_GROUP_COUNT 0
#endif

#if VEKTOR_FLEX_TIMER_CHANNEL_COUNT > 0
#ifndef VEKTOR_FLEX_MODE_FLAGS
#error "VEKTOR_FLEX_MODE_FLAGS is required when Flex channels are advertised"
#endif
constexpr uint8_t flex_mode_flags[] = { VEKTOR_FLEX_MODE_FLAGS };
static_assert(sizeof(flex_mode_flags) / sizeof(flex_mode_flags[0]) ==
                  VEKTOR_FLEX_TIMER_CHANNEL_COUNT,
              "Vektor Flex mode count does not match its channel count");
#define VEKTOR_FLEX_MODE_PTR flex_mode_flags
#else
constexpr const uint8_t *flex_mode_flags = nullptr;
#define VEKTOR_FLEX_MODE_PTR flex_mode_flags
#endif

#if VEKTOR_UART_ENDPOINT_COUNT > 0
#ifndef VEKTOR_UART_ENDPOINT_FLAGS
#error "VEKTOR_UART_ENDPOINT_FLAGS is required when UART endpoints are advertised"
#endif
constexpr uint8_t uart_endpoint_flags[] = { VEKTOR_UART_ENDPOINT_FLAGS };
static_assert(sizeof(uart_endpoint_flags) / sizeof(uart_endpoint_flags[0]) ==
                  VEKTOR_UART_ENDPOINT_COUNT,
              "Vektor UART endpoint flag count does not match its endpoint count");
#define VEKTOR_UART_ENDPOINT_FLAG_PTR uart_endpoint_flags
#else
constexpr const uint8_t *uart_endpoint_flags = nullptr;
#define VEKTOR_UART_ENDPOINT_FLAG_PTR uart_endpoint_flags
#endif

#ifndef VEKTOR_PROTOCOL_TRANSPORT_KIND
#if VEKTOR_HAS_NATIVE_USB
#define VEKTOR_PROTOCOL_TRANSPORT_KIND 1
#else
#define VEKTOR_PROTOCOL_TRANSPORT_KIND 0
#endif
#endif

#ifndef VEKTOR_PROTOCOL_UART_ENDPOINT
#define VEKTOR_PROTOCOL_UART_ENDPOINT -1
#endif

#ifndef VEKTOR_HEARTBEAT_LED_GPIO
#define VEKTOR_HEARTBEAT_LED_GPIO -1
#endif

namespace {

#if VEKTOR_PWM_TIMER_GROUP_COUNT > 0
#ifndef VEKTOR_PWM_TIMER_GROUPS
#error "VEKTOR_PWM_TIMER_GROUPS is required when timer groups are advertised"
#endif
constexpr Vektor::TimerGroup pwm_timer_groups[] = {
    VEKTOR_PWM_TIMER_GROUPS
};
static_assert(sizeof(pwm_timer_groups) / sizeof(pwm_timer_groups[0]) ==
                  VEKTOR_PWM_TIMER_GROUP_COUNT,
              "VEKTOR PWM timer group count does not match its hwdef list");
#define VEKTOR_PWM_TIMER_GROUP_PTR pwm_timer_groups
#else
constexpr const Vektor::TimerGroup *pwm_timer_groups = nullptr;
#define VEKTOR_PWM_TIMER_GROUP_PTR pwm_timer_groups
#endif

#if VEKTOR_FLEX_TIMER_GROUP_COUNT > 0
#ifndef VEKTOR_FLEX_TIMER_GROUPS
#error "VEKTOR_FLEX_TIMER_GROUPS is required when timer groups are advertised"
#endif
constexpr Vektor::TimerGroup flex_timer_groups[] = {
    VEKTOR_FLEX_TIMER_GROUPS
};
static_assert(sizeof(flex_timer_groups) / sizeof(flex_timer_groups[0]) ==
                  VEKTOR_FLEX_TIMER_GROUP_COUNT,
              "VEKTOR Flex timer group count does not match its hwdef list");
#define VEKTOR_FLEX_TIMER_GROUP_PTR flex_timer_groups
#else
constexpr const Vektor::TimerGroup *flex_timer_groups = nullptr;
#define VEKTOR_FLEX_TIMER_GROUP_PTR flex_timer_groups
#endif

static_assert(VEKTOR_PWM_OUTPUT_COUNT <= UINT8_MAX, "too many Vektor PWM outputs");
static_assert(VEKTOR_PWM_INPUT_COUNT <= UINT8_MAX, "too many Vektor PWM inputs");
static_assert(VEKTOR_FLEX_TIMER_CHANNEL_COUNT <= UINT8_MAX, "too many Vektor Flex channels");
static_assert(VEKTOR_DEDICATED_RECEIVER_ROW_COUNT <= UINT8_MAX, "too many Vektor receiver rows");
static_assert(VEKTOR_UART_ENDPOINT_COUNT <= UINT8_MAX, "too many Vektor UART endpoints");
static_assert(VEKTOR_CAN_PORT_COUNT <= UINT8_MAX, "too many Vektor CAN ports");
static_assert(VEKTOR_ADC_OBSERVABLE_COUNT <= UINT8_MAX, "too many Vektor ADC observables");
static_assert(VEKTOR_STORAGE_AREA_COUNT <= UINT8_MAX, "too many Vektor storage areas");
static_assert(VEKTOR_PWM_TIMER_GROUP_COUNT <= UINT8_MAX, "too many Vektor PWM timer groups");
static_assert(VEKTOR_FLEX_TIMER_GROUP_COUNT <= UINT8_MAX, "too many Vektor Flex timer groups");

constexpr uint32_t capability_flags =
    (VEKTOR_HAS_NATIVE_USB ? uint32_t(Vektor::CAP_NATIVE_USB) : 0U) |
    (VEKTOR_PWM_OUTPUT_COUNT > 0 ? uint32_t(Vektor::CAP_PWM_OUTPUTS) : 0U) |
    (VEKTOR_FLEX_TIMER_CHANNEL_COUNT > 0 ?
         uint32_t(Vektor::CAP_FLEX_TIMER_CHANNELS) : 0U) |
    (VEKTOR_DEDICATED_RECEIVER_ROW_COUNT > 0 ?
         uint32_t(Vektor::CAP_DEDICATED_RECEIVER_ROW) : 0U) |
    (VEKTOR_UART_ENDPOINT_COUNT > 0 ?
         uint32_t(Vektor::CAP_EXTERNAL_UARTS) : 0U) |
    (VEKTOR_CAN_PORT_COUNT > 0 ? uint32_t(Vektor::CAP_CLASSIC_CAN) : 0U) |
    (VEKTOR_ADC_OBSERVABLE_COUNT > 0 ?
         uint32_t(Vektor::CAP_ADC_OBSERVABLES) : 0U) |
    (VEKTOR_STORAGE_AREA_COUNT > 0 ? uint32_t(Vektor::CAP_STORAGE) : 0U) |
    (VEKTOR_HAS_ONBOARD_IMU ? uint32_t(Vektor::CAP_ONBOARD_IMU) : 0U) |
    (VEKTOR_HAS_ONBOARD_COMPASS ?
         uint32_t(Vektor::CAP_ONBOARD_COMPASS) : 0U) |
    (VEKTOR_HAS_SD_STORAGE ? uint32_t(Vektor::CAP_SD_STORAGE) : 0U) |
    (VEKTOR_HAS_DATAFLASH_STORAGE ?
         uint32_t(Vektor::CAP_DATAFLASH_STORAGE) : 0U);

constexpr Vektor::BoardCapability build_capability {
    VEKTOR_PRODUCT_NAME,
    VEKTOR_HARDWARE_REVISION,
    VEKTOR_MCU_NAME,
    VEKTOR_BOARD_ID,
    VEKTOR_PWM_OUTPUT_COUNT,
    VEKTOR_FLEX_TIMER_CHANNEL_COUNT,
    VEKTOR_DEDICATED_RECEIVER_ROW_COUNT,
    VEKTOR_UART_ENDPOINT_COUNT,
    VEKTOR_CAN_PORT_COUNT,
    VEKTOR_ADC_OBSERVABLE_COUNT,
    VEKTOR_STORAGE_AREA_COUNT,
    VEKTOR_IMU_NAME,
    VEKTOR_COMPASS_NAME,
    capability_flags,
    VEKTOR_PWM_TIMER_GROUP_PTR,
    VEKTOR_PWM_TIMER_GROUP_COUNT,
    VEKTOR_FLEX_TIMER_GROUP_PTR,
    VEKTOR_FLEX_TIMER_GROUP_COUNT,
    VEKTOR_HEARTBEAT_LED_GPIO,
    VEKTOR_PWM_INPUT_COUNT,
    VEKTOR_FLEX_MODE_PTR,
    VEKTOR_UART_ENDPOINT_FLAG_PTR,
    Vektor::ProtocolTransport(VEKTOR_PROTOCOL_TRANSPORT_KIND),
    VEKTOR_PROTOCOL_UART_ENDPOINT,
};

} // namespace

namespace Vektor {

const BoardCapability &default_capability_for_build()
{
    return build_capability;
}

bool capability_has(const BoardCapability &capability, CapabilityFlag flag)
{
    return (capability.flags & uint32_t(flag)) != 0;
}

bool capability_valid(const BoardCapability &capability)
{
    if (capability.product_name == nullptr ||
        capability.hardware_revision == nullptr ||
        capability.mcu_name == nullptr ||
        capability.imu_name == nullptr ||
        capability.compass_name == nullptr ||
        capability.pwm_outputs > PwmOutput::max_channels ||
        capability.pwm_inputs > PwmInput::max_channels) {
        return false;
    }
    if ((capability.flex_timer_channels != 0 &&
         capability.flex_mode_flags == nullptr) ||
        (capability.uart_endpoints != 0 &&
         capability.uart_endpoint_flags == nullptr) ||
        (capability.pwm_timer_group_count != 0 &&
         capability.pwm_timer_groups == nullptr) ||
        (capability.flex_timer_group_count != 0 &&
         capability.flex_timer_groups == nullptr)) {
        return false;
    }

    uint16_t pwm_members = 0;
    for (uint8_t i = 0; i < capability.pwm_timer_group_count; i++) {
        const TimerGroup &group = capability.pwm_timer_groups[i];
        if (group.name == nullptr || group.channel_count == 0 ||
            group.supported_rate_flags == 0 ||
            (group.supported_rate_flags &
             ~(TIMER_RATE_50_HZ | TIMER_RATE_100_HZ |
               TIMER_RATE_200_HZ | TIMER_RATE_330_HZ)) != 0) {
            return false;
        }
        pwm_members += group.channel_count;
    }
    uint16_t flex_members = 0;
    for (uint8_t i = 0; i < capability.flex_timer_group_count; i++) {
        const TimerGroup &group = capability.flex_timer_groups[i];
        if (group.name == nullptr || group.channel_count == 0) {
            return false;
        }
        flex_members += group.channel_count;
    }
    if (pwm_members != capability.pwm_outputs ||
        flex_members != capability.flex_timer_channels) {
        return false;
    }
    for (uint8_t i = 0; i < capability.flex_timer_channels; i++) {
        if (capability.flex_mode_flags[i] == 0 ||
            (capability.flex_mode_flags[i] &
             ~(FLEX_PWM_INPUT | FLEX_PWM_OUTPUT | FLEX_RPM_CAPTURE |
               FLEX_ADC_INPUT | FLEX_DIGITAL_INPUT |
               FLEX_DIGITAL_OUTPUT)) != 0) {
            return false;
        }
    }
    for (uint8_t i = 0; i < capability.uart_endpoints; i++) {
        if (capability.uart_endpoint_flags[i] == 0 ||
            (capability.uart_endpoint_flags[i] &
             ~(ENDPOINT_INPUT | ENDPOINT_OUTPUT)) != 0) {
            return false;
        }
    }

    switch (capability.protocol_transport) {
    case ProtocolTransport::NONE:
        return capability.protocol_uart_endpoint < 0;
    case ProtocolTransport::USB:
        return capability_has(capability, CAP_NATIVE_USB) &&
               capability.protocol_uart_endpoint < 0;
    case ProtocolTransport::UART:
        return capability.protocol_uart_endpoint >= 0 &&
               capability.protocol_uart_endpoint < capability.uart_endpoints &&
               (capability.uart_endpoint_flags[
                    capability.protocol_uart_endpoint] &
                (ENDPOINT_INPUT | ENDPOINT_OUTPUT)) ==
                   (ENDPOINT_INPUT | ENDPOINT_OUTPUT);
    }
    return false;
}

} // namespace Vektor

#undef VEKTOR_FLEX_MODE_PTR
#undef VEKTOR_UART_ENDPOINT_FLAG_PTR
