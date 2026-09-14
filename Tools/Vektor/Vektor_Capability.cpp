#include "Vektor_Capability.h"

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

} // namespace Vektor
