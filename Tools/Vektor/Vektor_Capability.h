#pragma once

#include <stdint.h>

namespace Vektor {

enum CapabilityFlag : uint32_t {
    CAP_NATIVE_USB = 1U << 0,
    CAP_PWM_OUTPUTS = 1U << 1,
    CAP_FLEX_TIMER_CHANNELS = 1U << 2,
    CAP_DEDICATED_RECEIVER_ROW = 1U << 3,
    CAP_EXTERNAL_UARTS = 1U << 4,
    CAP_CLASSIC_CAN = 1U << 5,
    CAP_ADC_OBSERVABLES = 1U << 6,
    CAP_STORAGE = 1U << 7,
    CAP_ONBOARD_IMU = 1U << 8,
    CAP_ONBOARD_COMPASS = 1U << 9,
    CAP_SD_STORAGE = 1U << 10,
    CAP_DATAFLASH_STORAGE = 1U << 11,
};

struct TimerGroup {
    const char *name;
    uint8_t channel_count;
};

struct BoardCapability {
    const char *product_name;
    const char *hardware_revision;
    const char *mcu_name;
    uint32_t board_id;
    uint8_t pwm_outputs;
    uint8_t flex_timer_channels;
    uint8_t dedicated_receiver_rows;
    uint8_t uart_endpoints;
    uint8_t can_ports;
    uint8_t adc_observables;
    uint8_t storage_areas;
    const char *imu_name;
    const char *compass_name;
    uint32_t flags;
    const TimerGroup *pwm_timer_groups;
    uint8_t pwm_timer_group_count;
    const TimerGroup *flex_timer_groups;
    uint8_t flex_timer_group_count;
    int16_t heartbeat_led_gpio;
};

const BoardCapability &default_capability_for_build();
bool capability_has(const BoardCapability &capability, CapabilityFlag flag);

} // namespace Vektor
