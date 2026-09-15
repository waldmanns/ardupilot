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

enum EndpointFlag : uint8_t {
    ENDPOINT_INPUT = 1U << 0,
    ENDPOINT_OUTPUT = 1U << 1,
    ENDPOINT_VEKTOR_TRANSPORT = 1U << 2,
};

enum FlexModeFlag : uint8_t {
    FLEX_PWM_INPUT = 1U << 0,
    FLEX_PWM_OUTPUT = 1U << 1,
    FLEX_RPM_CAPTURE = 1U << 2,
    FLEX_ADC_INPUT = 1U << 3,
    FLEX_DIGITAL_INPUT = 1U << 4,
    FLEX_DIGITAL_OUTPUT = 1U << 5,
};

enum TimerRateFlag : uint8_t {
    TIMER_RATE_50_HZ = 1U << 0,
    TIMER_RATE_100_HZ = 1U << 1,
    TIMER_RATE_200_HZ = 1U << 2,
    TIMER_RATE_330_HZ = 1U << 3,
};

enum class ProtocolTransport : uint8_t {
    NONE,
    USB,
    UART,
};

struct TimerGroup {
    const char *name;
    uint8_t channel_count;
    uint8_t supported_rate_flags;
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
    uint8_t pwm_inputs;
    const uint8_t *flex_mode_flags;
    const uint8_t *uart_endpoint_flags;
    ProtocolTransport protocol_transport;
    int8_t protocol_uart_endpoint;
};

const BoardCapability &default_capability_for_build();
bool capability_has(const BoardCapability &capability, CapabilityFlag flag);
bool capability_valid(const BoardCapability &capability);

} // namespace Vektor
