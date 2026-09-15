#pragma once

#include <AP_AHRS/AP_AHRS_config.h>

#include <stdint.h>

#define VEKTOR_FIRMWARE_NAME "Vektor"
#define VEKTOR_TRUTH_BASE_FILE "Tools/Vektor/VEKTOR_CORE_EVO_TRUTH_BASE.md"

#if AP_AHRS_DCM_ENABLED && AP_INERTIALSENSOR_ENABLED
#define VEKTOR_ATTITUDE_ENABLED 1
#else
#define VEKTOR_ATTITUDE_ENABLED 0
#endif

namespace Vektor {

static constexpr uint16_t firmware_major = 0;
static constexpr uint16_t firmware_minor = 1;
static constexpr uint16_t firmware_patch = 0;

static constexpr uint32_t protocol_baud = 115200;
static constexpr int32_t protocol_baud_min = 9600;
static constexpr int32_t protocol_baud_max = 921600;
static constexpr uint32_t protocol_rx_space = 512;
static constexpr uint32_t protocol_tx_space = 512;
static constexpr uint32_t default_service_rate_hz = 100;
static constexpr uint16_t max_realtime_rate_hz = default_service_rate_hz;
static constexpr uint16_t attitude_update_rate_hz =
#if VEKTOR_ATTITUDE_ENABLED
    default_service_rate_hz;
#else
    0;
#endif
static constexpr int16_t default_describe_page_records = 4;
static constexpr int16_t min_describe_page_records = 1;
static constexpr int16_t max_describe_page_records = 16;
static constexpr int16_t default_rcin_timeout_ms = 500;
static constexpr int16_t rcin_timeout_ms_min = 50;
static constexpr int16_t rcin_timeout_ms_max = 5000;
static constexpr int32_t rcin_protocol_mask_max = (1L << 17) - 1;
static constexpr int16_t default_pwmin_timeout_ms = 100;
static constexpr int16_t pwmin_timeout_ms_min = 20;
static constexpr int16_t pwmin_timeout_ms_max = 2000;
static constexpr int16_t pwm_input_valid_min_us = 800;
static constexpr int16_t pwm_input_valid_max_us = 2200;
static constexpr int16_t pwm_calibration_min_us = 800;
static constexpr int16_t pwm_calibration_max_us = 2200;
static constexpr int16_t default_pwm_min_us = 1000;
static constexpr int16_t default_pwm_trim_us = 1500;
static constexpr int16_t default_pwm_max_us = 2000;
static constexpr int16_t default_pwm_rate_hz = 50;
static constexpr int16_t pwm_reverse_mask_max = (1 << 12) - 1;

} // namespace Vektor
