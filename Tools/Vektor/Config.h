#pragma once

#include <stdint.h>

#define VEKTOR_FIRMWARE_NAME "Vektor"
#define VEKTOR_TRUTH_BASE_FILE "Tools/Vektor/VEKTOR_CORE_EVO_TRUTH_BASE.md"

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
static constexpr int16_t default_describe_page_records = 4;
static constexpr int16_t min_describe_page_records = 1;
static constexpr int16_t max_describe_page_records = 16;
static constexpr int16_t default_rcin_port = 1;
static constexpr int16_t rcin_port_min = 0;
static constexpr int16_t rcin_port_max = 9;
static constexpr int16_t default_rcin_timeout_ms = 500;
static constexpr int16_t rcin_timeout_ms_min = 50;
static constexpr int16_t rcin_timeout_ms_max = 5000;
static constexpr int32_t rcin_protocol_mask_max = (1L << 17) - 1;

} // namespace Vektor
