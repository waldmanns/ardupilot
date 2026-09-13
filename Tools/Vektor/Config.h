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
static constexpr int16_t default_describe_page_records = 4;
static constexpr int16_t min_describe_page_records = 1;
static constexpr int16_t max_describe_page_records = 16;

} // namespace Vektor
