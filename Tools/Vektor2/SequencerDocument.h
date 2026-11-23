#pragma once

#include "Sequencer.h"
#include <stddef.h>

namespace Vektor2 {
namespace SequencerDocument {

constexpr uint8_t schema_version = 3;
constexpr size_t header_size = 26;
constexpr size_t legacy_lane_size = 23;
constexpr size_t lane_size = legacy_lane_size + 2;
constexpr size_t block_size = 11;
constexpr size_t max_bytes = header_size + Sequencer::lane_count * lane_size +
    Sequencer::block_count * block_size;

enum class Decode : uint8_t { Valid, Invalid, Unsupported };

uint32_t checksum(const uint8_t* bytes, size_t length);
size_t encode(const Sequencer::Document& document, uint8_t* bytes);
Decode decode(const uint8_t* bytes, size_t length, Sequencer::Document& document);

} // namespace SequencerDocument
} // namespace Vektor2
