#include "SequencerDocument.h"
#include <string.h>

namespace Vektor2 {
namespace SequencerDocument {

static inline uint16_t read16(const uint8_t*& at)
{
    const uint16_t value = uint16_t(at[0]) | uint16_t(at[1]) << 8;
    at += 2;
    return value;
}

static inline uint32_t read32(const uint8_t*& at)
{
    const uint32_t low = read16(at);
    return low | uint32_t(read16(at)) << 16;
}

static inline void write16(uint8_t*& at, uint16_t value)
{
    *at++ = uint8_t(value);
    *at++ = uint8_t(value >> 8);
}

static inline void write32(uint8_t*& at, uint32_t value)
{
    write16(at, value);
    write16(at, value >> 16);
}

static Sequencer::Trigger read_trigger(const uint8_t*& at)
{
    Sequencer::Trigger trigger;
    trigger.edge = Sequencer::Edge(*at++);
    trigger.low = read16(at);
    trigger.high = read16(at);
    return trigger;
}

static void write_trigger(uint8_t*& at, const Sequencer::Trigger& trigger)
{
    *at++ = uint8_t(trigger.edge);
    write16(at, trigger.low);
    write16(at, trigger.high);
}

uint32_t checksum(const uint8_t* bytes, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; i++) {
        crc ^= i >= 16 && i < 20 ? 0 : bytes[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320U : 0);
        }
    }
    return ~crc;
}

size_t encode(const Sequencer::Document& document, uint8_t* bytes)
{
    uint16_t count = 0;
    for (const auto& block : document.blocks) {
        count += block.lane != 0;
    }
    const auto length = header_size + lane_size * Sequencer::lane_count + block_size * count;
    memcpy(bytes, "VSEQ", 4);
    uint8_t* at = bytes + 4;
    *at++ = schema_version;
    *at++ = Sequencer::lane_count;
    write16(at, count);
    write32(at, length);
    write32(at, document.revision);
    write32(at, 0);
    *at++ = document.rate;
    write_trigger(at, document.shared);

    for (uint8_t n = 0; n < Sequencer::lane_count; n++) {
        const auto& lane = document.lanes[n];
        *at++ = n + 1;
        *at++ = lane.enabled;
        *at++ = uint8_t(lane.output);
        *at++ = uint8_t(lane.clock);
        write16(at, lane.idle);
        write_trigger(at, lane.trigger);
        memcpy(at, lane.name, sizeof(lane.name));
        at += sizeof(lane.name);
        *at++ = lane.autostart;
        *at++ = lane.repeat;
    }
    for (uint16_t n = 0; n < Sequencer::block_count; n++) {
        const auto& block = document.blocks[n];
        if (block.lane == 0) {
            continue;
        }
        write16(at, n + 1);
        *at++ = block.lane;
        *at++ = block.position;
        *at++ = uint8_t(block.type);
        write16(at, block.duration);
        write16(at, block.from);
        write16(at, block.to);
    }
    at = bytes + 16;
    write32(at, checksum(bytes, length));
    return length;
}

Decode decode(const uint8_t* bytes, size_t length, Sequencer::Document& document)
{
    if (length < header_size || length > max_bytes || memcmp(bytes, "VSEQ", 4) != 0) {
        return Decode::Invalid;
    }
    const uint8_t* at = bytes + 4;
    const uint8_t version = *at++;
    if (version < 1 || version > schema_version) {
        return Decode::Unsupported;
    }
    const size_t file_lane_size = legacy_lane_size + (version - 1);
    const uint8_t lanes = *at++;
    const uint16_t blocks = read16(at);
    const uint32_t exact = read32(at);
    const uint32_t revision = read32(at);
    const uint32_t crc = read32(at);
    if (lanes > Sequencer::lane_count || blocks > Sequencer::block_count) {
        return Decode::Unsupported;
    }
    if (exact != length || length != header_size + file_lane_size * lanes + block_size * blocks ||
        crc != checksum(bytes, length)) {
        return Decode::Invalid;
    }

    // Reset in place to avoid a document-sized temporary on the IO stack.
    for (auto& lane : document.lanes) {
        lane = {};
    }
    for (auto& block : document.blocks) {
        block = {};
    }
    document.revision = revision;
    document.rate = *at++;
    document.shared = read_trigger(at);

    uint16_t seen = 0;
    for (uint8_t n = 0; n < lanes; n++) {
        const uint8_t slot = *at++;
        if (slot == 0 || slot > Sequencer::lane_count || (seen & (1U << (slot - 1)))) {
            return Decode::Invalid;
        }
        seen |= 1U << (slot - 1);
        auto& lane = document.lanes[slot - 1];
        lane.enabled = *at++;
        lane.output = Sequencer::Output(*at++);
        lane.clock = Sequencer::Clock(*at++);
        lane.idle = read16(at);
        lane.trigger = read_trigger(at);
        memcpy(lane.name, at, sizeof(lane.name));
        at += sizeof(lane.name);
        if (version >= 2) {
            lane.autostart = *at++;
        }
        if (version >= 3) {
            lane.repeat = *at++;
        }
    }
    for (uint16_t n = 0; n < blocks; n++) {
        const uint16_t slot = read16(at);
        if (slot == 0 || slot > Sequencer::block_count || document.blocks[slot - 1].lane != 0) {
            return Decode::Invalid;
        }
        auto& block = document.blocks[slot - 1];
        block.lane = *at++;
        if (block.lane == 0 || block.lane > Sequencer::lane_count ||
            !(seen & (1U << (block.lane - 1)))) {
            return Decode::Invalid;
        }
        block.position = *at++;
        block.type = Sequencer::Type(*at++);
        block.duration = read16(at);
        block.from = read16(at);
        block.to = read16(at);
    }
    return document.valid() ? Decode::Valid : Decode::Invalid;
}

} // namespace SequencerDocument
} // namespace Vektor2
