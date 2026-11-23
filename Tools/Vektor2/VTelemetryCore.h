#pragma once

// VTEL's protocol-independent, allocation-free mapping and wire codecs.
// This header deliberately has no ArduPilot dependencies so it can be
// exercised on a host and on small flight controllers alike.

#include <stdint.h>

namespace Vektor2 {
namespace VTelemetryCore {

enum class Type : uint8_t { Number, CoordinateE7 };
enum class Protocol : uint8_t { IBus, HottEAM, HottGPS, HottVario };

struct Source {
    uint16_t id;
    const char* name;
    Type type;
    const char* unit;
    uint8_t ibus_sensor_type;
    uint8_t ibus_width;
    float ibus_scale;
    float ibus_bias;
};

struct Sample {
    float number = 0;
    int32_t coordinate_e7 = 0;
    bool valid = false;
};

// Slot numbering is a persistent parameter interface. Never reorder entries.
// 1..15 i-BUS sensor addresses, 16..43 HoTT EAM, 44..58 HoTT GPS,
// 59..65 HoTT VARIO. All indexes are 1-based.
static constexpr uint8_t ibus_count = 15;
static constexpr uint8_t eam_first = 16;
static constexpr uint8_t gps_first = 44;
static constexpr uint8_t vario_first = 59;
static constexpr uint8_t slot_count = 65;

struct Slot {
    Protocol protocol;
    const char* name;
    Type type;
    uint8_t offset;  // index within the 44-byte HoTT packet, unused by i-BUS
    uint8_t width;   // HoTT payload width, 1/2; 0 for coordinate pair
    float scale;     // HoTT display quantity to wire units; no source-unit coercion
    float bias;      // raw additive offset (temperature, altitude, climb)
    bool signed_value;
};

const Source* find_source(uint16_t id);
const Slot* slot_definition(uint8_t slot_number);
uint8_t source_count();
const Source* source_at(uint8_t index);

using Reader = bool (*)(uint16_t source_id, Sample& value, void* context);

class Router {
public:
    // An invalid ID or incompatible structure disables ONLY that slot.
    // Physical quantity / unit is intentionally not used for compatibility.
    // Returns number of invalid nonzero assignments.
    uint8_t configure(const int16_t (&source_ids)[slot_count]);
    void sample(Reader reader, void* context);
    const Source* assigned(uint8_t slot_number) const;
    const Sample& value(uint8_t slot_number) const;
    bool module_enabled(Protocol protocol) const;

    // i-BUS uses receiver polling. Returns zero if a request should be ignored.
    // Output capacity must be 8 bytes. Includes subtractive 16-bit checksum.
    uint8_t ibus_reply(const uint8_t request[4], uint8_t output[8]) const;

    // HoTT's binary reply is 44 bytes + separate additive CRC. The wire
    // thread paces individual bytes. Returns false for unused modules.
    bool hott_frame(uint8_t sensor_id, uint8_t output[44]) const;

private:
    const Source* _assigned[slot_count] {};
    Sample _samples[slot_count] {};
};

} // namespace VTelemetryCore
} // namespace Vektor2
