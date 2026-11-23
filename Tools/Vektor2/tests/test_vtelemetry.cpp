#include "VTelemetryCore.h"

#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <iostream>

using namespace Vektor2::VTelemetryCore;

struct Inputs {
    bool gps_valid = true;
    float temperature = 23.5f;
};

static bool read(uint16_t id, Sample& s, void* p)
{
    auto& inputs = *static_cast<Inputs*>(p);
    if (id == 100) { s.number = inputs.temperature; return true; } // Voltage source, can be mapped to HoTT temperature
    if (id == 90) { s.number = 1500; return true; }
    if (id == 12) { s.number = -25; return true; }
    if (id == 6) {
        if (!inputs.gps_valid) { return false; }
        s.coordinate_e7 = 505500000; return true;
    }
    if (id == 7) { s.coordinate_e7 = -1231250000; return true; }
    if (id == 2) { s.number = 18.75f; return true; }
    if (id == 242) { s.coordinate_e7 = 505500001; return true; }
    if (id == 257) { s.number = -2.1f; return true; }
    if (id == 258) { s.number = .375f; return true; }
    if (id == 259) { s.number = -.4f; return true; }
    return false;
}

static uint16_t le16(const uint8_t* p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
static void ibus_query(uint8_t cmd, uint8_t addr, uint8_t q[4])
{
    q[0] = 4; q[1] = cmd | addr;
    const uint16_t crc = 0xFFFF - q[0] - q[1];
    q[2] = uint8_t(crc); q[3] = uint8_t(crc >> 8);
}
static bool ibus_crc_valid(const uint8_t* p, uint8_t n)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < n-2; i++) { crc -= p[i]; }
    return crc == le16(p+n-2);
}

int main()
{
    assert(source_count() > 50);
    assert(find_source(70) == nullptr && find_source(74) == nullptr);
    assert(find_source(140) != nullptr && find_source(141) != nullptr);
    for (uint16_t id = 240; id <= 261; id++) {
        assert(find_source(id) != nullptr);
    }
    assert(find_source(240)->type == Type::CoordinateE7);
    assert(find_source(243)->type == Type::CoordinateE7);
    assert(find_source(244)->type == Type::Number);
    assert(find_source(257)->type == Type::Number);
    assert(find_source(258)->ibus_scale == 1000);
    assert(find_source(259)->ibus_scale == 1000);
    assert(find_source(260)->type == Type::Number);
    assert(find_source(261)->type == Type::Number);
    for (uint8_t i = 0; i < source_count(); i++) {
        const Source* src = source_at(i);
        assert(src != nullptr);
        for (uint8_t j = 0; j < i; j++) { assert(source_at(j)->id != src->id); }
    }

    assert(find_source(0) == nullptr && find_source(999) == nullptr);
    assert(find_source(100)->ibus_sensor_type == 0x03);
    assert(slot_definition(0) == nullptr && slot_definition(66) == nullptr);
    assert(slot_definition(16)->protocol == Protocol::HottEAM);
    assert(slot_definition(46)->type == Type::CoordinateE7);
    assert(slot_definition(59)->protocol == Protocol::HottVario);

    int16_t cfg[slot_count]{};
    cfg[0] = 100; // voltage
    cfg[1] = 90;  // RPM
    cfg[2] = 6;   // latitude: 4-byte
    cfg[3] = 999; // unknown ID -> ignored
    cfg[4] = -1;  // negative -> ignored
    cfg[15] = 100; // HoTT EAM cell1: physical mismatch permitted
    cfg[31] = 100; // HoTT EAM temp1: voltage into temperature is permitted
    cfg[45] = 6; // HoTT GPS latitude
    cfg[46] = 7; // HoTT GPS longitude
    cfg[43] = 2; // HoTT GPS direction from unrelated speed: permitted
    cfg[58] = 90; // HoTT VARIO altitude from RPM: permitted
    cfg[59] = 6;  // HoTT VARIO numeric field from coordinate: incompatible
    cfg[62] = 12; // HoTT VARIO climb3 from negative heading, still numeric
    Router r;
    assert(r.configure(cfg) == 3);
    assert(!r.assigned(4) && !r.assigned(5) && !r.assigned(60));
    assert(r.assigned(32)->id == 100);
    Inputs inputs;
    r.sample(read, &inputs);

    uint8_t q[4], resp[8]{};
    ibus_query(0x80, 1, q);
    assert(r.ibus_reply(q, resp) == 4);
    assert(ibus_crc_valid(resp, 4) && resp[1] == 0x81);
    ibus_query(0x90, 1, q);
    assert(r.ibus_reply(q, resp) == 6);
    assert(resp[2] == 0x03 && resp[3] == 2 && ibus_crc_valid(resp, 6));
    ibus_query(0xA0, 1, q);
    assert(r.ibus_reply(q, resp) == 6);
    assert(le16(resp+2) == 2350 && ibus_crc_valid(resp, 6));
    ibus_query(0xA0, 2, q);
    assert(r.ibus_reply(q, resp) == 6 && le16(resp+2) == 1500);
    ibus_query(0x90, 3, q);
    assert(r.ibus_reply(q, resp) == 6 && resp[2] == 0x80 && resp[3] == 4);
    ibus_query(0xA0, 3, q);
    assert(r.ibus_reply(q, resp) == 8);
    assert((uint32_t(resp[2]) | (uint32_t(resp[3]) << 8) |
            (uint32_t(resp[4]) << 16) | (uint32_t(resp[5]) << 24)) == 505500000U);
    assert(ibus_crc_valid(resp, 8));
    q[3] ^= 1; // bad checksum
    assert(r.ibus_reply(q, resp) == 0);
    ibus_query(0xA0, 4, q);
    assert(r.ibus_reply(q, resp) == 0); // invalid assignment silent
    ibus_query(0xB0, 1, q);
    assert(r.ibus_reply(q, resp) == 0);
    ibus_query(0xA0, 0, q);
    assert(r.ibus_reply(q, resp) == 0);

    uint8_t frame[44]{};
    assert(r.hott_frame(0x8E, frame));
    assert(frame[0] == 0x7C && frame[1] == 0x8E && frame[3] == 0xE0 && frame[43] == 0x7D);
    assert(frame[6] == 255); // 23.5V cannot fit 8-bit cell format, clamps
    assert(frame[24] == 44); // 23.5 number -> 44 raw (0deg offset 20)
    assert(le16(frame+26) == 500); // unassigned altitude is zero with HoTT bias
    assert(le16(frame+34) == 30000); // unassigned climb zero
    assert(!r.hott_frame(0x8D, frame)); // GAM not supported/advertised

    assert(r.hott_frame(0x8A, frame));
    assert(frame[3] == 0xA0 && frame[27] == '-');
    assert(frame[9] == 0 && le16(frame+10) == 5033 && le16(frame+12) == 0);
    assert(frame[14] == 1 && le16(frame+15) == 12307 && le16(frame+17) == 5000);
    assert(le16(frame+21) == 500);
    assert(frame[6] == 9); // speed source 18.75, encoded at 0.5 step
    assert(r.hott_frame(0x89, frame));
    assert(le16(frame+5) == 2000); // RPM 1500 + 500 altitude bias
    assert(le16(frame+13) == 22500); // arbitrary negative heading encoded as climb
    assert(le16(frame+15) == 30000); // unconfigured remains neutral

    inputs.gps_valid = false;
    inputs.temperature = NAN;
    r.sample(read, &inputs);
    ibus_query(0xA0, 1, q);
    assert(r.ibus_reply(q, resp) == 0); // NaN suppressed, never shown as zero
    ibus_query(0x80, 1, q);
    assert(r.ibus_reply(q, resp) == 4); // discovery still works for temporarily invalid sample
    ibus_query(0xA0, 3, q);
    assert(r.ibus_reply(q, resp) == 0); // no GPS sample
    assert(r.hott_frame(0x8A, frame));
    assert(frame[9] == 0 && frame[10] == 0); // blank coordinate for invalid GPS
    assert(r.hott_frame(0x8E, frame));
    assert(frame[24] == 20); // neutral temp offset when no measurement

    // DP telemetry catalog: E7 actual point, signed heading error, and
    // normalized earth-frame correction vectors use the existing wire codec.
    std::memset(cfg, 0, sizeof(cfg));
    cfg[0] = 242;
    cfg[1] = 257;
    cfg[2] = 258;
    cfg[3] = 259;
    assert(r.configure(cfg) == 0);
    r.sample(read, &inputs);
    ibus_query(0xA0, 1, q);
    assert(r.ibus_reply(q, resp) == 8);
    assert((uint32_t(resp[2]) | (uint32_t(resp[3]) << 8) |
           (uint32_t(resp[4]) << 16) | (uint32_t(resp[5]) << 24)) == 505500001U);
    ibus_query(0xA0, 2, q);
    assert(r.ibus_reply(q, resp) == 6);
    assert(int16_t(le16(resp+2)) == -210);
    ibus_query(0xA0, 3, q);
    assert(r.ibus_reply(q, resp) == 6 && le16(resp+2) == 375);
    ibus_query(0xA0, 4, q);
    assert(r.ibus_reply(q, resp) == 6 && int16_t(le16(resp+2)) == -400);

    // All slots disabled -> no advertised HoTT modules or i-BUS sensors.
    std::memset(cfg, 0, sizeof(cfg));
    assert(r.configure(cfg) == 0);
    assert(!r.module_enabled(Protocol::HottEAM));
    assert(!r.hott_frame(0x8E, frame));
    ibus_query(0x80, 1, q);
    assert(r.ibus_reply(q, resp) == 0);

    std::cout << "VTEL codec and router tests passed\n";
}
