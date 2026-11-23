#include "SequencerParameters.h"
#include "SequencerDocument.h"

#include <cmath>
#include <cstring>

namespace Vektor2 {

#define CAPABILITY(name, n, member, value) \
    AP_GROUPINFO_FLAGS(name, n, SequencerParameters, member, value, AP_PARAM_FLAG_INTERNAL_USE_ONLY)
#define LANE(n) AP_SUBGROUPINFO(lane[(n) - 1], "SEQ" #n "_", 15 + n, SequencerParameters, Lane)
#define BANK(n, table) \
    { "SEQB_", AP_VAROFFSET(SequencerParameters, bank), {group_info : BlockBanks::table}, \
      AP_PARAM_FLAG_NESTED_OFFSET, n, AP_PARAM_GROUP }

const AP_Param::GroupInfo SequencerParameters::var_info[] = {
    CAPABILITY("SEQ_VER", 1, version, 2),
    CAPABILITY("SEQ_SCHEMA", 2, schema, SequencerDocument::schema_version),
    CAPABILITY("SEQ_LANES", 3, lanes, Sequencer::lane_count),
    CAPABILITY("SEQ_PER_LANE", 4, lane_limit, Sequencer::per_lane),
    CAPABILITY("SEQ_BLOCKS", 5, blocks, Sequencer::block_count),
    CAPABILITY("SEQ_OUTPUTS", 6, outputs, 3),
    CAPABILITY("SEQ_TYPES", 7, types, 7),
    CAPABILITY("SEQ_EDGES", 8, edges, 7),
    CAPABILITY("SEQ_STORE", 14, store, 1),
    CAPABILITY("SEQ_MS_MIN", 37, min_duration, Sequencer::min_duration_ms),
    CAPABILITY("SEQ_MS_MAX", 38, max_duration, Sequencer::max_duration_ms),
    CAPABILITY("SEQ_TEL_MAX", 39, rate_limit, Sequencer::max_rate_hz),
    CAPABILITY("SEQ_EDGE_LAT", 40, trigger_latency, 1),
    AP_GROUPINFO_FLAGS("SEQ_ENABLE", 9, SequencerParameters, enable, 0, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("SEQ_HZ_TEL", 10, SequencerParameters, rate, Sequencer::default_rate_hz),
    AP_GROUPINFO("SEQ_EDGE", 11, SequencerParameters, edge, 0),
    AP_GROUPINFO("SEQ_LOW", 12, SequencerParameters, low, 1500),
    AP_GROUPINFO("SEQ_HIGH", 13, SequencerParameters, high, 2000),
    LANE(1), LANE(2), LANE(3), LANE(4),
#if VEKTOR2_SEQ_LARGE
    LANE(5), LANE(6), LANE(7), LANE(8), LANE(9), LANE(10),
    LANE(11), LANE(12), LANE(13), LANE(14), LANE(15), LANE(16),
#endif
    BANK(32, first),
#if VEKTOR2_SEQ_LARGE
    BANK(33, second), BANK(34, third), BANK(35, fourth), BANK(36, fifth),
#endif
    AP_GROUPEND
};

#undef CAPABILITY
#undef LANE
#undef BANK

const AP_Param::GroupInfo SequencerParameters::Lane::var_info[] = {
    AP_GROUPINFO("ENABLE", 1, Lane, enable, 0),
    AP_GROUPINFO("TYPE", 2, Lane, type, 0),
    AP_GROUPINFO("IDLE", 3, Lane, idle, 1500),
    AP_GROUPINFO("CLOCK", 4, Lane, clock, 0),
    AP_GROUPINFO("EDGE", 5, Lane, edge, 0),
    AP_GROUPINFO("LOW", 6, Lane, low, 1500),
    AP_GROUPINFO("HIGH", 7, Lane, high, 2000),
    AP_GROUPINFO("NAME1", 8, Lane, name[0], 0),
    AP_GROUPINFO("NAME2", 9, Lane, name[1], 0),
    AP_GROUPINFO("NAME3", 10, Lane, name[2], 0),
    AP_GROUPINFO("NAME4", 11, Lane, name[3], 0),
    AP_GROUPINFO("AUTOSTART", 12, Lane, autostart, 0),
    AP_GROUPINFO("REPEAT", 13, Lane, repeat, 0),
    AP_GROUPEND
};

const AP_Param::GroupInfo SequencerParameters::Block::var_info[] = {
    AP_GROUPINFO("LANE", 1, Block, lane, 0),
    AP_GROUPINFO("POS", 2, Block, position, 0),
    AP_GROUPINFO("TYPE", 3, Block, type, 0),
    AP_GROUPINFO("DUR", 4, Block, duration_ms, 0),
    AP_GROUPINFO("FROM", 5, Block, from, 0),
    AP_GROUPINFO("TO", 6, Block, to, 0),
    AP_GROUPEND
};

#define SLOT(bank, n) AP_SUBGROUPINFO(blocks[(n) - 1], #n "_", (n) - 63 * (bank), BlockBanks, Block)
const AP_Param::GroupInfo SequencerParameters::BlockBanks::first[] = {
    SLOT(0, 1), SLOT(0, 2), SLOT(0, 3), SLOT(0, 4), SLOT(0, 5), SLOT(0, 6),
    SLOT(0, 7), SLOT(0, 8), SLOT(0, 9), SLOT(0, 10), SLOT(0, 11), SLOT(0, 12),
    SLOT(0, 13), SLOT(0, 14), SLOT(0, 15), SLOT(0, 16), SLOT(0, 17), SLOT(0, 18),
    SLOT(0, 19), SLOT(0, 20), SLOT(0, 21), SLOT(0, 22), SLOT(0, 23), SLOT(0, 24),
    SLOT(0, 25), SLOT(0, 26), SLOT(0, 27), SLOT(0, 28), SLOT(0, 29), SLOT(0, 30),
    SLOT(0, 31), SLOT(0, 32), SLOT(0, 33), SLOT(0, 34), SLOT(0, 35), SLOT(0, 36),
    SLOT(0, 37), SLOT(0, 38), SLOT(0, 39), SLOT(0, 40), SLOT(0, 41), SLOT(0, 42),
    SLOT(0, 43), SLOT(0, 44), SLOT(0, 45), SLOT(0, 46), SLOT(0, 47), SLOT(0, 48),
#if VEKTOR2_SEQ_LARGE
    SLOT(0, 49), SLOT(0, 50), SLOT(0, 51), SLOT(0, 52), SLOT(0, 53), SLOT(0, 54),
    SLOT(0, 55), SLOT(0, 56), SLOT(0, 57), SLOT(0, 58), SLOT(0, 59), SLOT(0, 60),
    SLOT(0, 61), SLOT(0, 62), SLOT(0, 63),
#endif
    AP_GROUPEND
};
#if VEKTOR2_SEQ_LARGE
const AP_Param::GroupInfo SequencerParameters::BlockBanks::second[] = {
    SLOT(1, 64), SLOT(1, 65), SLOT(1, 66), SLOT(1, 67), SLOT(1, 68), SLOT(1, 69),
    SLOT(1, 70), SLOT(1, 71), SLOT(1, 72), SLOT(1, 73), SLOT(1, 74), SLOT(1, 75),
    SLOT(1, 76), SLOT(1, 77), SLOT(1, 78), SLOT(1, 79), SLOT(1, 80), SLOT(1, 81),
    SLOT(1, 82), SLOT(1, 83), SLOT(1, 84), SLOT(1, 85), SLOT(1, 86), SLOT(1, 87),
    SLOT(1, 88), SLOT(1, 89), SLOT(1, 90), SLOT(1, 91), SLOT(1, 92), SLOT(1, 93),
    SLOT(1, 94), SLOT(1, 95), SLOT(1, 96), SLOT(1, 97), SLOT(1, 98), SLOT(1, 99),
    SLOT(1, 100), SLOT(1, 101), SLOT(1, 102), SLOT(1, 103), SLOT(1, 104), SLOT(1, 105),
    SLOT(1, 106), SLOT(1, 107), SLOT(1, 108), SLOT(1, 109), SLOT(1, 110), SLOT(1, 111),
    SLOT(1, 112), SLOT(1, 113), SLOT(1, 114), SLOT(1, 115), SLOT(1, 116), SLOT(1, 117),
    SLOT(1, 118), SLOT(1, 119), SLOT(1, 120), SLOT(1, 121), SLOT(1, 122), SLOT(1, 123),
    SLOT(1, 124), SLOT(1, 125), SLOT(1, 126),
    AP_GROUPEND
};
const AP_Param::GroupInfo SequencerParameters::BlockBanks::third[] = {
    SLOT(2, 127), SLOT(2, 128), SLOT(2, 129), SLOT(2, 130), SLOT(2, 131), SLOT(2, 132),
    SLOT(2, 133), SLOT(2, 134), SLOT(2, 135), SLOT(2, 136), SLOT(2, 137), SLOT(2, 138),
    SLOT(2, 139), SLOT(2, 140), SLOT(2, 141), SLOT(2, 142), SLOT(2, 143), SLOT(2, 144),
    SLOT(2, 145), SLOT(2, 146), SLOT(2, 147), SLOT(2, 148), SLOT(2, 149), SLOT(2, 150),
    SLOT(2, 151), SLOT(2, 152), SLOT(2, 153), SLOT(2, 154), SLOT(2, 155), SLOT(2, 156),
    SLOT(2, 157), SLOT(2, 158), SLOT(2, 159), SLOT(2, 160), SLOT(2, 161), SLOT(2, 162),
    SLOT(2, 163), SLOT(2, 164), SLOT(2, 165), SLOT(2, 166), SLOT(2, 167), SLOT(2, 168),
    SLOT(2, 169), SLOT(2, 170), SLOT(2, 171), SLOT(2, 172), SLOT(2, 173), SLOT(2, 174),
    SLOT(2, 175), SLOT(2, 176), SLOT(2, 177), SLOT(2, 178), SLOT(2, 179), SLOT(2, 180),
    SLOT(2, 181), SLOT(2, 182), SLOT(2, 183), SLOT(2, 184), SLOT(2, 185), SLOT(2, 186),
    SLOT(2, 187), SLOT(2, 188), SLOT(2, 189),
    AP_GROUPEND
};
const AP_Param::GroupInfo SequencerParameters::BlockBanks::fourth[] = {
    SLOT(3, 190), SLOT(3, 191), SLOT(3, 192), SLOT(3, 193), SLOT(3, 194), SLOT(3, 195),
    SLOT(3, 196), SLOT(3, 197), SLOT(3, 198), SLOT(3, 199), SLOT(3, 200), SLOT(3, 201),
    SLOT(3, 202), SLOT(3, 203), SLOT(3, 204), SLOT(3, 205), SLOT(3, 206), SLOT(3, 207),
    SLOT(3, 208), SLOT(3, 209), SLOT(3, 210), SLOT(3, 211), SLOT(3, 212), SLOT(3, 213),
    SLOT(3, 214), SLOT(3, 215), SLOT(3, 216), SLOT(3, 217), SLOT(3, 218), SLOT(3, 219),
    SLOT(3, 220), SLOT(3, 221), SLOT(3, 222), SLOT(3, 223), SLOT(3, 224), SLOT(3, 225),
    SLOT(3, 226), SLOT(3, 227), SLOT(3, 228), SLOT(3, 229), SLOT(3, 230), SLOT(3, 231),
    SLOT(3, 232), SLOT(3, 233), SLOT(3, 234), SLOT(3, 235), SLOT(3, 236), SLOT(3, 237),
    SLOT(3, 238), SLOT(3, 239), SLOT(3, 240), SLOT(3, 241), SLOT(3, 242), SLOT(3, 243),
    SLOT(3, 244), SLOT(3, 245), SLOT(3, 246), SLOT(3, 247), SLOT(3, 248), SLOT(3, 249),
    SLOT(3, 250), SLOT(3, 251), SLOT(3, 252),
    AP_GROUPEND
};
const AP_Param::GroupInfo SequencerParameters::BlockBanks::fifth[] = {
    SLOT(4, 253), SLOT(4, 254), SLOT(4, 255), SLOT(4, 256),
    AP_GROUPEND
};
#endif
#undef SLOT

bool SequencerParameters::owns(const char* name)
{
    return strncasecmp(name, "SEQ_", 4) == 0 || strncasecmp(name, "SEQB_", 5) == 0 ||
        (strncasecmp(name, "SEQ", 3) == 0 && name[3] >= '1' && name[3] <= '9');
}

bool SequencerParameters::valid_write(const char* name, float value)
{
    if (!std::isfinite(value) || value < 0 || value > 16777215 || value > uint32_t(value)) {
        return false;
    }
    const char* field = strrchr(name, '_');
    if (field == nullptr) {
        return false;
    }
    ++field;
    if (strcasecmp(name, "SEQ_HZ_TEL") == 0) {
        return value <= Sequencer::max_rate_hz;
    }
    const bool block = strncasecmp(name, "SEQB_", 5) == 0;
    if (strcasecmp(field, "ENABLE") == 0 || strcasecmp(field, "CLOCK") == 0 ||
        strcasecmp(field, "AUTOSTART") == 0 || strcasecmp(field, "REPEAT") == 0) {
        return value <= 1;
    }
    if (strcasecmp(field, "EDGE") == 0) {
        return value <= 2;
    }
    if (strcasecmp(field, "LOW") == 0 || strcasecmp(field, "HIGH") == 0) {
        return value >= 800 && value <= 2200;
    }
    if (strcasecmp(field, "TYPE") == 0) {
        return value <= (block ? 2 : 1);
    }
    if (strcasecmp(field, "IDLE") == 0 || (block &&
        (strcasecmp(field, "FROM") == 0 || strcasecmp(field, "TO") == 0))) {
        return value <= 1 || (value >= 1000 && value <= 2000);
    }
    if (block && strcasecmp(field, "DUR") == 0) {
        return value < 1 || (value >= Sequencer::min_duration_ms && value <= Sequencer::max_duration_ms);
    }
    if (block && strcasecmp(field, "LANE") == 0) {
        return value <= Sequencer::lane_count;
    }
    if (block && strcasecmp(field, "POS") == 0) {
        return value <= Sequencer::per_lane;
    }
    return strncasecmp(field, "NAME", 4) == 0;
}

void SequencerParameters::read(Sequencer::Document& document) const
{
    document.rate = rate.get();
    document.shared = {uint16_t(low.get()), uint16_t(high.get()), Sequencer::Edge(edge.get())};
    for (uint8_t n = 0; n < Sequencer::lane_count; n++) {
        const auto& from = lane[n];
        auto& to = document.lanes[n];
        to.enabled = from.enable.get();
        to.autostart = from.autostart.get();
        to.repeat = from.repeat.get();
        to.output = Sequencer::Output(from.type.get());
        to.clock = Sequencer::Clock(from.clock.get());
        to.idle = from.idle.get();
        to.trigger = {uint16_t(from.low.get()), uint16_t(from.high.get()), Sequencer::Edge(from.edge.get())};
        for (uint8_t i = 0; i < sizeof(to.name); i++) {
            to.name[i] = uint8_t(uint32_t(from.name[i / 3].get()) >> (8 * (i % 3)));
        }
    }
    for (uint16_t n = 0; n < Sequencer::block_count; n++) {
        const auto& from = bank.blocks[n];
        auto& to = document.blocks[n];
        to = {};
        if (from.lane.get() != 0) {
            to.lane = from.lane.get();
            to.position = from.position.get();
            to.type = Sequencer::Type(from.type.get());
            const int32_t duration = from.duration_ms.get();
            to.duration = duration >= 0 && duration <= Sequencer::max_duration_ms ? duration : 0;
            to.from = from.from.get();
            to.to = from.to.get();
        }
    }
}

// Legacy FTP imports already run on the HAL IO thread. Save only fields
// that changed and avoid filling the normal asynchronous save queue.
template <typename Param, typename Value>
static void import_value(Param& param, Value value)
{
    if (param.get() != value) {
        param.set(value);
        param.save_sync(true, false);
    }
}

void SequencerParameters::import(const Sequencer::Document& document)
{
    import_value(rate, document.rate);
    import_value(edge, uint8_t(document.shared.edge));
    import_value(low, document.shared.low);
    import_value(high, document.shared.high);
    for (uint8_t n = 0; n < Sequencer::lane_count; n++) {
        const auto& src = document.lanes[n];
        auto& dst = lane[n];
        import_value(dst.enable, src.enabled);
        import_value(dst.autostart, src.autostart);
        import_value(dst.repeat, src.repeat);
        import_value(dst.type, uint8_t(src.output));
        import_value(dst.clock, uint8_t(src.clock));
        import_value(dst.idle, src.idle);
        import_value(dst.edge, uint8_t(src.trigger.edge));
        import_value(dst.low, src.trigger.low);
        import_value(dst.high, src.trigger.high);
        for (uint8_t i = 0; i < 4; i++) {
            int32_t value = 0;
            for (uint8_t j = 0; j < 3; j++) {
                value |= int32_t(uint8_t(src.name[3 * i + j])) << (8 * j);
            }
            import_value(dst.name[i], value);
        }
    }
    for (uint16_t n = 0; n < Sequencer::block_count; n++) {
        const auto& src = document.blocks[n];
        auto& dst = bank.blocks[n];
        import_value(dst.lane, src.lane);
        import_value(dst.position, src.position);
        import_value(dst.type, uint8_t(src.type));
        import_value(dst.duration_ms, src.duration);
        import_value(dst.from, src.from);
        import_value(dst.to, src.to);
    }
}

} // namespace Vektor2
