#pragma once

#include "Sequencer.h"
#include <AP_Param/AP_Param.h>

namespace Vektor2 {

class SequencerParameters {
public:
    static const AP_Param::GroupInfo var_info[];

    SequencerParameters() { AP_Param::setup_object_defaults(this, var_info); }

    static bool owns(const char* name);
    static bool valid_write(const char* name, float value);
    void read(Sequencer::Document& document) const;
    void import(const Sequencer::Document& document);

    AP_Int8 enable;
    AP_Int8 rate;

private:
    struct Lane {
        static const AP_Param::GroupInfo var_info[];
        Lane() { AP_Param::setup_object_defaults(this, var_info); }
        AP_Int8 enable;
        AP_Int8 autostart;
        AP_Int8 repeat;
        AP_Int8 type;
        AP_Int16 idle;
        AP_Int8 clock;
        AP_Int8 edge;
        AP_Int16 low;
        AP_Int16 high;
        AP_Int32 name[4];
    };
    struct Block {
        static const AP_Param::GroupInfo var_info[];
        AP_Int16 from;
        AP_Int16 to;
        AP_Int8 lane;
        AP_Int8 position;
        AP_Int8 type;
        AP_Int32 duration_ms;
    };
    struct BlockBanks {
        static const AP_Param::GroupInfo first[], second[], third[], fourth[], fifth[];
        Block blocks[Sequencer::block_count] {};
    };

    AP_Int8 version, schema, lanes, lane_limit, outputs, types, edges, store;
    AP_Int16 blocks, min_duration;
    AP_Int32 max_duration;
    AP_Int8 rate_limit, trigger_latency;
    AP_Int8 edge;
    AP_Int16 low, high;
    Lane lane[Sequencer::lane_count];
    BlockBanks bank;
};

} // namespace Vektor2
