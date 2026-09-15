#pragma once

#include "Vektor_Schema.h"

#include <AP_Param/AP_Param.h>

#include <stdint.h>

namespace Vektor {

class AssignmentMatrix {
public:
    static constexpr uint8_t max_routes = 16;

    struct Entry {
        uint32_t route_id = 0;
        uint32_t source_output_id = 0;
        uint32_t destination_input_id = 0;
        uint16_t flags = 0;
        bool active = false;
    };

    enum class SetResult : uint8_t {
        OK,
        BAD_SOURCE,
        BAD_DESTINATION,
        SOURCE_NOT_ROUTABLE,
        DESTINATION_NOT_ROUTABLE,
        TYPE_MISMATCH,
        UNSUPPORTED_FLAGS,
        CYCLE,
        FULL,
        INVALID_ROUTE_ID,
    };

    AssignmentMatrix();

    void reset();
    void configure_channel_limits(uint8_t pwm_input_count,
                                  uint8_t pwm_output_count,
                                  uint32_t unavailable_pwm_output_mask = 0);
    void load_persistent();
    SetResult set(uint32_t source_output_id,
                  uint32_t destination_input_id,
                  uint16_t flags,
                  const Entry *&accepted);
    bool remove(uint32_t route_id);

    uint8_t count() const { return _count; }
    uint32_t revision() const { return _revision; }
    const Entry *by_index(uint8_t index) const;
    const Entry *by_id(uint32_t route_id) const;
    const Entry *to_destination(uint32_t destination_input_id) const;

    static uint32_t make_route_id(uint32_t source_output_id,
                                  uint32_t destination_input_id);
    static const AP_Param::GroupInfo var_info[];

private:
    SetResult validate(uint32_t source_output_id,
                       uint32_t destination_input_id,
                       uint16_t flags,
                       int8_t replaced_slot) const;
    bool creates_cycle(uint32_t source_output_id,
                       uint32_t destination_input_id,
                       int8_t replaced_slot) const;
    int8_t slot_for_destination(uint32_t destination_input_id) const;
    int8_t first_free_slot() const;
    void store_slot(uint8_t slot);
    void clear_stored_slot(uint8_t slot);

    Entry _entries[max_routes] {};
    AP_Int32 _stored_source[max_routes];
    AP_Int32 _stored_destination[max_routes];
    AP_Int16 _stored_flags[max_routes];
    uint8_t _count = 0;
    uint8_t _pwm_input_count = 6;
    uint8_t _pwm_output_count = 12;
    uint32_t _unavailable_pwm_output_mask = 0;
    bool _persistence_enabled = false;
    uint32_t _revision = 0;
};

} // namespace Vektor
