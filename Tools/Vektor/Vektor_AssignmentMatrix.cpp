#include "Vektor_AssignmentMatrix.h"

#include "Vektor_Protocol.h"

#include <string.h>

namespace {

static constexpr uint16_t stored_route_flags =
    AP_PARAM_FLAG_INTERNAL_USE_ONLY | AP_PARAM_FLAG_HIDDEN;

#define VEKTOR_ROUTE_SLOT(NUMBER, BASE)                                      \
    AP_GROUPINFO_FLAGS(#NUMBER "_SRC",                                      \
                       BASE,                                                 \
                       Vektor::AssignmentMatrix,                             \
                       _stored_source[NUMBER - 1],                           \
                       0,                                                    \
                       stored_route_flags),                                  \
    AP_GROUPINFO_FLAGS(#NUMBER "_DST",                                      \
                       BASE + 1,                                             \
                       Vektor::AssignmentMatrix,                             \
                       _stored_destination[NUMBER - 1],                      \
                       0,                                                    \
                       stored_route_flags),                                  \
    AP_GROUPINFO_FLAGS(#NUMBER "_FLG",                                      \
                       BASE + 2,                                             \
                       Vektor::AssignmentMatrix,                             \
                       _stored_flags[NUMBER - 1],                            \
                       0,                                                    \
                       stored_route_flags)

uint32_t owner_id(const Vektor::FieldDescriptor &field)
{
    return Vektor::Protocol::fnv1a32(field.owner_component_path);
}

char hex_digit(uint8_t value)
{
    return value < 10 ? char('0' + value) : char('a' + value - 10);
}

void append_hex32(char *destination, uint32_t value)
{
    for (uint8_t i = 0; i < 8; i++) {
        const uint8_t shift = uint8_t((7U - i) * 4U);
        destination[i] = hex_digit(uint8_t((value >> shift) & 0xFU));
    }
}

template <typename Parameter, typename Value>
void save_without_gcs(Parameter &parameter, Value value)
{
    if (parameter.get() == value) {
        return;
    }
    parameter.set(value);
    parameter.save_sync(true, false);
}

} // namespace

const AP_Param::GroupInfo Vektor::AssignmentMatrix::var_info[] = {
    VEKTOR_ROUTE_SLOT(1, 1),
    VEKTOR_ROUTE_SLOT(2, 4),
    VEKTOR_ROUTE_SLOT(3, 7),
    VEKTOR_ROUTE_SLOT(4, 10),
    VEKTOR_ROUTE_SLOT(5, 13),
    VEKTOR_ROUTE_SLOT(6, 16),
    VEKTOR_ROUTE_SLOT(7, 19),
    VEKTOR_ROUTE_SLOT(8, 22),
    VEKTOR_ROUTE_SLOT(9, 25),
    VEKTOR_ROUTE_SLOT(10, 28),
    VEKTOR_ROUTE_SLOT(11, 31),
    VEKTOR_ROUTE_SLOT(12, 34),
    VEKTOR_ROUTE_SLOT(13, 37),
    VEKTOR_ROUTE_SLOT(14, 40),
    VEKTOR_ROUTE_SLOT(15, 43),
    VEKTOR_ROUTE_SLOT(16, 46),
    AP_GROUPEND
};

#undef VEKTOR_ROUTE_SLOT

namespace Vektor {

AssignmentMatrix::AssignmentMatrix()
{
    AP_Param::setup_object_defaults(this, var_info);
    reset();
}

void AssignmentMatrix::reset()
{
    for (uint8_t i = 0; i < max_routes; i++) {
        _entries[i] = {};
    }
    _count = 0;
    _persistence_enabled = false;
}

void AssignmentMatrix::load_persistent()
{
    reset();
    _persistence_enabled = false;
    for (uint8_t slot = 0; slot < max_routes; slot++) {
        const uint32_t source = uint32_t(_stored_source[slot].get());
        const uint32_t destination =
            uint32_t(_stored_destination[slot].get());
        const uint16_t flags = uint16_t(_stored_flags[slot].get());
        if (source == 0 || destination == 0) {
            continue;
        }

        const int8_t replaced_slot = slot_for_destination(destination);
        if (replaced_slot >= 0) {
            continue;
        }
        if (validate(source, destination, flags, replaced_slot) !=
            SetResult::OK) {
            continue;
        }
        const uint32_t route_id = make_route_id(source, destination);
        if (route_id == 0) {
            continue;
        }
        bool route_id_collision = false;
        for (uint8_t existing_slot = 0;
             existing_slot < max_routes;
             existing_slot++) {
            route_id_collision |= _entries[existing_slot].active &&
                                  _entries[existing_slot].route_id == route_id;
        }
        if (route_id_collision) {
            continue;
        }
        Entry &entry = _entries[slot];
        entry.route_id = route_id;
        entry.source_output_id = source;
        entry.destination_input_id = destination;
        entry.flags = flags;
        entry.active = true;
        _count++;
    }
    _persistence_enabled = true;
}

AssignmentMatrix::SetResult AssignmentMatrix::set(
    uint32_t source_output_id,
    uint32_t destination_input_id,
    uint16_t flags,
    const Entry *&accepted)
{
    accepted = nullptr;
    int8_t slot = slot_for_destination(destination_input_id);
    const SetResult result = validate(source_output_id,
                                      destination_input_id,
                                      flags,
                                      slot);
    if (result != SetResult::OK) {
        return result;
    }
    if (slot < 0) {
        slot = first_free_slot();
        if (slot < 0) {
            return SetResult::FULL;
        }
    }

    const uint32_t route_id = make_route_id(source_output_id,
                                            destination_input_id);
    if (route_id == 0) {
        return SetResult::INVALID_ROUTE_ID;
    }
    for (uint8_t existing_slot = 0;
         existing_slot < max_routes;
         existing_slot++) {
        if (_entries[existing_slot].active &&
            existing_slot != uint8_t(slot) &&
            _entries[existing_slot].route_id == route_id) {
            return SetResult::INVALID_ROUTE_ID;
        }
    }

    Entry &entry = _entries[uint8_t(slot)];
    if (!entry.active) {
        _count++;
    }
    entry.route_id = route_id;
    entry.source_output_id = source_output_id;
    entry.destination_input_id = destination_input_id;
    entry.flags = flags;
    entry.active = true;
    if (_persistence_enabled) {
        store_slot(uint8_t(slot));
    }
    accepted = &entry;
    return SetResult::OK;
}

bool AssignmentMatrix::remove(uint32_t route_id)
{
    for (uint8_t slot = 0; slot < max_routes; slot++) {
        Entry &entry = _entries[slot];
        if (!entry.active || entry.route_id != route_id) {
            continue;
        }
        entry = {};
        _count--;
        if (_persistence_enabled) {
            clear_stored_slot(slot);
        }
        return true;
    }
    return false;
}

const AssignmentMatrix::Entry *AssignmentMatrix::by_index(uint8_t index) const
{
    uint8_t active_index = 0;
    for (uint8_t slot = 0; slot < max_routes; slot++) {
        if (!_entries[slot].active) {
            continue;
        }
        if (active_index == index) {
            return &_entries[slot];
        }
        active_index++;
    }
    return nullptr;
}

const AssignmentMatrix::Entry *AssignmentMatrix::by_id(uint32_t route_id) const
{
    for (uint8_t slot = 0; slot < max_routes; slot++) {
        if (_entries[slot].active && _entries[slot].route_id == route_id) {
            return &_entries[slot];
        }
    }
    return nullptr;
}

const AssignmentMatrix::Entry *AssignmentMatrix::to_destination(
    uint32_t destination_input_id) const
{
    const int8_t slot = slot_for_destination(destination_input_id);
    return slot < 0 ? nullptr : &_entries[uint8_t(slot)];
}

uint32_t AssignmentMatrix::make_route_id(uint32_t source_output_id,
                                         uint32_t destination_input_id)
{
    if (source_output_id == 0 || destination_input_id == 0) {
        return 0;
    }
    char path[24] = "route/00000000/00000000";
    append_hex32(&path[6], source_output_id);
    append_hex32(&path[15], destination_input_id);
    return Protocol::fnv1a32(path);
}

AssignmentMatrix::SetResult AssignmentMatrix::validate(
    uint32_t source_output_id,
    uint32_t destination_input_id,
    uint16_t flags,
    int8_t replaced_slot) const
{
    const SchemaRegistry &registry = schema_registry();
    const FieldDescriptor *source = registry.field_by_id(source_output_id);
    if (source == nullptr) {
        return SetResult::BAD_SOURCE;
    }
    const FieldDescriptor *destination =
        registry.field_by_id(destination_input_id);
    if (destination == nullptr) {
        return SetResult::BAD_DESTINATION;
    }
    if (source->kind != Protocol::FieldKind::OUTPUT ||
        (source->flags & FIELD_ROUTABLE) == 0) {
        return SetResult::SOURCE_NOT_ROUTABLE;
    }
    if (destination->kind != Protocol::FieldKind::INPUT ||
        (destination->flags & FIELD_ROUTABLE) == 0) {
        return SetResult::DESTINATION_NOT_ROUTABLE;
    }
    if (source->type != destination->type) {
        return SetResult::TYPE_MISMATCH;
    }
    if (flags != 0) {
        return SetResult::UNSUPPORTED_FLAGS;
    }
    if (creates_cycle(source_output_id,
                      destination_input_id,
                      replaced_slot)) {
        return SetResult::CYCLE;
    }
    return SetResult::OK;
}

bool AssignmentMatrix::creates_cycle(uint32_t source_output_id,
                                     uint32_t destination_input_id,
                                     int8_t replaced_slot) const
{
    const SchemaRegistry &registry = schema_registry();
    const FieldDescriptor *source = registry.field_by_id(source_output_id);
    const FieldDescriptor *destination =
        registry.field_by_id(destination_input_id);
    if (source == nullptr || destination == nullptr) {
        return true;
    }

    const uint32_t source_owner = owner_id(*source);
    const uint32_t destination_owner = owner_id(*destination);
    if (source_owner == destination_owner) {
        return true;
    }

    uint32_t pending[max_routes + 1] {};
    uint32_t visited[max_routes + 1] {};
    uint8_t pending_count = 1;
    uint8_t visited_count = 0;
    pending[0] = destination_owner;

    while (pending_count != 0) {
        const uint32_t current = pending[--pending_count];
        if (current == source_owner) {
            return true;
        }

        bool already_visited = false;
        for (uint8_t i = 0; i < visited_count; i++) {
            already_visited |= visited[i] == current;
        }
        if (already_visited) {
            continue;
        }
        visited[visited_count++] = current;

        for (uint8_t slot = 0; slot < max_routes; slot++) {
            if (!_entries[slot].active || slot == replaced_slot) {
                continue;
            }
            const FieldDescriptor *route_source =
                registry.field_by_id(_entries[slot].source_output_id);
            const FieldDescriptor *route_destination =
                registry.field_by_id(_entries[slot].destination_input_id);
            if (route_source == nullptr || route_destination == nullptr ||
                owner_id(*route_source) != current) {
                continue;
            }
            if (pending_count < sizeof(pending) / sizeof(pending[0])) {
                pending[pending_count++] = owner_id(*route_destination);
            }
        }
    }
    return false;
}

int8_t AssignmentMatrix::slot_for_destination(
    uint32_t destination_input_id) const
{
    for (uint8_t slot = 0; slot < max_routes; slot++) {
        if (_entries[slot].active &&
            _entries[slot].destination_input_id == destination_input_id) {
            return int8_t(slot);
        }
    }
    return -1;
}

int8_t AssignmentMatrix::first_free_slot() const
{
    for (uint8_t slot = 0; slot < max_routes; slot++) {
        if (!_entries[slot].active) {
            return int8_t(slot);
        }
    }
    return -1;
}

void AssignmentMatrix::store_slot(uint8_t slot)
{
    const Entry &entry = _entries[slot];
    save_without_gcs(_stored_source[slot], int32_t(0));
    save_without_gcs(_stored_destination[slot],
                     int32_t(entry.destination_input_id));
    save_without_gcs(_stored_flags[slot], int16_t(entry.flags));
    save_without_gcs(_stored_source[slot],
                     int32_t(entry.source_output_id));
}

void AssignmentMatrix::clear_stored_slot(uint8_t slot)
{
    save_without_gcs(_stored_source[slot], int32_t(0));
    save_without_gcs(_stored_destination[slot], int32_t(0));
    save_without_gcs(_stored_flags[slot], int16_t(0));
}

} // namespace Vektor
