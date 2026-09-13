#pragma once

#include "Vektor_Protocol.h"

#include <stdint.h>

namespace Vektor {

enum FieldFlag : uint32_t {
    FIELD_READABLE = 1U << 0,
    FIELD_WRITABLE = 1U << 1,
    FIELD_PERSISTENT = 1U << 2,
    FIELD_REALTIME = 1U << 3,
    FIELD_HAS_MIN = 1U << 4,
    FIELD_HAS_MAX = 1U << 5,
    FIELD_HAS_DEFAULT = 1U << 6,
    FIELD_APPLY_LIVE = 1U << 7,
    FIELD_APPLY_RECONFIGURE = 1U << 8,
    FIELD_APPLY_REBOOT = 1U << 9,
    FIELD_ROUTABLE = 1U << 10,
    FIELD_HIDDEN_NORMAL_UI = 1U << 11,
    FIELD_DEVELOPER_ONLY = 1U << 12,
};

enum class FieldSlot : uint8_t {
    RX_FRAMES,
    RX_DROPS,
    TX_DROPS,
    UPTIME_MS,
    LOOP_COUNT,
    LOOP_DT_US,
    LOOP_WORK_US,
    LOOP_MAX_WORK_US,
    SERVICE_RATE_HZ,
    SYS_OPTIONS,
    SYS_DESC_PAGE,
    SYS_PROTOCOL_BAUD,
    VSP_X,
    VSP_Y,
    VSP_SERVO_A,
    VSP_SERVO_B,
};

struct ComponentDescriptor {
    const char *path;
    const char *type_path;
    const char *name;
    const char *display_name;
    uint16_t instance;
    uint32_t flags;
};

struct FieldDescriptor {
    const char *path;
    const char *owner_component_path;
    const char *name;
    const char *display_name;
    const char *units;
    Protocol::FieldKind kind;
    Protocol::PrimitiveType type;
    uint32_t flags;
    int32_t min_value;
    int32_t max_value;
    int32_t default_value;
    FieldSlot slot;
};

class SchemaRegistry {
public:
    uint16_t component_count() const;
    uint16_t field_count() const;
    uint16_t parameter_count() const;

    const ComponentDescriptor *component_by_index(uint16_t index) const;
    const FieldDescriptor *field_by_index(uint16_t index) const;
    const FieldDescriptor *field_by_id(uint32_t field_id) const;

    uint32_t component_id(uint16_t index) const;
    uint32_t component_type_id(uint16_t index) const;
    uint32_t field_id(uint16_t index) const;

    bool build_component_record(uint16_t index,
                                uint8_t *record,
                                uint16_t record_capacity,
                                uint16_t &record_len) const;
    bool build_field_record(uint16_t index,
                            uint8_t *record,
                            uint16_t record_capacity,
                            uint16_t &record_len) const;
};

const SchemaRegistry &schema_registry();
bool field_is_parameter(const FieldDescriptor &field);
bool write_typed_payload(Protocol::PayloadWriter &writer,
                         Protocol::PrimitiveType type,
                         uint32_t raw);

} // namespace Vektor
