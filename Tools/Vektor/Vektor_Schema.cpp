#include "Vektor_Schema.h"

#include "Config.h"

#include <limits.h>

namespace {

namespace Protocol = Vektor::Protocol;

static constexpr const char *protocol_component_path = "component/system/0";
static constexpr const char *runtime_component_path = "component/system/1";
static constexpr const char *vsp_component_path = "component/vsp/1";
static constexpr uint32_t parameter_live_flags =
    Vektor::FIELD_READABLE |
    Vektor::FIELD_WRITABLE |
    Vektor::FIELD_PERSISTENT |
    Vektor::FIELD_HAS_MIN |
    Vektor::FIELD_HAS_MAX |
    Vektor::FIELD_HAS_DEFAULT |
    Vektor::FIELD_APPLY_LIVE;

const Vektor::ComponentDescriptor component_descriptors[] = {
    {
        protocol_component_path,
        "component_type/system/protocol",
        "protocol",
        "Protocol",
        0,
        0,
    },
    {
        runtime_component_path,
        "component_type/system/runtime",
        "runtime",
        "Runtime",
        1,
        0,
    },
    {
        vsp_component_path,
        "component_type/control/vsp",
        "vsp1",
        "VSP 1",
        1,
        0,
    },
};

const Vektor::FieldDescriptor field_descriptors[] = {
    {
        "component/system/0/observable/rx_frames",
        protocol_component_path,
        "rx_frames",
        "RX Frames",
        "",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::RX_FRAMES,
    },
    {
        "component/system/0/observable/rx_drops",
        protocol_component_path,
        "rx_drops",
        "RX Drops",
        "",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::RX_DROPS,
    },
    {
        "component/system/0/observable/tx_drops",
        protocol_component_path,
        "tx_drops",
        "TX Drops",
        "",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::TX_DROPS,
    },
    {
        "component/system/1/observable/uptime_ms",
        runtime_component_path,
        "uptime_ms",
        "Uptime",
        "ms",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::UPTIME_MS,
    },
    {
        "component/system/1/observable/loop_count",
        runtime_component_path,
        "loop_count",
        "Loop Count",
        "",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::LOOP_COUNT,
    },
    {
        "component/system/1/observable/loop_dt_us",
        runtime_component_path,
        "loop_dt_us",
        "Loop Delta",
        "us",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::LOOP_DT_US,
    },
    {
        "component/system/1/observable/loop_work_us",
        runtime_component_path,
        "loop_work_us",
        "Loop Work",
        "us",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::LOOP_WORK_US,
    },
    {
        "component/system/1/observable/loop_max_work_us",
        runtime_component_path,
        "loop_max_work_us",
        "Max Loop Work",
        "us",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::LOOP_MAX_WORK_US,
    },
    {
        "component/system/1/observable/service_rate_hz",
        runtime_component_path,
        "service_rate_hz",
        "Service Rate",
        "Hz",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::U16,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::SERVICE_RATE_HZ,
    },
    {
        "component/system/0/parameter/sys_options",
        protocol_component_path,
        "sys_options",
        "System Options",
        "",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I32,
        parameter_live_flags,
        0,
        INT32_MAX,
        0,
        Vektor::FieldSlot::SYS_OPTIONS,
    },
    {
        "component/system/0/parameter/sys_desc_page",
        protocol_component_path,
        "sys_desc_page",
        "Descriptor Page Size",
        "records",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        Vektor::min_describe_page_records,
        Vektor::max_describe_page_records,
        Vektor::default_describe_page_records,
        Vektor::FieldSlot::SYS_DESC_PAGE,
    },
    {
        "component/system/0/parameter/sys_protocol_baud",
        protocol_component_path,
        "sys_protocol_baud",
        "Protocol Baud",
        "baud",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I32,
        Vektor::FIELD_READABLE |
        Vektor::FIELD_WRITABLE |
        Vektor::FIELD_PERSISTENT |
        Vektor::FIELD_HAS_MIN |
        Vektor::FIELD_HAS_MAX |
        Vektor::FIELD_HAS_DEFAULT |
        Vektor::FIELD_APPLY_REBOOT,
        Vektor::protocol_baud_min,
        Vektor::protocol_baud_max,
        Vektor::protocol_baud,
        Vektor::FieldSlot::SYS_PROTOCOL_BAUD,
    },
    {
        "component/vsp/1/input/x",
        vsp_component_path,
        "x",
        "X Command",
        "normalized",
        Protocol::FieldKind::INPUT,
        Protocol::PrimitiveType::FLOAT32,
        Vektor::FIELD_READABLE |
        Vektor::FIELD_REALTIME |
        Vektor::FIELD_ROUTABLE,
        0,
        0,
        0,
        Vektor::FieldSlot::VSP_X,
    },
    {
        "component/vsp/1/input/y",
        vsp_component_path,
        "y",
        "Y Command",
        "normalized",
        Protocol::FieldKind::INPUT,
        Protocol::PrimitiveType::FLOAT32,
        Vektor::FIELD_READABLE |
        Vektor::FIELD_REALTIME |
        Vektor::FIELD_ROUTABLE,
        0,
        0,
        0,
        Vektor::FieldSlot::VSP_Y,
    },
    {
        "component/vsp/1/output/servo_a",
        vsp_component_path,
        "servo_a",
        "Servo A",
        "normalized",
        Protocol::FieldKind::OUTPUT,
        Protocol::PrimitiveType::FLOAT32,
        Vektor::FIELD_READABLE |
        Vektor::FIELD_REALTIME |
        Vektor::FIELD_ROUTABLE,
        0,
        0,
        0,
        Vektor::FieldSlot::VSP_SERVO_A,
    },
    {
        "component/vsp/1/output/servo_b",
        vsp_component_path,
        "servo_b",
        "Servo B",
        "normalized",
        Protocol::FieldKind::OUTPUT,
        Protocol::PrimitiveType::FLOAT32,
        Vektor::FIELD_READABLE |
        Vektor::FIELD_REALTIME |
        Vektor::FIELD_ROUTABLE,
        0,
        0,
        0,
        Vektor::FieldSlot::VSP_SERVO_B,
    },
};

static constexpr uint16_t component_descriptor_count =
    sizeof(component_descriptors) / sizeof(component_descriptors[0]);
static constexpr uint16_t field_descriptor_count =
    sizeof(field_descriptors) / sizeof(field_descriptors[0]);

} // namespace

namespace Vektor {

uint16_t SchemaRegistry::component_count() const
{
    return component_descriptor_count;
}

uint16_t SchemaRegistry::field_count() const
{
    return field_descriptor_count;
}

uint16_t SchemaRegistry::parameter_count() const
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < field_count(); i++) {
        const FieldDescriptor &field = field_descriptors[i];
        if (field_is_parameter(field) &&
            (field.flags & FIELD_READABLE) != 0) {
            count++;
        }
    }
    return count;
}

const ComponentDescriptor *SchemaRegistry::component_by_index(
    uint16_t index) const
{
    return index < component_count() ? &component_descriptors[index] : nullptr;
}

const FieldDescriptor *SchemaRegistry::field_by_index(uint16_t index) const
{
    return index < field_count() ? &field_descriptors[index] : nullptr;
}

const FieldDescriptor *SchemaRegistry::field_by_id(uint32_t requested_id) const
{
    for (uint16_t i = 0; i < field_count(); i++) {
        if (field_id(i) == requested_id) {
            return &field_descriptors[i];
        }
    }
    return nullptr;
}

uint32_t SchemaRegistry::component_id(uint16_t index) const
{
    const ComponentDescriptor *component = component_by_index(index);
    return component == nullptr ? 0 : Protocol::fnv1a32(component->path);
}

uint32_t SchemaRegistry::component_type_id(uint16_t index) const
{
    const ComponentDescriptor *component = component_by_index(index);
    return component == nullptr ? 0 : Protocol::fnv1a32(component->type_path);
}

uint32_t SchemaRegistry::field_id(uint16_t index) const
{
    const FieldDescriptor *field = field_by_index(index);
    return field == nullptr ? 0 : Protocol::fnv1a32(field->path);
}

bool SchemaRegistry::build_component_record(uint16_t index,
                                            uint8_t *record,
                                            uint16_t record_capacity,
                                            uint16_t &record_len) const
{
    const ComponentDescriptor *component = component_by_index(index);
    if (component == nullptr) {
        record_len = 0;
        return false;
    }

    Protocol::PayloadWriter writer(record, record_capacity);
    writer.u8(1); // record_version
    writer.u32(component_id(index));
    writer.u32(component_type_id(index));
    writer.u16(component->instance);
    writer.u32(component->flags);
    writer.str8(component->name);
    writer.str8(component->display_name);
    record_len = writer.length();
    return writer.ok();
}

bool SchemaRegistry::build_field_record(uint16_t index,
                                        uint8_t *record,
                                        uint16_t record_capacity,
                                        uint16_t &record_len) const
{
    const FieldDescriptor *field = field_by_index(index);
    if (field == nullptr) {
        record_len = 0;
        return false;
    }

    Protocol::PayloadWriter writer(record, record_capacity);
    writer.u8(1); // record_version
    writer.u32(field_id(index));
    writer.u32(Protocol::fnv1a32(field->owner_component_path));
    writer.u8(uint8_t(field->kind));
    writer.u8(uint8_t(field->type));
    writer.u32(field->flags);
    writer.str8(field->name);
    writer.str8(field->display_name);
    writer.str8(field->units);
    if ((field->flags & FIELD_HAS_MIN) != 0 &&
        !write_typed_payload(writer, field->type, uint32_t(field->min_value))) {
        record_len = 0;
        return false;
    }
    if ((field->flags & FIELD_HAS_MAX) != 0 &&
        !write_typed_payload(writer, field->type, uint32_t(field->max_value))) {
        record_len = 0;
        return false;
    }
    if ((field->flags & FIELD_HAS_DEFAULT) != 0 &&
        !write_typed_payload(writer,
                             field->type,
                             uint32_t(field->default_value))) {
        record_len = 0;
        return false;
    }
    record_len = writer.length();
    return writer.ok();
}

const SchemaRegistry &schema_registry()
{
    static const SchemaRegistry registry;
    return registry;
}

bool field_is_parameter(const FieldDescriptor &field)
{
    return field.kind == Protocol::FieldKind::PARAMETER;
}

bool write_typed_payload(Protocol::PayloadWriter &writer,
                         Protocol::PrimitiveType type,
                         uint32_t raw)
{
    switch (type) {
    case Protocol::PrimitiveType::U16:
    case Protocol::PrimitiveType::I16:
        return writer.u16(uint16_t(raw));
    case Protocol::PrimitiveType::U32:
    case Protocol::PrimitiveType::I32:
    case Protocol::PrimitiveType::FLOAT32:
        return writer.u32(raw);
    default:
        return false;
    }
}

} // namespace Vektor
