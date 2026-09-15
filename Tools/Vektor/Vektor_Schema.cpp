#include "Vektor_Schema.h"

#include "Config.h"
#include "Vektor_SerialCatalog.h"

#include <limits.h>

namespace {

namespace Protocol = Vektor::Protocol;
namespace Catalog = Vektor::SerialCatalog;

static constexpr uint32_t parameter_live_flags =
    Vektor::FIELD_READABLE |
    Vektor::FIELD_WRITABLE |
    Vektor::FIELD_PERSISTENT |
    Vektor::FIELD_HAS_MIN |
    Vektor::FIELD_HAS_MAX |
    Vektor::FIELD_HAS_DEFAULT |
    Vektor::FIELD_APPLY_LIVE;
static constexpr uint32_t parameter_reboot_flags =
    Vektor::FIELD_READABLE |
    Vektor::FIELD_WRITABLE |
    Vektor::FIELD_PERSISTENT |
    Vektor::FIELD_HAS_MIN |
    Vektor::FIELD_HAS_MAX |
    Vektor::FIELD_HAS_DEFAULT |
    Vektor::FIELD_APPLY_REBOOT;
static constexpr uint32_t realtime_flags =
    Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME;
static constexpr uint32_t routable_realtime_flags =
    realtime_flags | Vektor::FIELD_ROUTABLE;

const Vektor::ComponentDescriptor component_descriptors[] = {
#define VEKTOR_COMPONENT(SYMBOL, PATH, TYPE_PATH, INSTANCE_NAME,            \
                          DISPLAY_NAME, INSTANCE_INDEX, FLAGS)              \
    { Catalog::Component::SYMBOL.path, TYPE_PATH, INSTANCE_NAME,            \
      DISPLAY_NAME, INSTANCE_INDEX, FLAGS },
#define VEKTOR_PARAMETER(...)
#define VEKTOR_OBSERVABLE(...)
#define VEKTOR_INPUT(...)
#define VEKTOR_OUTPUT(...)
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT
};

const Vektor::FieldDescriptor field_descriptors[] = {
#define VEKTOR_COMPONENT(...)
#define VEKTOR_PARAMETER(SYMBOL, PATH, AP_NAME, TYPE, OWNER, NAME, DISPLAY, \
                         UNITS, FLAGS, MINIMUM, MAXIMUM, DEFAULT, SLOT)      \
    { Catalog::Parameter::SYMBOL.path, Catalog::Component::OWNER.path,      \
      NAME, DISPLAY, UNITS, Protocol::FieldKind::PARAMETER,                 \
      Protocol::PrimitiveType::TYPE, FLAGS, MINIMUM, MAXIMUM, DEFAULT,      \
      Vektor::FieldSlot::SLOT },
#define VEKTOR_OBSERVABLE(SYMBOL, PATH, TYPE, OWNER, NAME, DISPLAY, UNITS,  \
                          FLAGS, SLOT)                                      \
    { Catalog::Observable::SYMBOL.path, Catalog::Component::OWNER.path,     \
      NAME, DISPLAY, UNITS, Protocol::FieldKind::OBSERVABLE,                \
      Protocol::PrimitiveType::TYPE, FLAGS, 0, 0, 0,                       \
      Vektor::FieldSlot::SLOT },
#define VEKTOR_INPUT(SYMBOL, PATH, TYPE, OWNER, NAME, DISPLAY, UNITS,       \
                     FLAGS, SLOT)                                           \
    { Catalog::Input::SYMBOL.path, Catalog::Component::OWNER.path,          \
      NAME, DISPLAY, UNITS, Protocol::FieldKind::INPUT,                     \
      Protocol::PrimitiveType::TYPE, FLAGS, 0, 0, 0,                       \
      Vektor::FieldSlot::SLOT },
#define VEKTOR_OUTPUT(SYMBOL, PATH, TYPE, OWNER, NAME, DISPLAY, UNITS,      \
                      FLAGS, SLOT)                                          \
    { Catalog::Output::SYMBOL.path, Catalog::Component::OWNER.path,         \
      NAME, DISPLAY, UNITS, Protocol::FieldKind::OUTPUT,                    \
      Protocol::PrimitiveType::TYPE, FLAGS, 0, 0, 0,                       \
      Vektor::FieldSlot::SLOT },
#include "Vektor_SerialCatalog.def"
#undef VEKTOR_COMPONENT
#undef VEKTOR_PARAMETER
#undef VEKTOR_OBSERVABLE
#undef VEKTOR_INPUT
#undef VEKTOR_OUTPUT
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
    writer.u8(field->type == Protocol::PrimitiveType::ENUM ? 2 : 1);
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
    if (field->type == Protocol::PrimitiveType::ENUM) {
        const char *enum_path = enum_table_path_for_slot(field->slot);
        if (enum_path == nullptr || !writer.u32(Protocol::fnv1a32(enum_path))) {
            record_len = 0;
            return false;
        }
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
    case Protocol::PrimitiveType::BOOL:
        return writer.u8(raw == 0 ? 0 : 1);
    case Protocol::PrimitiveType::U8:
    case Protocol::PrimitiveType::I8:
        return writer.u8(uint8_t(raw));
    case Protocol::PrimitiveType::U16:
    case Protocol::PrimitiveType::I16:
        return writer.u16(uint16_t(raw));
    case Protocol::PrimitiveType::U32:
    case Protocol::PrimitiveType::I32:
    case Protocol::PrimitiveType::FLOAT32:
    case Protocol::PrimitiveType::ENUM:
        return writer.u32(raw);
    default:
        return false;
    }
}

bool rcin_channel_for_slot(FieldSlot slot, uint8_t &channel_index)
{
    const uint8_t value = uint8_t(slot);
    const uint8_t first = uint8_t(FieldSlot::RCIN_CHANNEL_1);
    const uint8_t last = uint8_t(FieldSlot::RCIN_CHANNEL_16);
    if (value < first || value > last) {
        return false;
    }
    channel_index = value - first;
    return true;
}

bool rcin_pwm_channel_for_slot(FieldSlot slot, uint8_t &channel_index)
{
    const uint8_t value = uint8_t(slot);
    const uint8_t first = uint8_t(FieldSlot::RCIN_PWM_1);
    const uint8_t last = uint8_t(FieldSlot::RCIN_PWM_16);
    if (value < first || value > last) {
        return false;
    }
    channel_index = value - first;
    return true;
}

bool pwmin_pin_for_slot(FieldSlot slot, uint8_t &channel_index)
{
    const uint8_t value = uint8_t(slot);
    const uint8_t first = uint8_t(FieldSlot::PWMIN_PIN_1);
    const uint8_t last = uint8_t(FieldSlot::PWMIN_PIN_6);
    if (value < first || value > last) {
        return false;
    }
    channel_index = value - first;
    return true;
}

bool pwmin_status_for_slot(FieldSlot slot, uint8_t &channel_index)
{
    const uint8_t value = uint8_t(slot);
    const uint8_t first = uint8_t(FieldSlot::PWMIN_ATTACH_STATUS_1);
    const uint8_t last = uint8_t(FieldSlot::PWMIN_ATTACH_STATUS_6);
    if (value < first || value > last) {
        return false;
    }
    channel_index = value - first;
    return true;
}

bool pwmin_channel_for_slot(FieldSlot slot, uint8_t &channel_index)
{
    const uint8_t value = uint8_t(slot);
    const uint8_t first = uint8_t(FieldSlot::PWMIN_CHANNEL_1);
    const uint8_t last = uint8_t(FieldSlot::PWMIN_CHANNEL_6);
    if (value < first || value > last) {
        return false;
    }
    channel_index = value - first;
    return true;
}

bool pwmout_channel_for_slot(FieldSlot slot, uint8_t &channel_index)
{
    const uint8_t value = uint8_t(slot);
    const uint8_t first = uint8_t(FieldSlot::PWMOUT_CHANNEL_1);
    const uint8_t last = uint8_t(FieldSlot::PWMOUT_CHANNEL_12);
    if (value < first || value > last) {
        return false;
    }
    channel_index = value - first;
    return true;
}

bool pwmout_pulse_channel_for_slot(FieldSlot slot, uint8_t &channel_index)
{
    const uint8_t value = uint8_t(slot);
    const uint8_t first = uint8_t(FieldSlot::PWMOUT_PULSE_1);
    const uint8_t last = uint8_t(FieldSlot::PWMOUT_PULSE_12);
    if (value < first || value > last) {
        return false;
    }
    channel_index = value - first;
    return true;
}

namespace {

bool serial_field_for_slot(Vektor::FieldSlot slot,
                           uint8_t field_offset,
                           uint8_t &serial_index)
{
    const uint8_t value = uint8_t(slot);
    const uint8_t first = uint8_t(Vektor::FieldSlot::SERIAL_0_ROLE);
    const uint8_t last = uint8_t(
        Vektor::FieldSlot::SERIAL_9_REBOOT_REQUIRED);
    if (value < first || value > last ||
        ((value - first) % 5U) != field_offset) {
        return false;
    }
    serial_index = (value - first) / 5U;
    return true;
}

} // namespace

bool serial_role_for_slot(FieldSlot slot, uint8_t &serial_index)
{
    return serial_field_for_slot(slot, 0, serial_index);
}

bool serial_baud_for_slot(FieldSlot slot, uint8_t &serial_index)
{
    return serial_field_for_slot(slot, 1, serial_index);
}

bool serial_active_role_for_slot(FieldSlot slot, uint8_t &serial_index)
{
    return serial_field_for_slot(slot, 2, serial_index);
}

bool serial_status_for_slot(FieldSlot slot, uint8_t &serial_index)
{
    return serial_field_for_slot(slot, 3, serial_index);
}

bool serial_reboot_required_for_slot(FieldSlot slot, uint8_t &serial_index)
{
    return serial_field_for_slot(slot, 4, serial_index);
}

const char *enum_table_path_for_slot(FieldSlot slot)
{
    uint8_t serial_index = 0;
    if (serial_role_for_slot(slot, serial_index) ||
        serial_active_role_for_slot(slot, serial_index)) {
        return "enum/serial_role";
    }
    if (serial_status_for_slot(slot, serial_index) ||
        slot == FieldSlot::USB_STATUS) {
        return "enum/serial_endpoint_status";
    }
    if (slot == FieldSlot::USB_MODE || slot == FieldSlot::USB_ACTIVE_MODE) {
        return "enum/usb_mode";
    }
    return nullptr;
}

} // namespace Vektor
