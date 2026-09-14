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
    ATTITUDE_ROLL_DEG,
    ATTITUDE_PITCH_DEG,
    ATTITUDE_YAW_DEG,
    ATTITUDE_QUATERNION,
    ATTITUDE_BODY_RATES_RAD_S,
    SYS_OPTIONS,
    SYS_DESC_PAGE,
    SYS_PROTOCOL_BAUD,
    RCIN_PORT,
    RCIN_TIMEOUT_MS,
    RCIN_PROTOCOLS,
    VSP_X,
    VSP_Y,
    VSP_SERVO_A,
    VSP_SERVO_B,
    RCIN_CHANNEL_1,
    RCIN_CHANNEL_2,
    RCIN_CHANNEL_3,
    RCIN_CHANNEL_4,
    RCIN_CHANNEL_5,
    RCIN_CHANNEL_6,
    RCIN_CHANNEL_7,
    RCIN_CHANNEL_8,
    RCIN_CHANNEL_9,
    RCIN_CHANNEL_10,
    RCIN_CHANNEL_11,
    RCIN_CHANNEL_12,
    RCIN_CHANNEL_13,
    RCIN_CHANNEL_14,
    RCIN_CHANNEL_15,
    RCIN_CHANNEL_16,
    RCIN_PWM_1,
    RCIN_PWM_2,
    RCIN_PWM_3,
    RCIN_PWM_4,
    RCIN_PWM_5,
    RCIN_PWM_6,
    RCIN_PWM_7,
    RCIN_PWM_8,
    RCIN_PWM_9,
    RCIN_PWM_10,
    RCIN_PWM_11,
    RCIN_PWM_12,
    RCIN_PWM_13,
    RCIN_PWM_14,
    RCIN_PWM_15,
    RCIN_PWM_16,
    PWMIN_PIN_1,
    PWMIN_PIN_2,
    PWMIN_PIN_3,
    PWMIN_PIN_4,
    PWMIN_PIN_5,
    PWMIN_PIN_6,
    PWMIN_TIMEOUT_MS,
    PWMIN_MIN_US,
    PWMIN_TRIM_US,
    PWMIN_MAX_US,
    PWMOUT_RATE_HZ,
    PWMOUT_MIN_US,
    PWMOUT_TRIM_US,
    PWMOUT_MAX_US,
    PWMOUT_REVERSE_MASK,
    PWMOUT_FAILSAFE_US,
    PWMIN_CHANNEL_1,
    PWMIN_CHANNEL_2,
    PWMIN_CHANNEL_3,
    PWMIN_CHANNEL_4,
    PWMIN_CHANNEL_5,
    PWMIN_CHANNEL_6,
    PWMOUT_CHANNEL_1,
    PWMOUT_CHANNEL_2,
    PWMOUT_CHANNEL_3,
    PWMOUT_CHANNEL_4,
    PWMOUT_CHANNEL_5,
    PWMOUT_CHANNEL_6,
    PWMOUT_CHANNEL_7,
    PWMOUT_CHANNEL_8,
    PWMOUT_CHANNEL_9,
    PWMOUT_CHANNEL_10,
    PWMOUT_CHANNEL_11,
    PWMOUT_CHANNEL_12,
    PWMOUT_PULSE_1,
    PWMOUT_PULSE_2,
    PWMOUT_PULSE_3,
    PWMOUT_PULSE_4,
    PWMOUT_PULSE_5,
    PWMOUT_PULSE_6,
    PWMOUT_PULSE_7,
    PWMOUT_PULSE_8,
    PWMOUT_PULSE_9,
    PWMOUT_PULSE_10,
    PWMOUT_PULSE_11,
    PWMOUT_PULSE_12,
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
bool rcin_channel_for_slot(FieldSlot slot, uint8_t &channel_index);
bool rcin_pwm_channel_for_slot(FieldSlot slot, uint8_t &channel_index);
bool pwmin_pin_for_slot(FieldSlot slot, uint8_t &channel_index);
bool pwmin_channel_for_slot(FieldSlot slot, uint8_t &channel_index);
bool pwmout_channel_for_slot(FieldSlot slot, uint8_t &channel_index);
bool pwmout_pulse_channel_for_slot(FieldSlot slot, uint8_t &channel_index);

} // namespace Vektor
