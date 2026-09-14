#include "Vektor_Schema.h"

#include "Config.h"
#include "Vektor_SerialCatalog.h"

#include <limits.h>

namespace {

namespace Protocol = Vektor::Protocol;
namespace Catalog = Vektor::SerialCatalog;

static constexpr const char *protocol_component_path =
    Catalog::Component::PROTOCOL.path;
static constexpr const char *runtime_component_path =
    Catalog::Component::RUNTIME.path;
static constexpr const char *attitude_component_path =
    Catalog::Component::ATTITUDE.path;
static constexpr const char *vsp_component_path = Catalog::Component::VSP.path;
static constexpr const char *rcin_component_path = Catalog::Component::RCIN.path;
static constexpr const char *pwmin_component_path =
    Catalog::Component::PWM_INPUT.path;
static constexpr const char *pwmout_component_path =
    Catalog::Component::PWM_OUTPUT.path;
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
        attitude_component_path,
        "component_type/sensor/attitude",
        "attitude0",
        "Attitude",
        0,
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
    {
        rcin_component_path,
        "component_type/input/rcin",
        "rcin0",
        "RC Input",
        0,
        0,
    },
    {
        pwmin_component_path,
        "component_type/input/pwm",
        "pwmin0",
        "PWM Input",
        0,
        0,
    },
    {
        pwmout_component_path,
        "component_type/output/pwm",
        "pwmout0",
        "PWM Output",
        0,
        0,
    },
};

#define VEKTOR_RCIN_FIELD(CHANNEL)                                           \
    {                                                                        \
        Catalog::Output::RCIN_CHANNEL_##CHANNEL.path,                        \
        rcin_component_path,                                                 \
        "channel_" #CHANNEL,                                                \
        "Channel " #CHANNEL,                                                \
        "normalized",                                                       \
        Protocol::FieldKind::OUTPUT,                                         \
        Protocol::PrimitiveType::FLOAT32,                                    \
        Vektor::FIELD_READABLE |                                             \
        Vektor::FIELD_REALTIME |                                             \
        Vektor::FIELD_ROUTABLE,                                              \
        0,                                                                   \
        0,                                                                   \
        0,                                                                   \
        Vektor::FieldSlot::RCIN_CHANNEL_##CHANNEL,                           \
    }

#define VEKTOR_RCIN_PWM_FIELD(CHANNEL)                                      \
    {                                                                        \
        Catalog::Observable::RCIN_CHANNEL_##CHANNEL##_US.path,               \
        rcin_component_path,                                                 \
        "channel_" #CHANNEL "_us",                                         \
        "Channel " #CHANNEL " Pulse",                                      \
        "us",                                                               \
        Protocol::FieldKind::OBSERVABLE,                                     \
        Protocol::PrimitiveType::U16,                                        \
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,                     \
        0,                                                                   \
        0,                                                                   \
        0,                                                                   \
        Vektor::FieldSlot::RCIN_PWM_##CHANNEL,                               \
    }

#define VEKTOR_PWMIN_PIN_FIELD(CHANNEL)                                      \
    {                                                                        \
        Catalog::Parameter::PWMIN_CHANNEL_##CHANNEL##_PIN.path,              \
        pwmin_component_path,                                                \
        "channel_" #CHANNEL "_pin",                                      \
        "Channel " #CHANNEL " Pin",                                      \
        "GPIO",                                                             \
        Protocol::FieldKind::PARAMETER,                                       \
        Protocol::PrimitiveType::I16,                                         \
        parameter_reboot_flags,                                               \
        -1,                                                                  \
        UINT8_MAX,                                                           \
        -1,                                                                  \
        Vektor::FieldSlot::PWMIN_PIN_##CHANNEL,                              \
    }

#define VEKTOR_PWMIN_FIELD(CHANNEL)                                          \
    {                                                                        \
        Catalog::Output::PWMIN_CHANNEL_##CHANNEL.path,                       \
        pwmin_component_path,                                                \
        "channel_" #CHANNEL,                                               \
        "Channel " #CHANNEL,                                               \
        "normalized",                                                       \
        Protocol::FieldKind::OUTPUT,                                          \
        Protocol::PrimitiveType::FLOAT32,                                     \
        Vektor::FIELD_READABLE |                                             \
        Vektor::FIELD_REALTIME |                                             \
        Vektor::FIELD_ROUTABLE,                                              \
        0,                                                                   \
        0,                                                                   \
        0,                                                                   \
        Vektor::FieldSlot::PWMIN_CHANNEL_##CHANNEL,                          \
    }

#define VEKTOR_PWMOUT_FIELD(CHANNEL)                                         \
    {                                                                        \
        Catalog::Input::PWMOUT_CHANNEL_##CHANNEL.path,                       \
        pwmout_component_path,                                               \
        "channel_" #CHANNEL,                                               \
        "Channel " #CHANNEL,                                               \
        "normalized",                                                       \
        Protocol::FieldKind::INPUT,                                           \
        Protocol::PrimitiveType::FLOAT32,                                     \
        Vektor::FIELD_READABLE |                                             \
        Vektor::FIELD_REALTIME |                                             \
        Vektor::FIELD_ROUTABLE,                                              \
        0,                                                                   \
        0,                                                                   \
        0,                                                                   \
        Vektor::FieldSlot::PWMOUT_CHANNEL_##CHANNEL,                         \
    }

#define VEKTOR_PWMOUT_PULSE_FIELD(CHANNEL)                                  \
    {                                                                        \
        Catalog::Output::PWMOUT_CHANNEL_##CHANNEL.path,                      \
        pwmout_component_path,                                               \
        "channel_" #CHANNEL,                                                \
        "Channel " #CHANNEL " Pulse",                                      \
        "us",                                                               \
        Protocol::FieldKind::OUTPUT,                                         \
        Protocol::PrimitiveType::U16,                                        \
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,                     \
        0,                                                                   \
        0,                                                                   \
        0,                                                                   \
        Vektor::FieldSlot::PWMOUT_PULSE_##CHANNEL,                           \
    }

const Vektor::FieldDescriptor field_descriptors[] = {
    {
        Catalog::Observable::RX_FRAMES.path,
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
        Catalog::Observable::RX_DROPS.path,
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
        Catalog::Observable::TX_DROPS.path,
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
        Catalog::Observable::UPTIME_MS.path,
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
        Catalog::Observable::LOOP_COUNT.path,
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
        Catalog::Observable::LOOP_DT_US.path,
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
        Catalog::Observable::LOOP_WORK_US.path,
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
        Catalog::Observable::LOOP_MAX_WORK_US.path,
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
        Catalog::Observable::SERVICE_RATE_HZ.path,
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
        Catalog::Observable::ATTITUDE_ROLL_DEG.path,
        attitude_component_path,
        "roll_deg",
        "Roll",
        "deg",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::FLOAT32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::ATTITUDE_ROLL_DEG,
    },
    {
        Catalog::Observable::ATTITUDE_PITCH_DEG.path,
        attitude_component_path,
        "pitch_deg",
        "Pitch",
        "deg",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::FLOAT32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::ATTITUDE_PITCH_DEG,
    },
    {
        Catalog::Observable::ATTITUDE_YAW_DEG.path,
        attitude_component_path,
        "yaw_deg",
        "Yaw",
        "deg",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::FLOAT32,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::ATTITUDE_YAW_DEG,
    },
    {
        Catalog::Observable::ATTITUDE_QUATERNION.path,
        attitude_component_path,
        "quaternion",
        "Quaternion",
        "",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::QUATERNIONF,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::ATTITUDE_QUATERNION,
    },
    {
        Catalog::Observable::ATTITUDE_BODY_RATES_RAD_S.path,
        attitude_component_path,
        "body_rates_rad_s",
        "Body Rates",
        "rad/s",
        Protocol::FieldKind::OBSERVABLE,
        Protocol::PrimitiveType::VECTOR3F,
        Vektor::FIELD_READABLE | Vektor::FIELD_REALTIME,
        0,
        0,
        0,
        Vektor::FieldSlot::ATTITUDE_BODY_RATES_RAD_S,
    },
    {
        Catalog::Parameter::SYS_OPTIONS.path,
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
        Catalog::Parameter::SYS_DESC_PAGE.path,
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
        Catalog::Parameter::SYS_PROTOCOL_BAUD.path,
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
        Catalog::Parameter::RCIN_UART_PORT.path,
        rcin_component_path,
        "uart_port",
        "Receiver UART",
        "serial index",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        Vektor::FIELD_READABLE |
        Vektor::FIELD_WRITABLE |
        Vektor::FIELD_PERSISTENT |
        Vektor::FIELD_HAS_MIN |
        Vektor::FIELD_HAS_MAX |
        Vektor::FIELD_HAS_DEFAULT |
        Vektor::FIELD_APPLY_REBOOT,
        Vektor::rcin_port_min,
        Vektor::rcin_port_max,
        Vektor::default_rcin_port,
        Vektor::FieldSlot::RCIN_PORT,
    },
    {
        Catalog::Parameter::RCIN_TIMEOUT_MS.path,
        rcin_component_path,
        "timeout_ms",
        "RC Input Timeout",
        "ms",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        Vektor::rcin_timeout_ms_min,
        Vektor::rcin_timeout_ms_max,
        Vektor::default_rcin_timeout_ms,
        Vektor::FieldSlot::RCIN_TIMEOUT_MS,
    },
    {
        Catalog::Parameter::RCIN_PROTOCOL_MASK.path,
        rcin_component_path,
        "protocol_mask",
        "Receiver Protocols",
        "bitmask",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I32,
        parameter_live_flags,
        0,
        Vektor::rcin_protocol_mask_max,
        1,
        Vektor::FieldSlot::RCIN_PROTOCOLS,
    },
    {
        Catalog::Input::VSP_X.path,
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
        Catalog::Input::VSP_Y.path,
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
        Catalog::Output::VSP_SERVO_A.path,
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
        Catalog::Output::VSP_SERVO_B.path,
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
    VEKTOR_RCIN_FIELD(1),
    VEKTOR_RCIN_FIELD(2),
    VEKTOR_RCIN_FIELD(3),
    VEKTOR_RCIN_FIELD(4),
    VEKTOR_RCIN_FIELD(5),
    VEKTOR_RCIN_FIELD(6),
    VEKTOR_RCIN_FIELD(7),
    VEKTOR_RCIN_FIELD(8),
    VEKTOR_RCIN_FIELD(9),
    VEKTOR_RCIN_FIELD(10),
    VEKTOR_RCIN_FIELD(11),
    VEKTOR_RCIN_FIELD(12),
    VEKTOR_RCIN_FIELD(13),
    VEKTOR_RCIN_FIELD(14),
    VEKTOR_RCIN_FIELD(15),
    VEKTOR_RCIN_FIELD(16),
    VEKTOR_RCIN_PWM_FIELD(1),
    VEKTOR_RCIN_PWM_FIELD(2),
    VEKTOR_RCIN_PWM_FIELD(3),
    VEKTOR_RCIN_PWM_FIELD(4),
    VEKTOR_RCIN_PWM_FIELD(5),
    VEKTOR_RCIN_PWM_FIELD(6),
    VEKTOR_RCIN_PWM_FIELD(7),
    VEKTOR_RCIN_PWM_FIELD(8),
    VEKTOR_RCIN_PWM_FIELD(9),
    VEKTOR_RCIN_PWM_FIELD(10),
    VEKTOR_RCIN_PWM_FIELD(11),
    VEKTOR_RCIN_PWM_FIELD(12),
    VEKTOR_RCIN_PWM_FIELD(13),
    VEKTOR_RCIN_PWM_FIELD(14),
    VEKTOR_RCIN_PWM_FIELD(15),
    VEKTOR_RCIN_PWM_FIELD(16),
    VEKTOR_PWMIN_PIN_FIELD(1),
    VEKTOR_PWMIN_PIN_FIELD(2),
    VEKTOR_PWMIN_PIN_FIELD(3),
    VEKTOR_PWMIN_PIN_FIELD(4),
    VEKTOR_PWMIN_PIN_FIELD(5),
    VEKTOR_PWMIN_PIN_FIELD(6),
    {
        Catalog::Parameter::PWMIN_TIMEOUT_MS.path,
        pwmin_component_path,
        "timeout_ms",
        "Input Timeout",
        "ms",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        Vektor::pwmin_timeout_ms_min,
        Vektor::pwmin_timeout_ms_max,
        Vektor::default_pwmin_timeout_ms,
        Vektor::FieldSlot::PWMIN_TIMEOUT_MS,
    },
    {
        Catalog::Parameter::PWMIN_MINIMUM_US.path,
        pwmin_component_path,
        "minimum_us",
        "Input Minimum",
        "PWM",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        Vektor::pwm_calibration_min_us,
        Vektor::pwm_calibration_max_us,
        Vektor::default_pwm_min_us,
        Vektor::FieldSlot::PWMIN_MIN_US,
    },
    {
        Catalog::Parameter::PWMIN_TRIM_US.path,
        pwmin_component_path,
        "trim_us",
        "Input Trim",
        "PWM",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        Vektor::pwm_calibration_min_us,
        Vektor::pwm_calibration_max_us,
        Vektor::default_pwm_trim_us,
        Vektor::FieldSlot::PWMIN_TRIM_US,
    },
    {
        Catalog::Parameter::PWMIN_MAXIMUM_US.path,
        pwmin_component_path,
        "maximum_us",
        "Input Maximum",
        "PWM",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        Vektor::pwm_calibration_min_us,
        Vektor::pwm_calibration_max_us,
        Vektor::default_pwm_max_us,
        Vektor::FieldSlot::PWMIN_MAX_US,
    },
    {
        Catalog::Parameter::PWMOUT_RATE_HZ.path,
        pwmout_component_path,
        "rate_hz",
        "Output Rate",
        "Hz",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        50,
        330,
        Vektor::default_pwm_rate_hz,
        Vektor::FieldSlot::PWMOUT_RATE_HZ,
    },
    {
        Catalog::Parameter::PWMOUT_MINIMUM_US.path,
        pwmout_component_path,
        "minimum_us",
        "Output Minimum",
        "PWM",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        Vektor::pwm_calibration_min_us,
        Vektor::pwm_calibration_max_us,
        Vektor::default_pwm_min_us,
        Vektor::FieldSlot::PWMOUT_MIN_US,
    },
    {
        Catalog::Parameter::PWMOUT_TRIM_US.path,
        pwmout_component_path,
        "trim_us",
        "Output Trim",
        "PWM",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        Vektor::pwm_calibration_min_us,
        Vektor::pwm_calibration_max_us,
        Vektor::default_pwm_trim_us,
        Vektor::FieldSlot::PWMOUT_TRIM_US,
    },
    {
        Catalog::Parameter::PWMOUT_MAXIMUM_US.path,
        pwmout_component_path,
        "maximum_us",
        "Output Maximum",
        "PWM",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        Vektor::pwm_calibration_min_us,
        Vektor::pwm_calibration_max_us,
        Vektor::default_pwm_max_us,
        Vektor::FieldSlot::PWMOUT_MAX_US,
    },
    {
        Catalog::Parameter::PWMOUT_REVERSE_MASK.path,
        pwmout_component_path,
        "reverse_mask",
        "Reverse Mask",
        "bitmask",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        0,
        Vektor::pwm_reverse_mask_max,
        0,
        Vektor::FieldSlot::PWMOUT_REVERSE_MASK,
    },
    {
        Catalog::Parameter::PWMOUT_FAILSAFE_US.path,
        pwmout_component_path,
        "failsafe_us",
        "Failsafe Pulse",
        "PWM",
        Protocol::FieldKind::PARAMETER,
        Protocol::PrimitiveType::I16,
        parameter_live_flags,
        0,
        Vektor::pwm_calibration_max_us,
        0,
        Vektor::FieldSlot::PWMOUT_FAILSAFE_US,
    },
    VEKTOR_PWMIN_FIELD(1),
    VEKTOR_PWMIN_FIELD(2),
    VEKTOR_PWMIN_FIELD(3),
    VEKTOR_PWMIN_FIELD(4),
    VEKTOR_PWMIN_FIELD(5),
    VEKTOR_PWMIN_FIELD(6),
    VEKTOR_PWMOUT_FIELD(1),
    VEKTOR_PWMOUT_FIELD(2),
    VEKTOR_PWMOUT_FIELD(3),
    VEKTOR_PWMOUT_FIELD(4),
    VEKTOR_PWMOUT_FIELD(5),
    VEKTOR_PWMOUT_FIELD(6),
    VEKTOR_PWMOUT_FIELD(7),
    VEKTOR_PWMOUT_FIELD(8),
    VEKTOR_PWMOUT_FIELD(9),
    VEKTOR_PWMOUT_FIELD(10),
    VEKTOR_PWMOUT_FIELD(11),
    VEKTOR_PWMOUT_FIELD(12),
    VEKTOR_PWMOUT_PULSE_FIELD(1),
    VEKTOR_PWMOUT_PULSE_FIELD(2),
    VEKTOR_PWMOUT_PULSE_FIELD(3),
    VEKTOR_PWMOUT_PULSE_FIELD(4),
    VEKTOR_PWMOUT_PULSE_FIELD(5),
    VEKTOR_PWMOUT_PULSE_FIELD(6),
    VEKTOR_PWMOUT_PULSE_FIELD(7),
    VEKTOR_PWMOUT_PULSE_FIELD(8),
    VEKTOR_PWMOUT_PULSE_FIELD(9),
    VEKTOR_PWMOUT_PULSE_FIELD(10),
    VEKTOR_PWMOUT_PULSE_FIELD(11),
    VEKTOR_PWMOUT_PULSE_FIELD(12),
};

#undef VEKTOR_RCIN_FIELD
#undef VEKTOR_RCIN_PWM_FIELD
#undef VEKTOR_PWMIN_PIN_FIELD
#undef VEKTOR_PWMIN_FIELD
#undef VEKTOR_PWMOUT_FIELD
#undef VEKTOR_PWMOUT_PULSE_FIELD

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

} // namespace Vektor
