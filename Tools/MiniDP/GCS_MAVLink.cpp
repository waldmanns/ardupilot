#include "GCS_MAVLink.h"
#include "MiniDP.h"

#include <AP_Common/AP_FWVersion.h>
#include <AP_Math/AP_Math.h>

#include <limits.h>

#if HAL_GCS_ENABLED

namespace {

bool parse_uint8_param(
    const float value,
    const uint8_t min_value,
    const uint8_t max_value,
    uint8_t &parsed)
{
    if (value < float(min_value) || value > float(max_value)) {
        return false;
    }

    parsed = uint8_t(value + 0.5f);
    return value >= float(parsed) - 0.001f &&
           value <= float(parsed) + 0.001f;
}

bool mode_from_custom_mode(const uint32_t custom_mode, MiniDP_Mode &mode)
{
    switch (custom_mode) {
    case uint32_t(MiniDP_Mode::MANUAL):
        mode = MiniDP_Mode::MANUAL;
        return true;
    case uint32_t(MiniDP_Mode::HEADING_HOLD):
        mode = MiniDP_Mode::HEADING_HOLD;
        return true;
    case uint32_t(MiniDP_Mode::DP_HOLD):
        mode = MiniDP_Mode::DP_HOLD;
        return true;
    default:
        break;
    }
    return false;
}

} // namespace

void GCS_MAVLINK_MiniDP::send_minidp_text(MAV_SEVERITY severity, const char *text) const
{
    if (HAVE_PAYLOAD_SPACE(chan, STATUSTEXT)) {
        mavlink_msg_statustext_send(chan, severity, text, 0, 0);
    }
}

void GCS_MiniDP::send_minidp_text(MAV_SEVERITY severity, const char *text) const
{
    for (uint8_t i = 0; i < num_gcs(); i++) {
        const auto *backend = static_cast<const GCS_MAVLINK_MiniDP *>(chan(i));
        if (backend != nullptr) {
            backend->send_minidp_text(severity, text);
        }
    }
}

void GCS_MAVLINK_MiniDP::send_minidp_firmware_identity() const
{
    const char *firmware = AP::fwversion().fw_short_string;
    if (firmware == nullptr) {
        firmware = "MiniDP";
    }

    char text[50];
    AP_HAL::get_HAL().util->snprintf(
        text,
        sizeof(text),
        "Firmware: %s",
        firmware);
    send_minidp_text(MAV_SEVERITY_INFO, text);
}

void GCS_MiniDP::send_minidp_heartbeat() const
{
    for (uint8_t i = 0; i < num_gcs(); i++) {
        const GCS_MAVLINK *backend = chan(i);
        if (backend != nullptr) {
            backend->send_heartbeat();
        }
    }
}

uint8_t GCS_MAVLINK_MiniDP::base_mode() const
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (minidp.is_armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    if (minidp.get_mode() == MiniDP_Mode::MANUAL) {
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
    return base_mode;
}

MAV_STATE GCS_MAVLINK_MiniDP::vehicle_system_status() const
{
    if (minidp.get_mode() == MiniDP_Mode::FAILSAFE) {
        return MAV_STATE_CRITICAL;
    }
    return minidp.is_armed() ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
}

void GCS_MAVLINK_MiniDP::send_minidp_sys_status() const
{
    const MiniDP_State &state = minidp.get_state();

    uint32_t present =
        MAV_SYS_STATUS_SENSOR_3D_GYRO |
        MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        MAV_SYS_STATUS_SENSOR_3D_MAG |
        MAV_SYS_STATUS_SENSOR_GPS |
        MAV_SYS_STATUS_AHRS;
#if MINIDP_BARO_ENABLED
    present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
#endif
#if AP_BATTERY_ENABLED
    const AP_BattMonitor &battery = AP::battery();
    if (battery.num_instances() > 0) {
        present |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#endif
    uint32_t enabled = present;
    uint32_t healthy = 0;

    if (state.imu_healthy) {
        healthy |= MAV_SYS_STATUS_SENSOR_3D_GYRO |
                   MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }
    if (state.compass_healthy) {
        healthy |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (state.gps_fix_type >= GPS_FIX_TYPE_2D_FIX) {
        healthy |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (state.ekf_healthy) {
        healthy |= MAV_SYS_STATUS_AHRS;
    }
#if AP_BATTERY_ENABLED
    uint16_t battery_voltage_mv = UINT16_MAX;
    int16_t battery_current_ca = -1;
    int8_t battery_remaining_pct = -1;
    if (battery.num_instances() > 0) {
        enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
        if (battery.healthy() && !battery.has_failsafed()) {
            healthy |= MAV_SYS_STATUS_SENSOR_BATTERY;

            battery_voltage_mv = uint16_t(constrain_float(
                battery.gcs_voltage() * 1000.0f,
                0.0f,
                float(UINT16_MAX)));

            float current_amps = 0.0f;
            if (battery.current_amps(current_amps)) {
                battery_current_ca = int16_t(constrain_float(
                    current_amps * 100.0f,
                    -float(INT16_MAX),
                    float(INT16_MAX)));
            }

            uint8_t percentage = 0;
            if (battery.capacity_remaining_pct(percentage)) {
                battery_remaining_pct =
                    int8_t(MIN(percentage, uint8_t(INT8_MAX)));
            }
        }
    }
#else
    constexpr uint16_t battery_voltage_mv = UINT16_MAX;
    constexpr int16_t battery_current_ca = -1;
    constexpr int8_t battery_remaining_pct = -1;
#endif

    mavlink_msg_sys_status_send(
        chan,
        present,
        enabled,
        healthy,
        0,
        battery_voltage_mv,
        battery_current_ca,
        battery_remaining_pct,
        0,
        0,
        0,
        0,
        0,
        0);
}

bool GCS_MAVLINK_MiniDP::try_send_message(const enum ap_message id)
{
    switch (id) {
    case MSG_SYS_STATUS:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_minidp_sys_status();
        return true;
#if AP_BATTERY_ENABLED
    case MSG_BATTERY_STATUS:
        return send_battery_status();
#endif
#if HAL_WITH_ESC_TELEM
    case MSG_ESC_TELEMETRY:
        return true;
#endif
    case MSG_WIND:
        return true;
    case MSG_AUTOPILOT_VERSION:
        if (GCS_MAVLINK::try_send_message(id)) {
            send_minidp_firmware_identity();
            return true;
        }
        return false;
    default:
        return GCS_MAVLINK::try_send_message(id);
    }
}

void GCS_MAVLINK_MiniDP::handle_manual_control_axes(
    const mavlink_manual_control_t &packet,
    const uint32_t tnow)
{
    minidp.record_mavlink_manual_control(
        tnow,
        packet.x,
        packet.y,
        packet.r);
}

void GCS_MAVLINK_MiniDP::send_nav_controller_output() const
{
    const MiniDP_ModeTarget &target = minidp.get_mode_target();
    if (!target.yaw_valid) {
        return;
    }

    const float target_heading_deg = wrap_360(degrees(target.yaw_rad));

    mavlink_msg_nav_controller_output_send(
        chan,
        0.0f,
        0.0f,
        target_heading_deg,
        target_heading_deg,
        0,
        0.0f,
        0.0f,
        0.0f);
}

void GCS_MAVLINK_MiniDP::send_position_target_global_int()
{
    const MiniDP_State &state = minidp.get_state();
    const MiniDP_ModeTarget &target = minidp.get_mode_target();
    if (!target.position_valid ||
        !state.origin_valid ||
        target.origin_id != state.origin_id) {
        return;
    }

    Location target_location;
    if (!minidp.ahrs.get_origin(target_location)) {
        return;
    }
    target_location.offset(target.pos_n_m, target.pos_e_m);

    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    uint16_t type_mask =
        POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE |
        POSITION_TARGET_TYPEMASK_LAST_BYTE;
    if (!target.yaw_valid) {
        type_mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    }

    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(),
        MAV_FRAME_GLOBAL,
        type_mask,
        target_location.lat,
        target_location.lng,
        target_location.alt * 0.01f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        target.yaw_valid ? target.yaw_rad : 0.0f,
        0.0f);
}

void GCS_MAVLINK_MiniDP::send_position_target_local_ned()
{
    const MiniDP_ModeTarget &target = minidp.get_mode_target();
    if (!target.position_valid) {
        return;
    }

    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    uint16_t type_mask =
        POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE |
        POSITION_TARGET_TYPEMASK_LAST_BYTE;
    if (!target.yaw_valid) {
        type_mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    }

    mavlink_msg_position_target_local_ned_send(
        chan,
        AP_HAL::millis(),
        MAV_FRAME_LOCAL_NED,
        type_mask,
        target.pos_n_m,
        target.pos_e_m,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        target.yaw_valid ? target.yaw_rad : 0.0f,
        0.0f);
}

uint8_t GCS_MAVLINK_MiniDP::send_available_mode(const uint8_t index) const
{
    struct AvailableMode {
        MiniDP_Mode mode;
        const char *name;
    };
    const AvailableMode modes[] {
        {MiniDP_Mode::MANUAL, "MANUAL"},
        {MiniDP_Mode::HEADING_HOLD, "HEADING_HOLD"},
        {MiniDP_Mode::DP_HOLD, "DP_HOLD"},
    };
    const uint8_t mode_count = ARRAY_SIZE(modes);
    const uint8_t index_zero = index - 1U;
    if (index_zero >= mode_count) {
        return mode_count;
    }

    mavlink_msg_available_modes_send(
        chan,
        mode_count,
        index,
        MAV_STANDARD_MODE::MAV_STANDARD_MODE_NON_STANDARD,
        uint32_t(modes[index_zero].mode),
        0,
        modes[index_zero].name);
    return mode_count;
}

MAV_RESULT GCS_MAVLINK_MiniDP::handle_mav_cmd_do_set_mode(
    const mavlink_command_int_t &packet)
{
    const uint8_t base_mode = uint8_t(packet.param1);
    if ((base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) == 0) {
        send_minidp_text(
            MAV_SEVERITY_WARNING,
            "Mode rejected: custom mode required");
        return MAV_RESULT_DENIED;
    }

    MiniDP_Mode requested_mode = MiniDP_Mode::MANUAL;
    if (!mode_from_custom_mode(uint32_t(packet.param2), requested_mode)) {
        send_minidp_text(
            MAV_SEVERITY_WARNING,
            "Mode rejected: unsupported");
        return MAV_RESULT_DENIED;
    }

    const MiniDP_ModeCommandResult result =
        minidp.request_mode(requested_mode, MiniDP_ModeReason::MAVLINK_REQUEST);
    if (!result.accepted) {
        char text[50];
        if (result.authority_rejection != MiniDP_AuthorityReject::NONE) {
            AP_HAL::get_HAL().util->snprintf(
                text,
                sizeof(text),
                "Mode rejected: %s",
                MiniDP_AuthorityArbiter::reject_name(
                    result.authority_rejection));
        } else {
            AP_HAL::get_HAL().util->snprintf(
                text,
                sizeof(text),
                "Mode rejected: %s",
                MiniDP_ModeManager::reject_name(result.mode_rejection));
        }
        send_minidp_text(MAV_SEVERITY_WARNING, text);
        return MAV_RESULT_DENIED;
    }

    send_minidp_text(MAV_SEVERITY_INFO, "Mode accepted");
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_MiniDP::handle_mav_cmd_do_motor_test(
    const mavlink_command_int_t &packet)
{
    const int32_t motor_count = packet.x;
    if (motor_count > 1) {
        send_minidp_text(
            MAV_SEVERITY_WARNING,
            "Actuator test rejected: count");
        return MAV_RESULT_DENIED;
    }

    const int32_t test_order = packet.y;
    if (test_order != MOTOR_TEST_ORDER_DEFAULT &&
        test_order != MOTOR_TEST_ORDER_BOARD) {
        send_minidp_text(
            MAV_SEVERITY_WARNING,
            "Actuator test rejected: order");
        return MAV_RESULT_DENIED;
    }

    if (packet.param1 < 1.0f ||
        packet.param1 > float(MiniDP_OutputManager::max_actuators)) {
        send_minidp_text(
            MAV_SEVERITY_WARNING,
            "Actuator test rejected: actuator");
        return MAV_RESULT_DENIED;
    }

    uint8_t actuator_number = 0;
    if (!parse_uint8_param(
            packet.param1,
            1U,
            MiniDP_OutputManager::max_actuators,
            actuator_number)) {
        send_minidp_text(
            MAV_SEVERITY_WARNING,
            "Actuator test rejected: actuator");
        return MAV_RESULT_DENIED;
    }

    uint8_t throttle_type = 0;
    if (!parse_uint8_param(
            packet.param2,
            uint8_t(MiniDP_ActuatorTestThrottleType::PERCENT),
            uint8_t(MiniDP_ActuatorTestThrottleType::PWM),
            throttle_type)) {
        send_minidp_text(
            MAV_SEVERITY_WARNING,
            "Actuator test rejected: throttle");
        return MAV_RESULT_DENIED;
    }

    MiniDP_ActuatorTestRequest request{};
    request.actuator_index = actuator_number - 1U;
    request.throttle_type = throttle_type;
    request.throttle_value = packet.param3;
    request.timeout_s = packet.param4;

    const MiniDP_ActuatorTestStartResult result =
        minidp.request_actuator_test(request);
    if (!result.accepted) {
        char text[50];
        AP_HAL::get_HAL().util->snprintf(
            text,
            sizeof(text),
            "Actuator test rejected: %s",
            MiniDP_ActuatorTest::reject_name(result.rejection));
        send_minidp_text(MAV_SEVERITY_WARNING, text);
        return MAV_RESULT_DENIED;
    }

    send_minidp_text(MAV_SEVERITY_INFO, "Actuator test started");
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_MiniDP::handle_mav_cmd_component_arm_disarm(
    const mavlink_command_int_t &packet)
{
    const bool force = is_equal(packet.param2, 21196.0f);

    if (is_equal(packet.param1, 1.0f)) {
        const MiniDP_ArmResult result = minidp.request_arm(true, force);
        if (!result.accepted) {
            char text[50];
            AP_HAL::get_HAL().util->snprintf(
                text,
                sizeof(text),
                "Arm rejected: %s",
                MiniDP_Arming::reject_name(result.rejection));
            send_minidp_text(MAV_SEVERITY_WARNING, text);
            return MAV_RESULT_DENIED;
        }
        send_minidp_text(
            MAV_SEVERITY_INFO,
            force ? "Armed (force)" : "Armed");
        return MAV_RESULT_ACCEPTED;
    }

    if (is_zero(packet.param1)) {
        const MiniDP_ArmResult result = minidp.request_arm(false, force);
        if (!result.accepted) {
            return MAV_RESULT_DENIED;
        }
        send_minidp_text(MAV_SEVERITY_INFO, "Disarmed");
        return MAV_RESULT_ACCEPTED;
    }

    send_minidp_text(
        MAV_SEVERITY_WARNING,
        "Arm rejected: unsupported command");
    return MAV_RESULT_DENIED;
}

MAV_RESULT GCS_MAVLINK_MiniDP::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        return handle_mav_cmd_component_arm_disarm(packet);
    case MAV_CMD_DO_SET_MODE:
        return handle_mav_cmd_do_set_mode(packet);
    case MAV_CMD_DO_MOTOR_TEST:
        return handle_mav_cmd_do_motor_test(packet);
    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        return handle_preflight_reboot(packet, msg);
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

uint32_t GCS_MiniDP::custom_mode() const
{
    return static_cast<uint32_t>(minidp.get_mode());
}

#endif
