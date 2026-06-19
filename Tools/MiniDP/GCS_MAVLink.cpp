#include "GCS_MAVLink.h"
#include "MiniDP.h"

#include <AP_Common/AP_FWVersion.h>

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
    if (minidp.get_mode() == MiniDP_Mode::MANUAL) {
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
    return base_mode;
}

MAV_STATE GCS_MAVLINK_MiniDP::vehicle_system_status() const
{
    return minidp.get_mode() == MiniDP_Mode::FAILSAFE ?
        MAV_STATE_CRITICAL :
        MAV_STATE_STANDBY;
}

void GCS_MAVLINK_MiniDP::send_minidp_sys_status() const
{
    const MiniDP_State &state = minidp.get_state();

    uint32_t present =
        MAV_SYS_STATUS_SENSOR_3D_GYRO |
        MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        MAV_SYS_STATUS_SENSOR_3D_MAG |
        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
        MAV_SYS_STATUS_SENSOR_GPS |
        MAV_SYS_STATUS_AHRS;
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

    mavlink_msg_sys_status_send(
        chan,
        present,
        enabled,
        healthy,
        0,
        UINT16_MAX,
        -1,
        -1,
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
        return true;
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

MAV_RESULT GCS_MAVLINK_MiniDP::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_DO_SET_MODE:
        return handle_mav_cmd_do_set_mode(packet);
    case MAV_CMD_DO_MOTOR_TEST:
        return handle_mav_cmd_do_motor_test(packet);
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

uint32_t GCS_MiniDP::custom_mode() const
{
    return static_cast<uint32_t>(minidp.get_mode());
}

#endif
