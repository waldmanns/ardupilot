#include "GCS_MAVLink.h"
#include "MiniDP.h"
#include "ControlMath.h"

#include <AP_Common/AP_FWVersion.h>
#include <AP_Math/AP_Math.h>

#include <limits.h>

#if HAL_GCS_ENABLED

extern const AP_HAL::HAL &hal;

namespace {

constexpr uint16_t set_pos_type_mask_pos_ignore = (1U << 0) | (1U << 1);
constexpr uint16_t set_pos_type_mask_acc_ignore = (7U << 6);
constexpr uint16_t set_pos_type_mask_yaw_ignore = 1U << 10;

bool parse_uint8_param(
    const float value,
    const uint8_t min_value,
    const uint8_t max_value,
    uint8_t &parsed)
{
    if (!isfinite(value) || value < float(min_value) || value > float(max_value)) {
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

int16_t radio_rc_channel_to_pwm(const int16_t channel)
{
    // RADIO_RC_CHANNELS uses centered 13-bit values: -4096..4096 around zero.
    return int16_t(((int32_t(channel) * 5) / 32) + 1500);
}

uint8_t rc_override_count(const mavlink_rc_channels_override_t &packet)
{
    const uint16_t override_data[] = {
        packet.chan1_raw,
        packet.chan2_raw,
        packet.chan3_raw,
        packet.chan4_raw,
        packet.chan5_raw,
        packet.chan6_raw,
        packet.chan7_raw,
        packet.chan8_raw,
        packet.chan9_raw,
        packet.chan10_raw,
        packet.chan11_raw,
        packet.chan12_raw,
        packet.chan13_raw,
        packet.chan14_raw,
        packet.chan15_raw,
        packet.chan16_raw
    };

    uint8_t count = 0;
    for (uint8_t i = 0; i < ARRAY_SIZE(override_data); i++) {
        const uint16_t value = override_data[i];
        if (value != 0 &&
            value != UINT16_MAX &&
            value != uint16_t(UINT16_MAX - 1)) {
            count = i + 1;
        }
    }
    return count;
}

bool supported_local_position_frame(const uint8_t frame)
{
    switch (frame) {
    case MAV_FRAME_LOCAL_NED:
    case MAV_FRAME_LOCAL_OFFSET_NED:
    case MAV_FRAME_BODY_NED:
    case MAV_FRAME_BODY_OFFSET_NED:
        return true;
    default:
        return false;
    }
}

bool supported_global_position_frame(const uint8_t frame)
{
    switch (frame) {
    case MAV_FRAME_GLOBAL:
    case MAV_FRAME_GLOBAL_INT:
    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
        return true;
    default:
        return false;
    }
}

bool body_position_frame(const uint8_t frame)
{
    return frame == MAV_FRAME_BODY_OFFSET_NED;
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

void GCS_MiniDP::send_minidp_thruster_status()
{
    for (uint8_t i = 0; i < num_gcs(); i++) {
        auto *backend = static_cast<GCS_MAVLINK_MiniDP *>(chan(i));
        if (backend != nullptr) {
            backend->send_minidp_thruster_status();
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
    if (minidp.get_mode() == MiniDP_Mode::HEADING_HOLD ||
        minidp.get_mode() == MiniDP_Mode::DP_HOLD) {
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
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

void GCS_MAVLINK_MiniDP::handle_message(const mavlink_message_t &msg)
{
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
    if (msg.msgid == MAVLINK_MSG_ID_RADIO_RC_CHANNELS) {
        mavlink_radio_rc_channels_t packet;
        mavlink_msg_radio_rc_channels_decode(&msg, &packet);
        minidp.record_mavlink_radio_rc_channels(
            msg.sysid,
            msg.compid,
            packet.count,
            packet.flags,
            packet.count > 0 ? radio_rc_channel_to_pwm(packet.channels[0]) : 0);
    }
#endif

#if AP_RC_CHANNEL_ENABLED
    if (msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
        mavlink_rc_channels_override_t packet;
        mavlink_msg_rc_channels_override_decode(&msg, &packet);
        minidp.record_mavlink_rc_override(
            msg.sysid,
            msg.compid,
            rc_override_count(packet),
            packet.chan1_raw);
    }
#endif

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_SET_MODE: {
        mavlink_set_mode_t packet;
        mavlink_msg_set_mode_decode(&msg, &packet);
        if ((packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) != 0) {
            MiniDP_Mode mode;
            if (mode_from_custom_mode(packet.custom_mode, mode)) {
                if (minidp.request_mode(mode, MiniDP_ModeReason::MAVLINK_REQUEST).accepted) {
                    sysid_mygcs_seen(AP_HAL::millis());
                }
            }
            return;
        }
        // The shared path handles hardware safety switch commands.
        break;
    }
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        handle_set_position_target_local_ned(msg);
        return;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        handle_set_position_target_global_int(msg);
        return;

    default:
        break;
    }

    GCS_MAVLINK::handle_message(msg);
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

bool GCS_MAVLINK_MiniDP::send_minidp_thruster_status()
{
    const MiniDP_OutputFrame &frame = minidp.get_output_frame();

    for (uint8_t i = 0; i < MiniDP_OutputManager::max_thrusters; i++) {
        const uint8_t thruster_index =
            (thruster_status_next + i) % MiniDP_OutputManager::max_thrusters;
        const MiniDP_ThrusterOutput &thruster = frame.thruster[thruster_index];
        if (!thruster.configured) {
            continue;
        }

        thruster_status_next =
            (thruster_index + 1U) % MiniDP_OutputManager::max_thrusters;

        CHECK_PAYLOAD_SIZE(MINIDP_THRUSTER_STATUS);

        mavlink_minidp_thruster_status_t packet {};
        packet.time_boot_ms = AP_HAL::millis();
        packet.thruster_id = thruster_index + 1U;
        packet.thruster_type = uint8_t(thruster.type);
        packet.output_state = uint8_t(frame.state);
        packet.flags = thruster.flags;
        packet.force_x = thruster.force_x;
        packet.force_y = thruster.force_y;
        packet.value = thruster.value;
        packet.value2 = thruster.value2;
        packet.motor1 = thruster.output1 < MiniDP_OutputManager::max_actuators ?
            thruster.output1 + 1U :
            0U;
        packet.motor2 = thruster.output2 < MiniDP_OutputManager::max_actuators ?
            thruster.output2 + 1U :
            0U;
        packet.pwm1 = thruster.pwm1_us;
        packet.pwm2 = thruster.pwm2_us;

        mavlink_msg_minidp_thruster_status_send_struct(chan, &packet);
        return true;
    }

    return true;
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

uint64_t GCS_MAVLINK_MiniDP::capabilities() const
{
    return MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
           MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
           MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
           GCS_MAVLINK::capabilities();
}

MAV_RESULT GCS_MAVLINK_MiniDP::request_dp_target_ne(
    const float pos_n_m,
    const float pos_e_m,
    const bool yaw_valid,
    const float yaw_rad,
    const char *accepted_text,
    const char *rejected_prefix)
{
    const MiniDP_ModeCommandResult result =
        minidp.request_dp_target(
            pos_n_m,
            pos_e_m,
            yaw_valid,
            yaw_rad,
            MiniDP_ModeReason::MAVLINK_REQUEST);

    if (result.accepted) {
        sysid_mygcs_seen(AP_HAL::millis());
        if (accepted_text != nullptr) {
            send_minidp_text(MAV_SEVERITY_INFO, accepted_text);
        }
        return MAV_RESULT_ACCEPTED;
    }

    if (rejected_prefix != nullptr) {
        char text[50];
        if (result.authority_rejection != MiniDP_AuthorityReject::NONE) {
            AP_HAL::get_HAL().util->snprintf(
                text,
                sizeof(text),
                "%s: %s",
                rejected_prefix,
                MiniDP_AuthorityArbiter::reject_name(
                    result.authority_rejection));
        } else {
            AP_HAL::get_HAL().util->snprintf(
                text,
                sizeof(text),
                "%s: %s",
                rejected_prefix,
                MiniDP_ModeManager::reject_name(result.mode_rejection));
        }
        send_minidp_text(MAV_SEVERITY_WARNING, text);
    }

    return MAV_RESULT_DENIED;
}

MAV_RESULT GCS_MAVLINK_MiniDP::request_dp_target_location(
    const Location &location,
    const bool yaw_valid,
    const float yaw_rad,
    const char *accepted_text,
    const char *rejected_prefix)
{
    if (!location.check_latlng()) {
        if (rejected_prefix != nullptr) {
            send_minidp_text(MAV_SEVERITY_WARNING, "DP target rejected: lat/lon");
        }
        return MAV_RESULT_DENIED;
    }

    Location origin;
    if (!minidp.ahrs.get_origin(origin)) {
        if (rejected_prefix != nullptr) {
            send_minidp_text(MAV_SEVERITY_WARNING, "DP target rejected: origin");
        }
        return MAV_RESULT_DENIED;
    }

    const Vector2f target_ne = origin.get_distance_NE(location);
    return request_dp_target_ne(
        target_ne.x,
        target_ne.y,
        yaw_valid,
        yaw_rad,
        accepted_text,
        rejected_prefix);
}

bool GCS_MAVLINK_MiniDP::current_or_target_position(
    float &pos_n_m,
    float &pos_e_m) const
{
    const MiniDP_State &state = minidp.get_state();
    const MiniDP_ModeTarget &target = minidp.get_mode_target();
    if (target.position_valid &&
        target.origin_id == state.origin_id &&
        target.reset_counter == state.reset_counter) {
        pos_n_m = target.pos_n_m;
        pos_e_m = target.pos_e_m;
        return true;
    }

    if (state.position_valid) {
        pos_n_m = state.pos_n_m;
        pos_e_m = state.pos_e_m;
        return true;
    }

    return false;
}

bool GCS_MAVLINK_MiniDP::handle_guided_request(
    AP_Mission::Mission_Command &cmd)
{
    return request_dp_target_location(
        cmd.content.location,
        false,
        0.0f,
        "DP target accepted",
        "DP target rejected") == MAV_RESULT_ACCEPTED;
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

void GCS_MAVLINK_MiniDP::handle_set_position_target_local_ned(
    const mavlink_message_t &msg)
{
    mavlink_set_position_target_local_ned_t packet;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

    if (!supported_local_position_frame(packet.coordinate_frame)) {
        return;
    }

    const uint16_t pos_mask = packet.type_mask & set_pos_type_mask_pos_ignore;
    if (pos_mask != 0 && pos_mask != set_pos_type_mask_pos_ignore) {
        return;
    }
    const bool pos_ignore = pos_mask == set_pos_type_mask_pos_ignore;
    const bool yaw_ignore =
        (packet.type_mask & set_pos_type_mask_yaw_ignore) != 0;
    const bool acc_ignore =
        (packet.type_mask & set_pos_type_mask_acc_ignore) ==
        set_pos_type_mask_acc_ignore;
    constexpr uint16_t unsupported_mask = (7U << 3) | (1U << 11);
    if (!acc_ignore || (pos_ignore && yaw_ignore) ||
        (packet.type_mask & unsupported_mask) != unsupported_mask ||
        (!yaw_ignore && !isfinite(packet.yaw))) {
        return;
    }

    if (!pos_ignore && (!isfinite(packet.x) || !isfinite(packet.y) ||
        fabsf(packet.x) > 2.0e7f || fabsf(packet.y) > 2.0e7f)) {
        return;
    }
    const MiniDP_State &state = minidp.get_state();
    float pos_n_m = 0.0f;
    float pos_e_m = 0.0f;

    if (pos_ignore) {
        if (!current_or_target_position(pos_n_m, pos_e_m)) {
            return;
        }
    } else {
        switch (packet.coordinate_frame) {
        case MAV_FRAME_LOCAL_NED:
        case MAV_FRAME_BODY_NED:
            pos_n_m = packet.x;
            pos_e_m = packet.y;
            break;

        case MAV_FRAME_LOCAL_OFFSET_NED:
            if (!state.position_valid) {
                return;
            }
            pos_n_m = state.pos_n_m + packet.x;
            pos_e_m = state.pos_e_m + packet.y;
            break;

        case MAV_FRAME_BODY_OFFSET_NED:
            if (!state.position_valid || !state.yaw_valid) {
                return;
            }
            pos_n_m =
                state.pos_n_m +
                (cosf(state.yaw_rad) * packet.x) -
                (sinf(state.yaw_rad) * packet.y);
            pos_e_m =
                state.pos_e_m +
                (sinf(state.yaw_rad) * packet.x) +
                (cosf(state.yaw_rad) * packet.y);
            break;

        default:
            return;
        }
    }

    bool yaw_valid = false;
    float yaw_rad = 0.0f;
    if (!yaw_ignore) {
        yaw_valid = true;
        yaw_rad = packet.yaw;
        if (body_position_frame(packet.coordinate_frame) && state.yaw_valid) {
            yaw_rad = MiniDP_Math::wrap_pi(state.yaw_rad + MiniDP_Math::wrap_pi(yaw_rad));
        }
    }

    (void)request_dp_target_ne(
        pos_n_m,
        pos_e_m,
        yaw_valid,
        yaw_rad,
        nullptr,
        nullptr);
}

void GCS_MAVLINK_MiniDP::handle_set_position_target_global_int(
    const mavlink_message_t &msg)
{
    mavlink_set_position_target_global_int_t packet;
    mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

    if (!supported_global_position_frame(packet.coordinate_frame)) {
        return;
    }

    const uint16_t pos_mask = packet.type_mask & set_pos_type_mask_pos_ignore;
    if (pos_mask != 0 && pos_mask != set_pos_type_mask_pos_ignore) {
        return;
    }
    const bool pos_ignore = pos_mask == set_pos_type_mask_pos_ignore;
    const bool yaw_ignore =
        (packet.type_mask & set_pos_type_mask_yaw_ignore) != 0;
    const bool acc_ignore =
        (packet.type_mask & set_pos_type_mask_acc_ignore) ==
        set_pos_type_mask_acc_ignore;
    constexpr uint16_t unsupported_mask = (7U << 3) | (1U << 11);
    if (!acc_ignore || (pos_ignore && yaw_ignore) ||
        (packet.type_mask & unsupported_mask) != unsupported_mask ||
        (!yaw_ignore && !isfinite(packet.yaw))) {
        return;
    }

    bool yaw_valid = false;
    float yaw_rad = 0.0f;
    if (!yaw_ignore) {
        yaw_valid = true;
        yaw_rad = packet.yaw;
    }

    if (pos_ignore) {
        float pos_n_m = 0.0f;
        float pos_e_m = 0.0f;
        if (!current_or_target_position(pos_n_m, pos_e_m)) {
            return;
        }
        (void)request_dp_target_ne(
            pos_n_m,
            pos_e_m,
            yaw_valid,
            yaw_rad,
            nullptr,
            nullptr);
        return;
    }

    if (!check_latlng(packet.lat_int, packet.lon_int)) {
        return;
    }

    Location target_location{};
    target_location.lat = packet.lat_int;
    target_location.lng = packet.lon_int;
    (void)request_dp_target_location(
        target_location,
        yaw_valid,
        yaw_rad,
        nullptr,
        nullptr);
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
    uint8_t base_mode;
    uint8_t requested_number;
    if (!parse_uint8_param(packet.param1, 0, UINT8_MAX, base_mode) ||
        !parse_uint8_param(packet.param2, 0, 2, requested_number)) {
        return MAV_RESULT_DENIED;
    }
    if ((base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) == 0) {
        send_minidp_text(
            MAV_SEVERITY_WARNING,
            "Mode rejected: custom mode required");
        return MAV_RESULT_DENIED;
    }

    MiniDP_Mode requested_mode = MiniDP_Mode::MANUAL;
    if (!mode_from_custom_mode(requested_number, requested_mode)) {
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

    sysid_mygcs_seen(AP_HAL::millis());
    send_minidp_text(MAV_SEVERITY_INFO, "Mode accepted");
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_MiniDP::handle_mav_cmd_do_motor_test(
    const mavlink_command_int_t &packet)
{
    const int32_t motor_count = packet.x;
    if (motor_count < 0 || motor_count > 1) {
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

    if (!isfinite(packet.param1) || packet.param1 < 1.0f ||
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

MAV_RESULT GCS_MAVLINK_MiniDP::handle_mav_cmd_do_reposition(
    const mavlink_command_int_t &packet)
{
    if (!check_latlng(packet.x, packet.y) ||
        (packet.x == 0 && packet.y == 0)) {
        send_minidp_text(MAV_SEVERITY_WARNING, "DP target rejected: lat/lon");
        return MAV_RESULT_DENIED;
    }

    Location requested_location{};
    if (!location_from_command_t(packet, requested_location) ||
        !requested_location.check_latlng()) {
        send_minidp_text(MAV_SEVERITY_WARNING, "DP target rejected: location");
        return MAV_RESULT_DENIED;
    }

    return request_dp_target_location(
        requested_location,
        false,
        0.0f,
        "DP target accepted",
        "DP target rejected");
}

MAV_RESULT GCS_MAVLINK_MiniDP::handle_mav_cmd_component_arm_disarm(
    const mavlink_command_int_t &packet)
{
    if (!isfinite(packet.param1) || !isfinite(packet.param2)) {
        return MAV_RESULT_DENIED;
    }
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

MAV_RESULT GCS_MAVLINK_MiniDP::handle_preflight_reboot(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    if (!isfinite(packet.param1)) {
        return MAV_RESULT_DENIED;
    }
    return GCS_MAVLINK::handle_preflight_reboot(packet, msg);
}

MAV_RESULT GCS_MAVLINK_MiniDP::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_RUN_PREARM_CHECKS: {
        if (minidp.is_armed() || hal.util->get_soft_armed()) {
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
        const auto result = minidp.check_arm();
        if (!result.accepted) {
            send_text(MAV_SEVERITY_WARNING, "PreArm: %s",
                MiniDP_Arming::reject_name(result.rejection));
        }
        return result.accepted ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
    }
    case MAV_CMD_COMPONENT_ARM_DISARM:
        return handle_mav_cmd_component_arm_disarm(packet);
    case MAV_CMD_DO_SET_MODE:
        return handle_mav_cmd_do_set_mode(packet);
    case MAV_CMD_DO_REPOSITION:
        return handle_mav_cmd_do_reposition(packet);
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
