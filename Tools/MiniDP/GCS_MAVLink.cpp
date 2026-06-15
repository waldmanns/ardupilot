#include "GCS_MAVLink.h"
#include "MiniDP.h"

#if HAL_GCS_ENABLED

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
    default:
        return GCS_MAVLINK::try_send_message(id);
    }
}

uint32_t GCS_MiniDP::custom_mode() const
{
    return static_cast<uint32_t>(minidp.get_mode());
}

#endif
