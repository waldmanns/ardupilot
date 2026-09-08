#include "State.h"

#include <AP_HAL/AP_HAL.h>

void MiniDP_StateSource::update_origin(AP_AHRS &ahrs, MiniDP_State &next)
{
    Location origin;
    next.origin_valid = ahrs.get_origin(origin);
    if (!next.origin_valid) {
        next.origin_id = origin_id;
        return;
    }

    if (!have_origin) {
        have_origin = true;
        last_origin = origin;
        origin_id = 1;
    } else if (!last_origin.same_loc_as(origin)) {
        last_origin = origin;
        origin_id++;
    }

    next.origin_id = origin_id;
}

void MiniDP_StateSource::update_reset_counter(AP_AHRS &ahrs, MiniDP_State &next)
{
    float yaw_delta;
    Vector2f position_delta;
    Vector2f velocity_delta;

    const uint32_t yaw_reset_ms = ahrs.getLastYawResetAngle(yaw_delta);
    const uint32_t position_reset_ms = ahrs.getLastPosNorthEastReset(position_delta);
    const uint32_t velocity_reset_ms = ahrs.getLastVelNorthEastReset(velocity_delta);

    const bool reset_changed =
        (yaw_reset_ms != 0 && yaw_reset_ms != last_yaw_reset_ms) ||
        (position_reset_ms != 0 && position_reset_ms != last_position_reset_ms) ||
        (velocity_reset_ms != 0 && velocity_reset_ms != last_velocity_reset_ms);

    last_yaw_reset_ms = yaw_reset_ms;
    last_position_reset_ms = position_reset_ms;
    last_velocity_reset_ms = velocity_reset_ms;

    if (reset_changed) {
        reset_counter++;
    }
    next.reset_counter = reset_counter;
}

void MiniDP_StateSource::update(
    AP_AHRS &ahrs,
    const AP_InertialSensor &ins,
    const Compass &compass,
    const AP_GPS &gps)
{
    MiniDP_State next{};
    next.time_us = AP_HAL::micros64();
    next.imu_healthy = ins.healthy();
    next.compass_healthy = compass.healthy();
    next.ekf_healthy = ahrs.healthy();
    next.gps_fix_type = static_cast<uint8_t>(gps.status());
    next.gps_num_sats = gps.num_sats();

    nav_filter_status filter_status{};
    const bool have_filter_status = ahrs.get_filter_status(filter_status);
    next.attitude_valid =
        have_filter_status &&
        filter_status.flags.attitude;
    next.yaw_valid = next.attitude_valid && next.ekf_healthy;

    if (next.yaw_valid) {
        next.yaw_rad = ahrs.get_yaw_rad();
        next.yaw_rate_rad_s = ahrs.get_yaw_rate_earth();
    }

    update_origin(ahrs, next);

    Vector3f position_ned;
    next.position_valid =
        next.origin_valid &&
        have_filter_status &&
        filter_status.flags.horiz_pos_abs &&
        ahrs.get_relative_position_NED_origin_float(position_ned);
    if (next.position_valid) {
        next.pos_n_m = position_ned.x;
        next.pos_e_m = position_ned.y;
    }

    Vector3f velocity_ned;
    next.velocity_valid =
        have_filter_status &&
        filter_status.flags.horiz_vel &&
        ahrs.get_velocity_NED(velocity_ned);
    if (next.velocity_valid) {
        next.vel_n_m_s = velocity_ned.x;
        next.vel_e_m_s = velocity_ned.y;
    }

    if (gps.is_healthy() && gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        (void)gps.horizontal_accuracy(next.gps_hacc_m);
        (void)gps.vertical_accuracy(next.gps_vacc_m);
        (void)gps.speed_accuracy(next.gps_sacc_m);
    }

    next.yaw_valid = next.yaw_valid && isfinite(next.yaw_rad) && isfinite(next.yaw_rate_rad_s);
    next.position_valid = next.position_valid && isfinite(next.pos_n_m) && isfinite(next.pos_e_m);
    next.velocity_valid = next.velocity_valid && isfinite(next.vel_n_m_s) && isfinite(next.vel_e_m_s);
    update_reset_counter(ahrs, next);
    state = next;
}
