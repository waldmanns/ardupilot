#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#include <math.h>

struct MiniDP_State {
    uint64_t time_us;

    bool attitude_valid;
    bool yaw_valid;
    bool position_valid;
    bool velocity_valid;
    bool origin_valid;

    float yaw_rad;
    float yaw_rate_rad_s;

    float pos_n_m;
    float pos_e_m;
    float vel_n_m_s;
    float vel_e_m_s;

    float gps_hacc_m = NAN;
    float gps_vacc_m = NAN;
    float gps_sacc_m = NAN;

    uint8_t gps_fix_type;
    uint8_t gps_num_sats;

    bool compass_healthy;
    bool imu_healthy;
    bool ekf_healthy;

    uint32_t origin_id;
    uint32_t reset_counter;
};

class MiniDP_StateSource {
public:
    void update(
        AP_AHRS &ahrs,
        const AP_InertialSensor &ins,
        const Compass &compass,
        const AP_GPS &gps);

    const MiniDP_State &get() const { return state; }

private:
    MiniDP_State state{};
    Location last_origin{};
    bool have_origin = false;
    uint32_t origin_id = 0;
    uint32_t reset_counter = 0;
    uint32_t last_yaw_reset_ms = 0;
    uint32_t last_position_reset_ms = 0;
    uint32_t last_velocity_reset_ms = 0;

    void update_origin(AP_AHRS &ahrs, MiniDP_State &next);
    void update_reset_counter(AP_AHRS &ahrs, MiniDP_State &next);
};
