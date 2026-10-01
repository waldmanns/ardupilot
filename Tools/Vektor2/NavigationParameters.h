#pragma once

#include <AP_Param/AP_Param.h>

namespace Vektor2 {

// Parameter storage for future navigation controllers; no control behavior is
// enabled by registering these values.
class NavigationParameters {
public:
    NavigationParameters() { AP_Param::setup_object_defaults(this, var_info); }
    static const AP_Param::GroupInfo var_info[];

    AP_Int8 enable;
    AP_Int8 mode;
    AP_Int8 hdg_enable;
    AP_Int8 hdg_capture;
    AP_Float hdg_kp;
    AP_Float hdg_kd;
    AP_Float hdg_ki;
    AP_Float hdg_deadband;
    AP_Float hdg_max_yaw;
    AP_Float hdg_stick_rate;
    AP_Int8 dp_enable;
    AP_Int8 dp_capture_pos;
    AP_Int8 dp_capture_hdg;
    AP_Float dp_radius;
    AP_Float dp_pos_kp;
    AP_Float dp_max_speed;
    AP_Float dp_accel_limit;
    AP_Float dp_vel_kp;
    AP_Float dp_vel_ki;
    AP_Float dp_pos_filter;
    AP_Float dp_vel_filter;
    AP_Float dp_yaw_priority;
    AP_Float alloc_w_surge;
    AP_Float alloc_w_sway;
    AP_Float alloc_w_yaw;
    AP_Float alloc_max_surge;
    AP_Float alloc_max_sway;
    AP_Float alloc_max_yaw;
    AP_Float alloc_surge_slew;
    AP_Float alloc_sway_slew;
    AP_Float alloc_yaw_slew;
    AP_Int8 manual_override;
    AP_Float manual_deadband;
    AP_Int8 override_mode;
    AP_Float gps_timeout;
    AP_Float hdg_timeout;
    AP_Int8 failsafe_mode;
    AP_Int8 log_enable;
    AP_Int16 log_rate;
};

} // namespace Vektor2
