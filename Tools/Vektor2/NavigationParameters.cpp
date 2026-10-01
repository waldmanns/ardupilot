#include "NavigationParameters.h"

namespace Vektor2 {

const AP_Param::GroupInfo NavigationParameters::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Vektor navigation system
    // @Description: Enable Vektor navigation system
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, NavigationParameters, enable, 0),

    // @Param: MODE
    // @DisplayName: Current navigation mode
    // @Description: Current navigation mode
    // @Values: 0:Manual,1:Heading Hold,2:Dynamic Positioning
    // @User: Standard
    AP_GROUPINFO("MODE", 2, NavigationParameters, mode, 0),

    // @Param: HDG_ENABLE
    // @DisplayName: Enable heading hold mode
    // @Description: Enable heading hold mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("HDG_ENABLE", 3, NavigationParameters, hdg_enable, 0),

    // @Param: HDG_CAPTURE
    // @DisplayName: Capture current heading on activation
    // @Description: Capture current heading on activation
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("HDG_CAPTURE", 4, NavigationParameters, hdg_capture, 0),

    // @Param: HDG_KP
    // @DisplayName: Heading proportional gain
    // @Description: Heading proportional gain
    // @User: Standard
    AP_GROUPINFO("HDG_KP", 5, NavigationParameters, hdg_kp, 0),

    // @Param: HDG_KD
    // @DisplayName: Heading damping gain from yaw rate
    // @Description: Heading damping gain from yaw rate
    // @User: Standard
    AP_GROUPINFO("HDG_KD", 6, NavigationParameters, hdg_kd, 0),

    // @Param: HDG_KI
    // @DisplayName: Heading integral correction gain
    // @Description: Heading integral correction gain
    // @User: Standard
    AP_GROUPINFO("HDG_KI", 7, NavigationParameters, hdg_ki, 0),

    // @Param: HDG_DBAND
    // @DisplayName: Heading error ignored before correction (deg)
    // @Description: Heading error ignored before correction (deg)
    // @User: Standard
    AP_GROUPINFO("HDG_DBAND", 8, NavigationParameters, hdg_deadband, 0),

    // @Param: HDG_MAX_YAW
    // @DisplayName: Maximum allowed yaw control output (%)
    // @Description: Maximum allowed yaw control output (%)
    // @User: Standard
    AP_GROUPINFO("HDG_MAX_YAW", 9, NavigationParameters, hdg_max_yaw, 100),

    // @Param: HDG_STK_RT
    // @DisplayName: Heading target change rate from steering input (deg/s)
    // @Description: Heading target change rate from steering input (deg/s)
    // @User: Standard
    AP_GROUPINFO("HDG_STK_RT", 10, NavigationParameters, hdg_stick_rate, 0),

    // @Param: DP_ENABLE
    // @DisplayName: Enable dynamic positioning
    // @Description: Enable dynamic positioning
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DP_ENABLE", 11, NavigationParameters, dp_enable, 0),

    // @Param: DP_CAP_POS
    // @DisplayName: Save current GPS position as DP target
    // @Description: Save current GPS position as DP target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DP_CAP_POS", 12, NavigationParameters, dp_capture_pos, 0),

    // @Param: DP_CAP_HDG
    // @DisplayName: Save current heading as DP heading target
    // @Description: Save current heading as DP heading target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DP_CAP_HDG", 13, NavigationParameters, dp_capture_hdg, 0),

    // @Param: DP_RADIUS
    // @DisplayName: Position tolerance before considered stable (m)
    // @Description: Position tolerance before considered stable (m)
    // @User: Standard
    AP_GROUPINFO("DP_RADIUS", 14, NavigationParameters, dp_radius, 0),

    // @Param: DP_POS_KP
    // @DisplayName: Position error to velocity command gain
    // @Description: Position error to velocity command gain
    // @User: Standard
    AP_GROUPINFO("DP_POS_KP", 15, NavigationParameters, dp_pos_kp, 0),

    // @Param: DP_MAX_SPD
    // @DisplayName: Maximum DP correction speed (m/s)
    // @Description: Maximum DP correction speed (m/s)
    // @User: Standard
    AP_GROUPINFO("DP_MAX_SPD", 16, NavigationParameters, dp_max_speed, 0),

    // @Param: DP_ACC_LIM
    // @DisplayName: Maximum acceleration command (m/s/s)
    // @Description: Maximum acceleration command (m/s/s)
    // @User: Standard
    AP_GROUPINFO("DP_ACC_LIM", 17, NavigationParameters, dp_accel_limit, 0),

    // @Param: DP_VEL_KP
    // @DisplayName: Velocity tracking gain
    // @Description: Velocity tracking gain
    // @User: Standard
    AP_GROUPINFO("DP_VEL_KP", 18, NavigationParameters, dp_vel_kp, 0),

    // @Param: DP_VEL_KI
    // @DisplayName: Velocity integral correction gain
    // @Description: Velocity integral correction gain
    // @User: Standard
    AP_GROUPINFO("DP_VEL_KI", 19, NavigationParameters, dp_vel_ki, 0),

    // @Param: DP_POS_FILT
    // @DisplayName: GPS position low-pass filter frequency (Hz)
    // @Description: GPS position low-pass filter frequency (Hz)
    // @User: Standard
    AP_GROUPINFO("DP_POS_FILT", 20, NavigationParameters, dp_pos_filter, 0),

    // @Param: DP_VEL_FILT
    // @DisplayName: Velocity estimate filter frequency (Hz)
    // @Description: Velocity estimate filter frequency (Hz)
    // @User: Standard
    AP_GROUPINFO("DP_VEL_FILT", 21, NavigationParameters, dp_vel_filter, 0),

    // @Param: DP_YAW_PRI
    // @DisplayName: Priority of heading vs translation (0-100%)
    // @Description: Priority of heading vs translation (0-100%)
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("DP_YAW_PRI", 22, NavigationParameters, dp_yaw_priority, 100),

    // @Param: AL_W_SURGE
    // @DisplayName: Surge control importance weighting
    // @Description: Surge control importance weighting
    // @User: Standard
    AP_GROUPINFO("AL_W_SURGE", 23, NavigationParameters, alloc_w_surge, 1),

    // @Param: AL_W_SWAY
    // @DisplayName: Sway control importance weighting
    // @Description: Sway control importance weighting
    // @User: Standard
    AP_GROUPINFO("AL_W_SWAY", 24, NavigationParameters, alloc_w_sway, 1),

    // @Param: ALLOC_W_YAW
    // @DisplayName: Yaw control importance weighting
    // @Description: Yaw control importance weighting
    // @User: Standard
    AP_GROUPINFO("ALLOC_W_YAW", 25, NavigationParameters, alloc_w_yaw, 1),

    // @Param: AL_MAX_SRG
    // @DisplayName: Maximum requested forward force
    // @Description: Maximum requested forward force
    // @User: Standard
    AP_GROUPINFO("AL_MAX_SRG", 26, NavigationParameters, alloc_max_surge, 0),

    // @Param: AL_MAX_SWAY
    // @DisplayName: Maximum requested sideways force
    // @Description: Maximum requested sideways force
    // @User: Standard
    AP_GROUPINFO("AL_MAX_SWAY", 27, NavigationParameters, alloc_max_sway, 0),

    // @Param: AL_MAX_YAW
    // @DisplayName: Maximum requested yaw moment
    // @Description: Maximum requested yaw moment
    // @User: Standard
    AP_GROUPINFO("AL_MAX_YAW", 28, NavigationParameters, alloc_max_yaw, 0),

    // @Param: AL_SURGE_SL
    // @DisplayName: Maximum surge command change rate
    // @Description: Maximum surge command change rate
    // @User: Standard
    AP_GROUPINFO("AL_SURGE_SL", 29, NavigationParameters, alloc_surge_slew, 0),

    // @Param: AL_SWAY_SL
    // @DisplayName: Maximum sway command change rate
    // @Description: Maximum sway command change rate
    // @User: Standard
    AP_GROUPINFO("AL_SWAY_SL", 30, NavigationParameters, alloc_sway_slew, 0),

    // @Param: AL_YAW_SL
    // @DisplayName: Maximum yaw command change rate
    // @Description: Maximum yaw command change rate
    // @User: Standard
    AP_GROUPINFO("AL_YAW_SL", 31, NavigationParameters, alloc_yaw_slew, 0),

    // @Param: MAN_OVR
    // @DisplayName: Allow pilot control to override autonomy
    // @Description: Allow pilot control to override autonomy
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("MAN_OVR", 32, NavigationParameters, manual_override, 1),

    // @Param: MAN_DBAND
    // @DisplayName: Pilot input deadband (0-1)
    // @Description: Pilot input deadband (0-1)
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("MAN_DBAND", 33, NavigationParameters, manual_deadband, 0),

    // @Param: OVR_MODE
    // @DisplayName: Override behaviour
    // @Description: Override behaviour
    // @Values: 0:Full,1:Blend,2:Nudge
    // @User: Standard
    AP_GROUPINFO("OVR_MODE", 34, NavigationParameters, override_mode, 0),

    // @Param: GPS_TIMEOUT
    // @DisplayName: Time without GPS before leaving DP (s)
    // @Description: Time without GPS before leaving DP (s)
    // @User: Standard
    AP_GROUPINFO("GPS_TIMEOUT", 35, NavigationParameters, gps_timeout, 0),

    // @Param: HDG_TIMEOUT
    // @DisplayName: Time without valid heading before leaving HH (s)
    // @Description: Time without valid heading before leaving HH (s)
    // @User: Standard
    AP_GROUPINFO("HDG_TIMEOUT", 36, NavigationParameters, hdg_timeout, 0),

    // @Param: FAIL_MODE
    // @DisplayName: Action after navigation failure
    // @Description: Action after navigation failure
    // @User: Standard
    AP_GROUPINFO("FAIL_MODE", 37, NavigationParameters, failsafe_mode, 0),

    // @Param: LOG_ENABLE
    // @DisplayName: Enable navigation debug logging
    // @Description: Enable navigation debug logging
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LOG_ENABLE", 38, NavigationParameters, log_enable, 0),

    // @Param: LOG_RATE
    // @DisplayName: Navigation logging frequency (Hz)
    // @Description: Navigation logging frequency (Hz)
    // @User: Standard
    AP_GROUPINFO("LOG_RATE", 39, NavigationParameters, log_rate, 0),

    AP_GROUPEND
};

} // namespace Vektor2
