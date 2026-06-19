#include "MiniDP.h"

const AP_Param::Info MiniDP::var_info[] = {
    GSCALAR(format_version, "FORMAT_VERSION", 0),
    GSCALAR(log_bitmask, "LOG_BITMASK", -1),

    GOBJECT(board_config, "BRD_", AP_BoardConfig),
    GOBJECT(serial_manager, "SERIAL", AP_SerialManager),
    GOBJECT(ins, "INS", AP_InertialSensor),
    GOBJECT(compass, "COMPASS_", Compass),
    GOBJECT(gps, "GPS", AP_GPS),
    GOBJECT(logger, "LOG", AP_Logger),
    GOBJECT(notify, "NTF_", AP_Notify),
    GOBJECT(rc_channels, "RC", RC_Channels),
    GOBJECT(servo_channels, "SERVO", SRV_Channels),
#if HAL_GCS_ENABLED
    GOBJECT(gcs_backend, "MAV", GCS),
#endif
#if MINIDP_BARO_ENABLED
    GOBJECT(barometer, "BARO", AP_Baro),
#endif
    GOBJECT(ahrs, "AHRS_", AP_AHRS),
#if HAL_NAVEKF2_AVAILABLE
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif
#if HAL_NAVEKF3_AVAILABLE
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif
    GSCALAR(auth_mav_manual, "AUTH_MAV_MAN", 0),
    GSCALAR(auth_mav_target, "AUTH_MAV_TGT", 0),
    GSCALAR(auth_test, "AUTH_TEST", 0),
    GSCALAR(auth_rc_takeover, "AUTH_RC_TAKE", 0),
    GSCALAR(auth_rc_fallback, "AUTH_RC_FB", 1),
    GSCALAR(auth_mav_fallback, "AUTH_MAV_FB", 0),
    GSCALAR(auth_target_loss, "AUTH_TGT_LOSS", 0),
    GSCALAR(auth_rc_timeout, "AUTH_RC_TMO", 0.5f),
    GSCALAR(auth_mav_timeout, "AUTH_MAV_TMO", 3.0f),
    GSCALAR(frame_type, "FRAME_TYPE", MiniDP_OutputManager::default_frame_type),
    GSCALAR(in_rc_surge_channel, "IN_RC_SURGE", 2),
    GSCALAR(in_rc_sway_channel, "IN_RC_SWAY", 1),
    GSCALAR(in_rc_yaw_channel, "IN_RC_YAW", 4),
    GSCALAR(in_rc_kill_channel, "IN_RC_KILL", 0),
    GSCALAR(in_rc_kill_pwm, "IN_KILL_PWM", 1800),
    GSCALAR(manual_enable, "MAN_ENABLE", 0),
    GSCALAR(manual_deadband, "MAN_DZ", 0.03f),
    GSCALAR(manual_surge_limit, "MAN_SRG_LIM", 0.5f),
    GSCALAR(manual_sway_limit, "MAN_SWY_LIM", 0.5f),
    GSCALAR(manual_yaw_limit, "MAN_YAW_LIM", 0.5f),
    GSCALAR(manual_mavlink_timeout, "MAN_MAV_TMO", 0.5f),
    GSCALAR(axis_deadband, "AXIS_DZ", 0.0f),
    GSCALAR(axis_surge_limit, "AXIS_SRG_MAX", 1.0f),
    GSCALAR(axis_sway_limit, "AXIS_SWY_MAX", 1.0f),
    GSCALAR(axis_yaw_limit, "AXIS_YAW_MAX", 1.0f),
    GSCALAR(axis_surge_slew, "AXIS_SRG_SLW", 0.0f),
    GSCALAR(axis_sway_slew, "AXIS_SWY_SLW", 0.0f),
    GSCALAR(axis_yaw_slew, "AXIS_YAW_SLW", 0.0f),
    GSCALAR(dp_yaw_p, "DP_YAW_P", 0.0f),
    GSCALAR(dp_yaw_d, "DP_YAW_D", 0.0f),
    GSCALAR(dp_position_p, "DP_POS_P", 0.0f),
    GSCALAR(dp_velocity_d, "DP_VEL_D", 0.0f),
    GSCALAR(dp_surge_limit, "DP_SRG_MAX", 0.5f),
    GSCALAR(dp_sway_limit, "DP_SWY_MAX", 0.5f),
    GSCALAR(dp_yaw_limit, "DP_YAW_MAX", 0.5f),
    GSCALAR(dp_retarget, "DP_RETARGET", 1),

    AP_VAREND
};
