#include "MiniDP.h"

const AP_Param::Info MiniDP::var_info[] = {
    GSCALAR(format_version, "FORMAT_VERSION", 0),
    GSCALAR(log_bitmask, "LOG_BITMASK", -1),

    GOBJECT(board_config, "BRD_", AP_BoardConfig),
    GOBJECT(serial_manager, "SERIAL", AP_SerialManager),
    GOBJECT(ins, "INS", AP_InertialSensor),
    GOBJECT(compass, "COMPASS_", Compass),
    GOBJECT(gps, "GPS", AP_GPS),
#if AP_BATTERY_ENABLED
    GOBJECT(battery, "BATT", AP_BattMonitor),
#endif
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
    GSCALAR(frame_screw_position, "FRAME_SCR_POS", int8_t(MiniDP_ScrewPosition::AFT)),
    GSCALAR(frame_screw_yaw_scale, "FRAME_SCR_YAW", 1.0f),
    GSCALAR(in_rc_surge_channel, "IN_RC_SURGE", 2),
    GSCALAR(in_rc_sway_channel, "IN_RC_SWAY", 1),
    GSCALAR(in_rc_yaw_channel, "IN_RC_YAW", 4),
    GSCALAR(in_rc_kill_channel, "IN_RC_KILL", 0),
    GSCALAR(in_rc_kill_pwm, "IN_KILL_PWM", 1800),
    GSCALAR(in_rc_arm_channel, "IN_RC_ARM", 0),
    GSCALAR(in_rc_arm_pwm, "IN_ARM_PWM", 1800),
    GSCALAR(in_rc_disarm_channel, "IN_RC_DISARM", 0),
    GSCALAR(in_rc_disarm_pwm, "IN_DISARM_PWM", 1200),
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
    GSCALAR(dp_position_radius, "DP_POS_RAD", 2.0f),
    GSCALAR(dp_position_deadband, "DP_POS_DZ", 0.5f),
    GSCALAR(dp_hacc_max, "DP_HACC_MAX", 10.0f),
    GSCALAR(dp_sacc_max, "DP_SACC_MAX", 2.0f),
    GSCALAR(arm_require, "ARMING_REQUIRE", 1),
    GSCALAR(arm_gps_require, "ARM_GPS_REQ", 0),
    GSCALAR(arm_hacc_max, "ARM_HACC_MAX", 10.0f),
    GSCALAR(arm_sacc_max, "ARM_SACC_MAX", 2.0f),
    GSCALAR(out_disarmed_action, "OUT_DARM_ACT", 0),
    GSCALAR(out_failsafe_action, "OUT_FS_ACT", 1),
    GSCALAR(out_kill_action, "OUT_KILL_ACT", 0),
    GSCALAR(out_failsafe_pwm, "OUT_FS_PWM", 1500),

    AP_VAREND
};
