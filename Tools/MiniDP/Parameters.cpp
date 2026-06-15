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
#if HAL_GCS_ENABLED
    GOBJECT(gcs_backend, "MAV", GCS),
#endif
    GOBJECT(barometer, "BARO", AP_Baro),
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

    AP_VAREND
};
