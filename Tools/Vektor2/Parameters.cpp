#include "Vektor2.h"

#if !AP_MISSION_ENABLED
namespace AP {
AP_Mission *mission()
{
    return nullptr;
}
}
#endif

namespace Vektor2 {

#define APP_SCALAR(v, name, def) { name, (const void *)&app.v, {def_value : def}, 0, Parameters::k_param_ ## v, app.v.vtype }

const AP_Param::Info App::var_info[] = {
    APP_SCALAR(format_version, "FORMAT_VERSION", 0),
#if AP_SIM_ENABLED
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // Board-level orientation and hardware configuration.
    GOBJECT(board_config, "BRD_", AP_BoardConfig),

    // Serial roles and baud rates. In particular:
    //   protocol 2  = MAVLink2
    //   protocol 5  = GPS
    //   protocol 23 = RC input
    GOBJECT(serial_manager, "SERIAL", AP_SerialManager),

    // Vektor2 logic components. Their nested prefixes produce exactly
    // VSP1_LIM / VSP1_THR_ANG and the matching VSP2_* parameters.
    GOBJECT(logic, "", Logic),

    // Compact 32-slot routing configuration exposed through standard MAVLink
    // parameters only: RT1_SRC/RT1_DST ... RT32_SRC/RT32_DST.
    GOBJECT(route_params, "RT", RoutingParameters),

    GOBJECT(rcin, "RC", RcInput),

    GOBJECT(navigation, "VNAV_", NavigationParameters),

    // Per-physical-output scaling and reversal: PWM1_MIN/MAX/INV etc.
    GOBJECT(pwm, "PWM", PwmOut),

    // Raw inertial drivers/calibration.
    GOBJECT(ins, "INS", AP_InertialSensor),

    // Optional magnetometer hardware and calibration parameters.
    GOBJECT(compass, "COMPASS_", Compass),

    // AHRS frontend and EKF3 parameters.
    GOBJECT(ahrs, "AHRS_", AP_AHRS),
#if HAL_NAVEKF3_AVAILABLE
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

    // Standard ArduPilot GPS frontend and parameter group.
    GOBJECT(gps, "GPS", AP_GPS),

#if HAL_GCS_ENABLED
    // MAV_SYSID, stream/message intervals and the normal ArduPilot MAVLink
    // configuration surface.
    GOBJECT(_gcs, "MAV", GCS),
#endif

    AP_VAREND
};
#undef APP_SCALAR

void App::load_parameters()
{
    AP_Param::setup_sketch_defaults();
    AP_Param::check_var_info();

    // Use Rover's AP_GPS defaults: AUTO detection and automatic receiver
    // configuration (230400 baud for u-blox). Saved settings take precedence.
    AP_Param::set_default_by_name("AHRS_EKF_TYPE", 3);
    AP_Param::set_default_by_name("SERIAL0_PROTOCOL", 2); // MAVLink2
    AP_Param::set_default_by_name("SERIAL1_PROTOCOL", 23); // RC input
    AP_Param::set_default_by_name("SERIAL3_PROTOCOL", 5); // GPS
    AP_Param::set_default_by_name("GPS_AUTO_CONFIG", 1);
    AP_Param::set_default_by_name("GPS1_DELAY_MS", 120); // M10 timing; allows EKF startup without GPS
#if HAL_NAVEKF3_AVAILABLE
    AP_Param::set_default_by_name("EK3_SRC1_POSXY", 3); // GPS position
    AP_Param::set_default_by_name("EK3_SRC1_VELXY", 3); // GPS velocity
    AP_Param::set_default_by_name("EK3_SRC1_POSZ", 0);  // synthetic height; no baro
    AP_Param::set_default_by_name("EK3_SRC1_VELZ", 3);  // GPS vertical speed
    AP_Param::set_default_by_name("EK3_SRC1_YAW", 1);   // compass and gyro
    // If no compass is installed, select this source set at startup. GPS
    // position/velocity stay the same; GSF supplies yaw once motion allows it.
    AP_Param::set_default_by_name("EK3_SRC2_POSXY", 3);
    AP_Param::set_default_by_name("EK3_SRC2_VELXY", 3);
    AP_Param::set_default_by_name("EK3_SRC2_POSZ", 0);
    AP_Param::set_default_by_name("EK3_SRC2_VELZ", 3);
    AP_Param::set_default_by_name("EK3_SRC2_YAW", 8);
#endif

    AP_Param::load_all();

#if HAL_NAVEKF3_AVAILABLE
    // This firmware has no barometer and does not need GPS altitude. A GPS
    // height source without a fix supplies no observations, preventing the
    // EKF from constraining tilt while unaided. Override older saved values
    // in RAM; leave yaw sources and saved parameters unchanged.
    AP_Param::set_by_name("EK3_SRC1_POSZ", int8_t(AP_NavEKF_Source::SourceZ::NONE));
    AP_Param::set_by_name("EK3_SRC2_POSZ", int8_t(AP_NavEKF_Source::SourceZ::NONE));
#endif

    // EKF3 cannot allocate its GPS observation buffer when a fixed u-blox
    // type has no detected receiver and GPS1_DELAY_MS is zero. A stored zero
    // from older firmware must not prevent IMU-only EKF startup. Keep any
    // explicitly configured nonzero delay.
    enum ap_var_type gps_delay_type;
    AP_Param *gps_delay = AP_Param::find("GPS1_DELAY_MS", &gps_delay_type);
    if (gps_delay != nullptr && gps_delay_type == AP_PARAM_INT16 &&
        static_cast<AP_Int16 *>(gps_delay)->get() == 0) {
        AP_Param::set_by_name("GPS1_DELAY_MS", 120);
    }
}

} // namespace Vektor2
