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
    // VSP1_LIM / VSP1_X_C / VSP1_Y_C and the matching VSP2_* parameters.
    GOBJECT(logic, "", Logic),

    // Compact 32-slot routing configuration exposed through standard MAVLink
    // parameters only: RT1_SRC/RT1_DST ... RT32_SRC/RT32_DST.
    GOBJECT(route_params, "RT", RoutingParameters),

    // Raw inertial drivers/calibration.
    GOBJECT(ins, "INS", AP_InertialSensor),

    // Optional magnetometer hardware and calibration parameters.
    GOBJECT(compass, "COMPASS_", Compass),

    // AHRS frontend and EKF3 parameters.
    GOBJECT(ahrs, "AHRS_", AP_AHRS),
#if HAL_NAVEKF3_AVAILABLE
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

    // Kept from day one but disabled by AP_GPS defaults until a receiver is
    // configured. This avoids a later architectural change when GPS is added.
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

    // Vektor2 starts as an unaided IMU/EKF application. EKF3 is selected now,
    // while all external position/velocity/height/yaw sources are deliberately
    // disabled. GPS can later be enabled by changing EK3_SRC1_* and GPS/SERIAL
    // parameters, without touching the application architecture.
    AP_Param::set_default_by_name("AHRS_EKF_TYPE", 3);
    AP_Param::set_default_by_name("SERIAL0_PROTOCOL", 2); // MAVLink2
#if HAL_NAVEKF3_AVAILABLE
    AP_Param::set_default_by_name("EK3_SRC1_POSXY", 0);
    AP_Param::set_default_by_name("EK3_SRC1_VELXY", 0);
    AP_Param::set_default_by_name("EK3_SRC1_POSZ", 0);
    AP_Param::set_default_by_name("EK3_SRC1_VELZ", 0);
    AP_Param::set_default_by_name("EK3_SRC1_YAW", 0);
#endif

    AP_Param::load_all();
}

} // namespace Vektor2
