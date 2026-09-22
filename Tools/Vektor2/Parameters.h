#pragma once

// AP_Param's vehicle-style macros operate on the single Vektor2::app object.
#define AP_PARAM_VEHICLE_NAME app

#include <AP_Param/AP_Param.h>

namespace Vektor2 {

// Stable top-level AP_Param keys for the Vektor2 application object. Keep
// existing values stable once firmware is deployed so stored parameters remain
// compatible across upgrades.
class Parameters {
public:
    enum : uint16_t {
        k_param_format_version = 0,
        k_param_board_config,
        k_param_serial_manager,
        k_param_logic,
        k_param_route_params,
        k_param_ins,
        k_param_ahrs,
        k_param_NavEKF3,
        k_param_gps,
        k_param__gcs,
        k_param_sitl,
    };
};

} // namespace Vektor2
