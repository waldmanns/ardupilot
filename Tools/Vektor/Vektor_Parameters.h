#pragma once

#define AP_PARAM_VEHICLE_NAME vektor

#include "Config.h"
#include "Vektor_SerialRoles.h"

#include <AP_Param/AP_Param.h>

namespace Vektor {

class Parameters {
public:
    enum {
        k_param_format_version = 0,
        k_param_sys_options,
        k_param_sys_desc_page,
        k_param_serial_roles,
        k_param_rcin_timeout_ms,
        k_param_rc_channels,
        k_param_assignments,
        k_param_pwm_input,
        k_param_pwm_output,
#if VEKTOR_ATTITUDE_ENABLED
        k_param_board_config,
        k_param_ins,
        k_param_compass,
        k_param_ahrs,
#endif
    };

    AP_Int16 format_version;
    AP_Int32 sys_options;
    AP_Int16 sys_desc_page;
    SerialRoleParameters serial_roles;
    AP_Int16 rcin_timeout_ms;
};

} // namespace Vektor
