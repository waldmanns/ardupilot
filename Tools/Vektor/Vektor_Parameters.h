#pragma once

#define AP_PARAM_VEHICLE_NAME vektor

#include "Config.h"

#include <AP_Param/AP_Param.h>

namespace Vektor {

class Parameters {
public:
    enum {
        k_param_format_version = 0,
        k_param_sys_options,
        k_param_sys_desc_page,
        k_param_sys_protocol_baud,
    };

    AP_Int16 format_version;
    AP_Int32 sys_options;
    AP_Int16 sys_desc_page;
    AP_Int32 sys_protocol_baud;
};

} // namespace Vektor
