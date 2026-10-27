#pragma once

#include "HeadingHold.h"
#include <AP_Param/AP_Param.h>

namespace Vektor2 {
class HeadingHoldParameters {
public:
    HeadingHoldParameters() { AP_Param::setup_object_defaults(this, var_info); }
    static const AP_Param::GroupInfo var_info[];
    HeadingSettings values() const;
    bool enabled() const { return enable.get() == 1; }
    int16_t telemetry_hz() const { return tel_hz.get(); }

private:
    AP_Int8 enable;
    AP_Int8 vsp_enable;
    AP_Int16 stick_deadband;
    AP_Int16 idle_ms;
    AP_Float p;
    AP_Float i;
    AP_Float d;
    AP_Float imax;
    AP_Float heading_deadband;
    AP_Float maximum;
    AP_Float slew;
    AP_Float filter_hz;
    AP_Int8 invert1;
    AP_Int8 invert2;
    AP_Int16 tel_hz;
};
} // namespace Vektor2
