#pragma once

#include "RollStabilization.h"
#include <AP_Param/AP_Param.h>

namespace Vektor2 {

// Parameter storage stays out of the controller and VSP algorithms. Existing
// component storage IDs are preserved; these are new subgroups only.
class RollStabilizationParameters {
public:
    RollStabilizationParameters() { AP_Param::setup_object_defaults(this, var_info); }
    static const AP_Param::GroupInfo var_info[];
    RollSettings values() const;
    bool enabled() const { return enable.get() == 1; }

private:
    AP_Int8 enable;
    AP_Float p;
    AP_Float d;
    AP_Float trim;
    AP_Float filter_hz;
    AP_Float deadband;
    AP_Int16 on_ms;
    AP_Int16 off_ms;
    AP_Float slew;
    AP_Float maneuver_limit;
};

class RollDriveParameters {
public:
    RollDriveParameters() { AP_Param::setup_object_defaults(this, var_info); }
    static const AP_Param::GroupInfo var_info[];
    RollDriveSettings values() const;

private:
    AP_Int16 angle;
    AP_Int8 sign;
    AP_Float maximum;
};

} // namespace Vektor2
