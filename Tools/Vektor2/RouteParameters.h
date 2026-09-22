#pragma once

#include "Config.h"

#include <AP_Param/AP_Param.h>
#include <stdint.h>

namespace Vektor2 {

class Routing;
class PwmOut;

class RouteSlotParameters {
public:
    RouteSlotParameters()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const AP_Param::GroupInfo var_info[];

    // Compact standard-MAVLink parameter representation:
    // SRC: 0 disabled; 1..18 RC1..RC18; 101/102 VSP1 O1/O2;
    //      111/112 VSP2 O1/O2.
    // DST: 0 disabled; 1..32 PWM1..PWM32; 101/102 VSP1 I1/I2;
    //      111/112 VSP2 I1/I2.
    AP_Int16 source;
    AP_Int16 destination;
};

class RoutingParameters {
public:
    RoutingParameters();

    static const AP_Param::GroupInfo var_info[];

    // Applies changed parameter values to the runtime router. Returns false if
    // one or more configured routes is invalid, duplicated or cyclic.
    bool sync(Routing& routing, PwmOut& pwm, bool force = false);

    RouteSlotParameters slots[max_routes];

private:
    int16_t _last_source[max_routes] {};
    int16_t _last_destination[max_routes] {};
    bool _have_snapshot = false;
    bool _last_apply_valid = true;
};

} // namespace Vektor2
