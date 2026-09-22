#pragma once

#include <AP_Param/AP_Param.h>

#include "VSP1.h"
#include "VSP2.h"

namespace Vektor2 {

// AP_Param-backed storage stays outside the component algorithms. Each store
// exposes a plain typed snapshot that the logic framework passes to the loop.
class VSP1ParameterStore {
public:
    VSP1ParameterStore() { AP_Param::setup_object_defaults(this, var_info); }

    static const AP_Param::GroupInfo var_info[];

    Components::VSP1Params values() const
    {
        return {
            int16_t(lim.get()),
            int16_t(x_c.get()),
            int16_t(y_c.get()),
        };
    }

    AP_Int16 lim;
    AP_Int16 x_c;
    AP_Int16 y_c;
};

class VSP2ParameterStore {
public:
    VSP2ParameterStore() { AP_Param::setup_object_defaults(this, var_info); }

    static const AP_Param::GroupInfo var_info[];

    Components::VSP2Params values() const
    {
        return {
            int16_t(lim.get()),
            int16_t(x_c.get()),
            int16_t(y_c.get()),
        };
    }

    AP_Int16 lim;
    AP_Int16 x_c;
    AP_Int16 y_c;
};

} // namespace Vektor2
