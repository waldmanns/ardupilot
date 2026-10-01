#pragma once

#include <AP_Param/AP_Param.h>

#include "VSP1.h"
#include "VSP2.h"
#include "ThrusterBow.h"
#include "ThrusterStern.h"

namespace Vektor2 {

inline uint16_t vsp_parameter_value(int16_t value)
{
    return uint16_t(value < 0 ? 0 : (value > 10 ? 10 : value));
}

inline int16_t vsp_angle_value(int16_t value)
{
    return value < 0 ? 0 : (value > 359 ? 359 : value);
}

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
            vsp_parameter_value(dir.get()),
            vsp_angle_value(thr_ang.get()),
        };
    }

    AP_Int16 lim;
    AP_Int16 x_c;
    AP_Int16 y_c;
    AP_Int16 dir;
    AP_Int16 thr_ang;
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
            vsp_parameter_value(dir.get()),
            vsp_angle_value(thr_ang.get()),
        };
    }

    AP_Int16 lim;
    AP_Int16 x_c;
    AP_Int16 y_c;
    AP_Int16 dir;
    AP_Int16 thr_ang;
};

class ThrusterBowParameterStore {
public:
    ThrusterBowParameterStore() { AP_Param::setup_object_defaults(this, var_info); }

    static const AP_Param::GroupInfo var_info[];

    ThrusterBowParams values() const
    {
        return { int16_t(mid.get()), int16_t(dst.get()) };
    }

    AP_Int16 mid;
    AP_Int16 dst;
};

class ThrusterSternParameterStore {
public:
    ThrusterSternParameterStore() { AP_Param::setup_object_defaults(this, var_info); }

    static const AP_Param::GroupInfo var_info[];

    ThrusterSternParams values() const
    {
        return { int16_t(mid.get()), int16_t(dst.get()) };
    }

    AP_Int16 mid;
    AP_Int16 dst;
};

} // namespace Vektor2
