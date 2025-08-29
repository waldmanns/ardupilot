#pragma once
#include <AP_Param/AP_Param.h>


class VSPVESSEL {
public:
    VSPVESSEL();

    AP_Float vsp1_ring_limiter;
    AP_Float vsp1_x_center;
    AP_Float vsp1_y_center;
    AP_Float vsp1_thrust_dir;

    AP_Float vsp2_ring_limiter;
    AP_Float vsp2_x_center;
    AP_Float vsp2_y_center;
    AP_Float vsp2_thrust_dir;

    AP_Float vsp3_ring_limiter;
    AP_Float vsp3_x_center;
    AP_Float vsp3_y_center;
    AP_Float vsp3_thrust_dir;

    AP_Float vsp4_ring_limiter;
    AP_Float vsp4_x_center;
    AP_Float vsp4_y_center;
    AP_Float vsp4_thrust_dir;

    static const AP_Param::GroupInfo var_info[];

protected:
    
};
