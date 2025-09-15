#include<stdio.h>
#include "Rover.h"
#include "vspvessel.h"


VSPVESSEL::VSPVESSEL(){


    

}

const AP_Param::GroupInfo VSPVESSEL::var_info[] = {
    AP_GROUPINFO("_P1_RLIM", 0, VSPVESSEL, vsp1_ring_limiter, 0.01),
    AP_GROUPINFO("_P1_XCEN", 1, VSPVESSEL, vsp1_x_center, 0.0),
    AP_GROUPINFO("_P1_YCEN", 2, VSPVESSEL, vsp1_y_center, 0.0),
    AP_GROUPINFO("_P1_THRD", 3, VSPVESSEL, vsp1_thrust_dir, 0),
    AP_GROUPINFO("_P2_RLIM", 4, VSPVESSEL, vsp2_ring_limiter, 0.01),
    AP_GROUPINFO("_P2_XCEN", 5, VSPVESSEL, vsp2_x_center, 0.0),
    AP_GROUPINFO("_P2_YCEN", 6, VSPVESSEL, vsp2_y_center, 0.0),
    AP_GROUPINFO("_P2_THRD", 7, VSPVESSEL, vsp2_thrust_dir, 0),
    AP_GROUPINFO("_P3_RLIM", 8, VSPVESSEL, vsp3_ring_limiter, 0.01),
    AP_GROUPINFO("_P3_XCEN", 9, VSPVESSEL, vsp3_x_center, 0.0),
    AP_GROUPINFO("_P3_YCEN", 10, VSPVESSEL, vsp3_y_center, 0.0),
    AP_GROUPINFO("_P3_THRD", 11, VSPVESSEL, vsp3_thrust_dir, 0),
    AP_GROUPINFO("_P4_RLIM", 12, VSPVESSEL, vsp4_ring_limiter, 0.01),
    AP_GROUPINFO("_P4_XCEN", 13, VSPVESSEL, vsp4_x_center, 0.0),
    AP_GROUPINFO("_P4_YCEN", 14, VSPVESSEL, vsp4_y_center, 0.0),
    AP_GROUPINFO("_P4_THRD", 15, VSPVESSEL, vsp4_thrust_dir, 0),
    AP_GROUPEND
    };






// returns true if vehicle is a balance bot
// called in AP_MotorsUGV::output()
// this affects whether the vehicle tries to control its pitch with throttle output
bool Rover::is_vspvessel() const
{
    return ((enum frame_class)g2.frame_class.get() == FRAME_VSPVESSEL);
}
