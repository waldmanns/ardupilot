#include "ComponentParameters.h"

namespace Vektor2 {

const AP_Param::GroupInfo VSP1ParameterStore::var_info[] = {
    // @Param: LIM
    // @DisplayName: VSP1 limit
    // @Description: Custom VSP1 limit value. Interpretation belongs to VSP1 logic.
    // @User: Standard
    AP_GROUPINFO("LIM", 1, VSP1ParameterStore, lim, 25),

    // @Param: X_C
    // @DisplayName: VSP1 X_C
    // @Description: Custom VSP1 X_C value. Interpretation belongs to VSP1 logic.
    // @User: Standard
    AP_GROUPINFO("X_C", 2, VSP1ParameterStore, x_c, 1500),

    // @Param: Y_C
    // @DisplayName: VSP1 Y_C
    // @Description: Custom VSP1 Y_C value. Interpretation belongs to VSP1 logic.
    // @User: Standard
    AP_GROUPINFO("Y_C", 3, VSP1ParameterStore, y_c, 1500),

    // @Param: DIR
    // @DisplayName: VSP1 direction
    // @Description: VSP1 direction value.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("DIR", 4, VSP1ParameterStore, dir, 0),

    // @Param: THR_ANG
    // @DisplayName: VSP1 threshold angle
    // @Description: Threshold angle supplied to VSP1 logic; values outside 0..359 are clamped for the loop.
    // @Units: deg
    // @Range: 0 359
    // @User: Standard
    AP_GROUPINFO("THR_ANG", 5, VSP1ParameterStore, thr_ang, 0),

    // @Group: RS_
    // @Path: RollStabilizationParams.cpp
    AP_SUBGROUPINFO(roll, "RS_", 6, VSP1ParameterStore, RollDriveParameters),

    AP_GROUPEND
};

const AP_Param::GroupInfo VSP2ParameterStore::var_info[] = {
    // @Param: LIM
    // @DisplayName: VSP2 limit
    // @Description: Custom VSP2 limit value. Interpretation belongs to VSP2 logic.
    // @User: Standard
    AP_GROUPINFO("LIM", 1, VSP2ParameterStore, lim, 25),

    // @Param: X_C
    // @DisplayName: VSP2 X_C
    // @Description: Custom VSP2 X_C value. Interpretation belongs to VSP2 logic.
    // @User: Standard
    AP_GROUPINFO("X_C", 2, VSP2ParameterStore, x_c, 1500),

    // @Param: Y_C
    // @DisplayName: VSP2 Y_C
    // @Description: Custom VSP2 Y_C value. Interpretation belongs to VSP2 logic.
    // @User: Standard
    AP_GROUPINFO("Y_C", 3, VSP2ParameterStore, y_c, 1500),

    // @Param: DIR
    // @DisplayName: VSP2 direction
    // @Description: VSP2 direction value.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("DIR", 4, VSP2ParameterStore, dir, 0),

    // @Param: THR_ANG
    // @DisplayName: VSP2 threshold angle
    // @Description: Threshold angle supplied to VSP2 logic; values outside 0..359 are clamped for the loop.
    // @Units: deg
    // @Range: 0 359
    // @User: Standard
    AP_GROUPINFO("THR_ANG", 5, VSP2ParameterStore, thr_ang, 0),

    // @Group: RS_
    // @Path: RollStabilizationParams.cpp
    AP_SUBGROUPINFO(roll, "RS_", 6, VSP2ParameterStore, RollDriveParameters),

    AP_GROUPEND
};

const AP_Param::GroupInfo ThrusterBowParameterStore::var_info[] = {
    // @Param: MID
    // @DisplayName: Bow thruster midpoint
    // @Description: Midpoint reserved for bow thruster control logic.
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("MID", 1, ThrusterBowParameterStore, mid, 1500),

    // @Param: DST
    // @DisplayName: Bow thruster distance
    // @Description: Distance reserved for bow thruster control logic.
    // @User: Standard
    AP_GROUPINFO("DST", 2, ThrusterBowParameterStore, dst, 0),

    AP_GROUPEND
};

const AP_Param::GroupInfo ThrusterSternParameterStore::var_info[] = {
    // @Param: MID
    // @DisplayName: Stern thruster midpoint
    // @Description: Midpoint reserved for stern thruster control logic.
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("MID", 1, ThrusterSternParameterStore, mid, 1500),

    // @Param: DST
    // @DisplayName: Stern thruster distance
    // @Description: Distance reserved for stern thruster control logic.
    // @User: Standard
    AP_GROUPINFO("DST", 2, ThrusterSternParameterStore, dst, 0),

    AP_GROUPEND
};

} // namespace Vektor2
