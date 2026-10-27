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
    // @Description: Neutral and stop pulse for bow thruster, 1000..2000 us before final PWM scaling.
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("MID", 1, ThrusterBowParameterStore, mid, 1500),

    // @Param: DST
    // @DisplayName: Bow thruster distance
    // @Description: Distance reserved for bow thruster control logic.
    // @User: Standard
    AP_GROUPINFO("DST", 2, ThrusterBowParameterStore, dst, 0),

    // @Param: ENABLE
    // @DisplayName: ThrusterBow ENABLE
    // @Description: Master enable. Disabled sends MID; enabling requires a valid neutral input.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 3, ThrusterBowParameterStore, enable, 0),

    // @Param: DB
    // @DisplayName: ThrusterBow DB
    // @Description: Manual neutral deadband around MID in microseconds.
    // @User: Standard
    AP_GROUPINFO("DB", 4, ThrusterBowParameterStore, deadband, 30),

    // @Param: MAX
    // @DisplayName: ThrusterBow MAX
    // @Description: Maximum manual displacement as a fraction of symmetric travel around MID.
    // @User: Standard
    AP_GROUPINFO("MAX", 5, ThrusterBowParameterStore, maximum, 1),

    // @Param: SLEW
    // @DisplayName: ThrusterBow SLEW
    // @Description: Manual slew in symmetric travel fractions per second. Also bounds automatic heading slew.
    // @User: Standard
    AP_GROUPINFO("SLEW", 6, ThrusterBowParameterStore, slew, 2),

    // @Param: HDG_EN
    // @DisplayName: ThrusterBow HDG_EN
    // @Description: Participate in shared heading hold when enabled and neutral. Independent of VSPs.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("HDG_EN", 7, ThrusterBowParameterStore, heading_enable, 0),

    // @Param: HDG_SIGN
    // @DisplayName: ThrusterBow HDG_SIGN
    // @Description: PWM direction for positive heading demand; calibrate for this installation.
    // @Values: -1:Negative,1:Positive
    // @User: Standard
    AP_GROUPINFO("HDG_SIGN", 8, ThrusterBowParameterStore, heading_sign, 1),

    // @Param: HDG_MAX
    // @DisplayName: ThrusterBow HDG_MAX
    // @Description: Symmetric travel fraction at unit heading demand. Must not exceed MAX. VHDG_MAX further limits the shared demand.
    // @User: Standard
    AP_GROUPINFO("HDG_MAX", 9, ThrusterBowParameterStore, heading_maximum, 0.3),

    AP_GROUPEND
};

const AP_Param::GroupInfo ThrusterSternParameterStore::var_info[] = {
    // @Param: MID
    // @DisplayName: Stern thruster midpoint
    // @Description: Neutral and stop pulse for stern thruster, 1000..2000 us before final PWM scaling.
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("MID", 1, ThrusterSternParameterStore, mid, 1500),

    // @Param: DST
    // @DisplayName: Stern thruster distance
    // @Description: Distance reserved for stern thruster control logic.
    // @User: Standard
    AP_GROUPINFO("DST", 2, ThrusterSternParameterStore, dst, 0),

    // @Param: ENABLE
    // @DisplayName: ThrusterStern ENABLE
    // @Description: Master enable. Disabled sends MID; enabling requires a valid neutral input.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 3, ThrusterSternParameterStore, enable, 0),

    // @Param: DB
    // @DisplayName: ThrusterStern DB
    // @Description: Manual neutral deadband around MID in microseconds.
    // @User: Standard
    AP_GROUPINFO("DB", 4, ThrusterSternParameterStore, deadband, 30),

    // @Param: MAX
    // @DisplayName: ThrusterStern MAX
    // @Description: Maximum manual displacement as a fraction of symmetric travel around MID.
    // @User: Standard
    AP_GROUPINFO("MAX", 5, ThrusterSternParameterStore, maximum, 1),

    // @Param: SLEW
    // @DisplayName: ThrusterStern SLEW
    // @Description: Manual slew in symmetric travel fractions per second. Also bounds automatic heading slew.
    // @User: Standard
    AP_GROUPINFO("SLEW", 6, ThrusterSternParameterStore, slew, 2),

    // @Param: HDG_EN
    // @DisplayName: ThrusterStern HDG_EN
    // @Description: Participate in shared heading hold when enabled and neutral. Independent of VSPs.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("HDG_EN", 7, ThrusterSternParameterStore, heading_enable, 0),

    // @Param: HDG_SIGN
    // @DisplayName: ThrusterStern HDG_SIGN
    // @Description: PWM direction for positive heading demand; calibrate for this installation.
    // @Values: -1:Negative,1:Positive
    // @User: Standard
    AP_GROUPINFO("HDG_SIGN", 8, ThrusterSternParameterStore, heading_sign, 1),

    // @Param: HDG_MAX
    // @DisplayName: ThrusterStern HDG_MAX
    // @Description: Symmetric travel fraction at unit heading demand. Must not exceed MAX. VHDG_MAX further limits the shared demand.
    // @User: Standard
    AP_GROUPINFO("HDG_MAX", 9, ThrusterSternParameterStore, heading_maximum, 0.3),

    AP_GROUPEND
};

} // namespace Vektor2
