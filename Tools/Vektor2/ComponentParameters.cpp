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
    // @Description: Rotor rotation viewed from above. Used by the thrust mapper; physical positive X moves right and positive Y up.
    // @Values: 0:CW,1:CCW
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

    // Storage ID 7 is reserved (removed experimental MAP_ROT).
    // @Param: MAP_ANG
    // @DisplayName: VSP1 installation angle
    // @Description: Local forward axis clockwise from vessel bow. Used once by the mapper, independently of the legacy THR_ANG rod transform.
    // @Units: deg
    // @Range: 0 359.99
    // @User: Standard
    AP_GROUPINFO("MAP_ANG", 8, VSP1ParameterStore, map_angle, 0),

    // @Param: VMODE
    // @DisplayName: VSP1 input control mode
    // @Description: Vector mode interprets IN1 as forward and IN2 as starboard demand. Mode changes require centered inputs. VSP heading requires both drives in vector mode. VRS remains inhibited while either drive uses vector mode.
    // @Values: 0:Legacy rods,1:Manual thrust vector
    // @User: Standard
    AP_GROUPINFO("VMODE", 9, VSP1ParameterStore, vmode, 0),

    // @Param: VDB
    // @DisplayName: VSP1 vector input deadband
    // @Description: Half width of the neutral interval around 1500 us. Used for vector commands and neutral-before-movement transitions.
    // @Units: us
    // @Range: 0 250
    // @User: Standard
    AP_GROUPINFO("VDB", 10, VSP1ParameterStore, vector_deadband, 30),

    // @Param: VSLEW
    // @DisplayName: VSP1 vector slew rate
    // @Description: Maximum normalized vector change per second. Neutral and invalid input stop immediately.
    // @Range: 0.01 100
    // @User: Standard
    AP_GROUPINFO("VSLEW", 11, VSP1ParameterStore, vector_slew, 2),

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
    // @Description: Rotor rotation viewed from above. Used by the thrust mapper; physical positive X moves right and positive Y up.
    // @Values: 0:CW,1:CCW
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

    // Storage ID 7 is reserved (removed experimental MAP_ROT).
    // @Param: MAP_ANG
    // @DisplayName: VSP2 installation angle
    // @Description: Local forward axis clockwise from vessel bow. Used once by the mapper, independently of the legacy THR_ANG rod transform.
    // @Units: deg
    // @Range: 0 359.99
    // @User: Standard
    AP_GROUPINFO("MAP_ANG", 8, VSP2ParameterStore, map_angle, 0),

    // @Param: VMODE
    // @DisplayName: VSP2 input control mode
    // @Description: Vector mode interprets IN1 as forward and IN2 as starboard demand. Mode changes require centered inputs. VSP heading requires both drives in vector mode. VRS remains inhibited while either drive uses vector mode.
    // @Values: 0:Legacy rods,1:Manual thrust vector
    // @User: Standard
    AP_GROUPINFO("VMODE", 9, VSP2ParameterStore, vmode, 0),

    // @Param: VDB
    // @DisplayName: VSP2 vector input deadband
    // @Description: Half width of the neutral interval around 1500 us. Used for vector commands and neutral-before-movement transitions.
    // @Units: us
    // @Range: 0 250
    // @User: Standard
    AP_GROUPINFO("VDB", 10, VSP2ParameterStore, vector_deadband, 30),

    // @Param: VSLEW
    // @DisplayName: VSP2 vector slew rate
    // @Description: Maximum normalized vector change per second. Neutral and invalid input stop immediately.
    // @Range: 0.01 100
    // @User: Standard
    AP_GROUPINFO("VSLEW", 11, VSP2ParameterStore, vector_slew, 2),

    AP_GROUPEND
};

const AP_Param::GroupInfo AzipodParameterStore::var_info[] = {
    // Layout selects installed drives; these parameters are calibration only.
    // @Param: ANG_MIN
    // @Description: Mechanical angle at logical 1000 us (degrees).
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("ANG_MIN", 2, AzipodParameterStore, angle_min, -90),
    // @Param: ANG_MAX
    // @Description: Mechanical angle at logical 2000 us (degrees). Travel must be less than 360 degrees.
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("ANG_MAX", 3, AzipodParameterStore, angle_max, 90),
    // @Param: OFFSET
    // @Description: Body thrust angle at mechanical zero (degrees clockwise).
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("OFFSET", 4, AzipodParameterStore, offset, 0),
    // @Param: REV
    // @Description: ESC type: 0 forward only (1000 stop), 1 reversible (1500 stop).
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("REV", 5, AzipodParameterStore, reversible, 0),
    // @Param: REV_OPT
    // @Description: Prefer short steering travel using reverse when possible.
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("REV_OPT", 6, AzipodParameterStore, reverse_opt, 1),
    // @Param: UNREACH
    // @Description: 0 stop for unreachable thrust, 1 project onto reachable direction.
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("UNREACH", 7, AzipodParameterStore, unreachable, 0),
    // @Param: DB
    // @Description: Radial joystick deadband in microseconds.
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("DB", 8, AzipodParameterStore, deadband, 25),
    // @Param: RATE
    // @Description: Conservative estimated servo travel rate in degrees per second.
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("RATE", 9, AzipodParameterStore, rate, 90),
    // @Param: REV_HYST
    // @Description: Extra steering cost in degrees for changing motor direction.
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("REV_HYST", 10, AzipodParameterStore, hysteresis, 15),
    // @Param: REV_DLY
    // @Description: Minimum neutral dwell before reversing in milliseconds.
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("REV_DLY", 11, AzipodParameterStore, reverse_delay, 500),
    // @Param: ALIGN
    // @Description: Maximum commanded steering error for motor thrust in degrees.
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("ALIGN", 12, AzipodParameterStore, align, 5),
    // @Param: SLEW
    // @Description: Motor thrust slew in normalized units per second.
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("SLEW", 13, AzipodParameterStore, motor_slew, 1.0f),
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
    // @Description: Positive longitudinal distance forward of the vessel center, in centimeters. Required for automatic yaw when HDG_EN is enabled.
    // @Units: cm
    // @Range: 0 32767
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
    // @Description: Symmetric travel fraction at unit heading demand. Must not exceed MAX. The navigation yaw allocator will limit the shared demand.
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
    // @Description: Positive longitudinal distance aft of the vessel center, in centimeters. Required for automatic yaw when HDG_EN is enabled.
    // @Units: cm
    // @Range: 0 32767
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
    // @Description: Symmetric travel fraction at unit heading demand. Must not exceed MAX. The navigation yaw allocator will limit the shared demand.
    // @User: Standard
    AP_GROUPINFO("HDG_MAX", 9, ThrusterSternParameterStore, heading_maximum, 0.3),

    AP_GROUPEND
};

} // namespace Vektor2
