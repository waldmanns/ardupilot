#include "HeadingHoldParams.h"

namespace Vektor2 {
const AP_Param::GroupInfo HeadingHoldParameters::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Heading hold enable
    // @Description: Enables shared heading hold for selected VSPs and independent thrusters. Requires healthy compass yaw and neutral controls for IDLE_MS before capturing heading. Any other value bypasses heading hold.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, HeadingHoldParameters, enable, 0),

    // @Param: STK_DB
    // @DisplayName: Heading hold rod deadzone
    // @Description: Half-width of the inclusive neutral interval around 1500 us on both VSP X and Y inputs. Movement outside any one interval immediately releases both heading corrections. RPM is excluded.
    // @Units: us
    // @Range: 0 250
    // @User: Standard
    AP_GROUPINFO("STK_DB", 2, HeadingHoldParameters, stick_deadband, 30),

    // @Param: IDLE_MS
    // @DisplayName: Heading hold neutral wait
    // @Description: Selected VSP rods and enabled thruster controls must remain neutral, with valid heading sensors, for this interval. Current heading is captured at its end, including after sensor or input recovery.
    // @Units: ms
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("IDLE_MS", 3, HeadingHoldParameters, idle_ms, 1500),

    // @Param: P
    // @DisplayName: Heading hold proportional gain
    // @Description: Added displacement in ring radii per degree of heading error beyond HDG_DB.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("P", 4, HeadingHoldParameters, p, 0),

    // @Param: I
    // @DisplayName: Heading hold integral gain
    // @Description: Added displacement in ring radii per degree-second of heading error beyond HDG_DB. Limited by IMAX with anti-windup for slew and actuator limits.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("I", 5, HeadingHoldParameters, i, 0),

    // @Param: D
    // @DisplayName: Heading hold rate damping
    // @Description: Added displacement in ring radii per degree/second of filtered earth-frame yaw rate. Damping remains active inside HDG_DB.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("D", 6, HeadingHoldParameters, d, 0),

    // @Param: IMAX
    // @DisplayName: Heading hold integral limit
    // @Description: Maximum magnitude of the integral contribution as a fraction of ring radius.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("IMAX", 7, HeadingHoldParameters, imax, 0.1),

    // @Param: HDG_DB
    // @DisplayName: Heading hold heading deadband
    // @Description: Heading error inside this band contributes neither P nor I. Outside it the band width is subtracted from the error. Rate damping remains active.
    // @Units: deg
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("HDG_DB", 8, HeadingHoldParameters, heading_deadband, 1),

    // @Param: MAX
    // @DisplayName: Heading hold correction limit
    // @Description: Maximum automatic correction as a fraction of each VSP ring radius. Both drives share the normalized demand and yield together to the smaller available Y travel.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("MAX", 9, HeadingHoldParameters, maximum, 0.2),

    // @Param: SLEW
    // @DisplayName: Heading hold correction slew
    // @Description: Normal correction change limit in ring radii per second. Manual override, disable, invalidity and reduced physical travel remove or reduce correction immediately.
    // @Range: 0.01 100
    // @User: Standard
    AP_GROUPINFO("SLEW", 10, HeadingHoldParameters, slew, 0.5),

    // @Param: FILT_HZ
    // @DisplayName: Heading hold rate filter
    // @Description: First-order low-pass cutoff for earth-frame yaw rate.
    // @Units: Hz
    // @Range: 0.1 20
    // @User: Standard
    AP_GROUPINFO("FILT_HZ", 11, HeadingHoldParameters, filter_hz, 2),

    // @Param: Y1_INV
    // @DisplayName: Invert VSP1 automatic heading Y
    // @Description: Reverses only the automatic VSP1 heading correction in post-mapping actuator coordinates. Does not reverse manual input or physical PWM output.
    // @Values: 0:Normal,1:Inverted
    // @User: Standard
    AP_GROUPINFO("Y1_INV", 12, HeadingHoldParameters, invert1, 0),

    // @Param: Y2_INV
    // @DisplayName: Invert VSP2 automatic heading Y
    // @Description: Reverses only the automatic VSP2 heading correction in post-mapping actuator coordinates. Does not reverse manual input or physical PWM output.
    // @Values: 0:Normal,1:Inverted
    // @User: Standard
    AP_GROUPINFO("Y2_INV", 13, HeadingHoldParameters, invert2, 0),

    // @Param: TEL_HZ
    // @DisplayName: Heading hold telemetry rate
    // @Description: Rate for heading measurement and correction named floats, independent of VSP_TEL_HZ. Zero disables these measurements. While heading hold is enabled, state, inhibition flags and reason string continue at 1 Hz below 10, or 2 Hz at 10 and above. Telemetry rate changes do not reset heading hold.
    // @Units: Hz
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("TEL_HZ", 14, HeadingHoldParameters, tel_hz, 5),
    // @Param: VSP_EN
    // @DisplayName: VSP heading participation
    // @Description: Enables the existing paired VSP heading corrections and input requirements. Set zero for independent thruster-only heading hold; bow/stern participation uses each thruster HDG_EN.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("VSP_EN", 15, HeadingHoldParameters, vsp_enable, 1),
    AP_GROUPEND
};

HeadingSettings HeadingHoldParameters::values() const
{
    HeadingSettings s;
    s.enabled = enabled();
    s.vsp_enabled = vsp_enable.get();
    s.stick_deadband = stick_deadband.get();
    s.idle_ms = idle_ms.get();
    s.p = p.get();
    s.i = i.get();
    s.d = d.get();
    s.imax = imax.get();
    s.heading_deadband = heading_deadband.get();
    s.maximum = maximum.get();
    s.slew = slew.get();
    s.filter_hz = filter_hz.get();
    s.invert[0] = invert1.get();
    s.invert[1] = invert2.get();
    return s;
}
} // namespace Vektor2
