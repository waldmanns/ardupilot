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
    // @DisplayName: Heading hold vector neutral interval
    // @Description: Half-width of the inclusive neutral interval around 1500 us on both VSP forward and sideways inputs. Movement outside any one interval immediately releases both heading corrections. RPM is excluded.
    // @Units: us
    // @Range: 0 250
    // @User: Standard
    AP_GROUPINFO("STK_DB", 2, HeadingHoldParameters, stick_deadband, 30),

    // @Param: IDLE_MS
    // @DisplayName: Heading hold neutral wait
    // @Description: Selected VSP vector inputs and enabled thruster controls must remain neutral, with valid heading sensors, for this interval. Current heading is captured at its end, including after sensor or input recovery.
    // @Units: ms
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("IDLE_MS", 3, HeadingHoldParameters, idle_ms, 1500),

    // @Param: P
    // @DisplayName: Heading hold proportional gain
    // @Description: Normalized yaw demand per degree of heading error beyond HDG_DB.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("P", 4, HeadingHoldParameters, p, 0),

    // @Param: I
    // @DisplayName: Heading hold integral gain
    // @Description: Normalized yaw demand per degree-second of heading error beyond HDG_DB. Limited by IMAX with anti-windup for slew and actuator limits.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("I", 5, HeadingHoldParameters, i, 0),

    // @Param: D
    // @DisplayName: Heading hold rate damping
    // @Description: Normalized yaw demand per degree/second of filtered earth-frame yaw rate. Damping remains active inside HDG_DB.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("D", 6, HeadingHoldParameters, d, 0),

    // @Param: IMAX
    // @DisplayName: Heading hold integral limit
    // @Description: Maximum magnitude of the normalized yaw integral contribution.
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
    // @Description: Maximum normalized yaw effort. VSP1 commands forward and VSP2 backward for positive effort, through their vector mappers.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("MAX", 9, HeadingHoldParameters, maximum, 0.2),

    // @Param: SLEW
    // @DisplayName: Heading hold correction slew
    // @Description: Normalized yaw effort change limit per second, further bounded by both VSP VSLEW values and participating thruster limits. Manual override, disable and invalidity release automatic effort immediately.
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

    // Storage IDs 12 and 13 retired with legacy rod-Y inversion. Do not reuse.

    // @Param: TEL_HZ
    // @DisplayName: Heading hold telemetry rate
    // @Description: Rate for heading measurement and correction named floats, independent of VSP_TEL_HZ. Zero disables these measurements. While heading hold is enabled, state, inhibition flags and reason string continue at 1 Hz below 10, or 2 Hz at 10 and above. Telemetry rate changes do not reset heading hold.
    // @Units: Hz
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("TEL_HZ", 14, HeadingHoldParameters, tel_hz, 5),
    // @Param: VSP_EN
    // @DisplayName: VSP heading participation
    // @Description: Enables paired vector VSP heading. Both drives must have VMODE=1 and valid neutral X/Y controls; RPM input is not required. Set zero for independent thruster-only heading hold; bow/stern participation uses each thruster HDG_EN.
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
    return s;
}
} // namespace Vektor2
