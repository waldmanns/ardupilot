#include "RollStabilizationParams.h"

namespace Vektor2 {

const AP_Param::GroupInfo RollStabilizationParameters::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: VSP roll stabilization enable
    // @Description: Enables additive fixed-vector roll bursts after each VSP ring limiter. Zero bypasses all VRS output changes and VRS telemetry. This application has no arming state. Configure drive angles/effect signs and tune gains before enabling.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, RollStabilizationParameters, enable, 0),

    // @Param: P
    // @DisplayName: Roll angle gain
    // @Description: Requested correction in ring radii per degree of roll error. Start with zero and tune rate damping first. No integral term is used.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("P", 2, RollStabilizationParameters, p, 0),

    // @Param: D
    // @DisplayName: Roll rate damping gain
    // @Description: Requested correction in ring radii per degree/second of bias-corrected body roll rate. Zero by default until the actuator effect sign has been established.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("D", 3, RollStabilizationParameters, d, 0),

    // @Param: TRIM
    // @DisplayName: Target roll angle
    // @Description: Roll angle used by the proportional term. Positive is positive AHRS body roll, normally starboard down on a correctly oriented vessel.
    // @Units: deg
    // @Range: -45 45
    // @User: Standard
    AP_GROUPINFO("TRIM", 4, RollStabilizationParameters, trim, 0),

    // @Param: FILT_HZ
    // @DisplayName: Roll rate filter cutoff
    // @Description: First-order low-pass cutoff for body roll rate. Excess filtering adds phase delay; tune with the vessel roll period and actuator response.
    // @Units: Hz
    // @Range: 0.1 20
    // @User: Advanced
    AP_GROUPINFO("FILT_HZ", 5, RollStabilizationParameters, filter_hz, 2),

    // @Param: DB
    // @DisplayName: Roll correction deadband
    // @Description: Start correcting above this absolute normalized PD demand. Release below half this value. Applied to demand, not angle, so rate damping works while crossing level.
    // @Range: 0 0.5
    // @User: Standard
    AP_GROUPINFO("DB", 6, RollStabilizationParameters, deadband, 0.02),

    // @Param: ON_MS
    // @DisplayName: Maximum roll burst duration
    // @Description: Maximum correction window including amplitude ramps. Maneuvering remains active throughout. Demand loss or reversal ends the window early. No catch-up bursts are queued.
    // @Units: ms
    // @Range: 20 10000
    // @User: Standard
    AP_GROUPINFO("ON_MS", 7, RollStabilizationParameters, on_ms, 250),

    // @Param: OFF_MS
    // @DisplayName: Minimum interval without roll correction
    // @Description: Shared maneuver-only interval after every burst, including sign reversals. Also applies after enabling, configuration edits and recovery from invalid sensor/control input.
    // @Units: ms
    // @Range: 20 10000
    // @User: Standard
    AP_GROUPINFO("OFF_MS", 8, RollStabilizationParameters, off_ms, 250),

    // @Param: SLEW
    // @DisplayName: Roll correction slew rate
    // @Description: Maximum normal change of added displacement in ring radii per second. Travel/authority limits, disable, loss of input, reversal and burst deadlines override ramp-down immediately.
    // @Range: 0.01 100
    // @User: Advanced
    AP_GROUPINFO("SLEW", 9, RollStabilizationParameters, slew, 1),

    // @Param: MAN_LIM
    // @DisplayName: Maneuvering priority threshold
    // @Description: Maneuver command length divided by ring radius at which roll correction is fully suppressed. Available VRS authority fades linearly from full at neutral to zero at this threshold.
    // @Range: 0.01 1
    // @User: Standard
    AP_GROUPINFO("MAN_LIM", 10, RollStabilizationParameters, maneuver_limit, 0.8),
    AP_GROUPEND
};

const AP_Param::GroupInfo RollDriveParameters::var_info[] = {
    // @Param: ANG
    // @DisplayName: Fixed roll burst vector
    // @Description: Added correction direction in the component POST-limiter X/Y plane. 0=positive X, 90=positive Y, 180=negative X, 270=negative Y. Not a compass bearing. Calibrate for physical port/starboard thrust including downstream output scaling/reversal. THR_ANG is not reapplied to this vector.
    // @Units: deg
    // @Range: 0 359
    // @User: Standard
    AP_GROUPINFO("ANG", 1, RollDriveParameters, angle, 0),

    // @Param: SIGN
    // @DisplayName: Roll effect of the configured burst vector
    // @Description: Sign of the roll moment produced by positive displacement along RS_ANG. Zero excludes this drive from stabilization. Two enabled drives must have opposite signs. This setting selects the drive; it never reverses the vector.
    // @Values: -1:Negative roll effect,0:Unused,1:Positive roll effect
    // @User: Standard
    AP_GROUPINFO("SIGN", 2, RollDriveParameters, sign, 0),

    // @Param: MAX
    // @DisplayName: Maximum added roll correction
    // @Description: Maximum correction displacement divided by this drive's existing ring radius. Maneuvering priority and available travel can reduce it further. This is rod displacement, not measured thrust. Integer PWM quantizes coordinates to microseconds.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("MAX", 3, RollDriveParameters, maximum, 0.2),
    AP_GROUPEND
};

RollSettings RollStabilizationParameters::values() const
{
    RollSettings result;
    result.enabled = enabled();
    result.p = p.get();
    result.d = d.get();
    result.trim_deg = trim.get();
    result.filter_hz = filter_hz.get();
    result.deadband = deadband.get();
    // Preserve invalid values for validation: negative times become out-of-
    // range unsigned values and inhibit VRS, rather than silently becoming valid.
    result.on_ms = uint16_t(on_ms.get());
    result.off_ms = uint16_t(off_ms.get());
    result.slew = slew.get();
    result.maneuver_limit = maneuver_limit.get();
    return result;
}

RollDriveSettings RollDriveParameters::values() const
{
    RollDriveSettings result;
    result.angle_deg = angle.get();
    result.sign = sign.get();
    result.maximum = maximum.get();
    return result;
}

} // namespace Vektor2
