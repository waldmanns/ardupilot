#include "NavigationSupervisor.h"

namespace Vektor2 {

void NavigationSupervisor::update(const Input& in)
{
    uint16_t reasons = 0;
    if (!in.rc_valid) { reasons |= RCInvalid; }
    if (!in.configuration_valid || in.requested_mode < 0 || in.requested_mode > 2) {
        reasons |= ConfigInvalid;
    }
    if (!in.timing_ok) { reasons |= TimingFault; }

    if (in.requested_mode == 0 || in.requested_mode < 0 || in.requested_mode > 2) {
        // Free movement does not require position, heading, or autonomous
        // outputs. Configuration faults are still visible via NAV_RBITS.
        _status.state = State::Free;
        _status.reason_bits = reasons;
        return;
    }

    uint16_t yaw_blockers = 0;
    if (!in.enabled || !in.heading_enabled) { yaw_blockers |= Disabled; }
    if (!in.heading_valid) { yaw_blockers |= HeadingInvalid; }
    if (!in.rc_valid) { yaw_blockers |= RCInvalid; }
    if (!in.output_ready || !in.heading_command_valid) { yaw_blockers |= OutputNotReady; }
    if (!in.controls_neutral) { yaw_blockers |= WaitNeutral; }
    if (in.pilot_override) { yaw_blockers |= PilotOverride; }
    if (!in.heading_target_valid) { yaw_blockers |= TargetInvalid; }
    if (!in.configuration_valid) { yaw_blockers |= ConfigInvalid; }
    if (!in.timing_ok) { yaw_blockers |= TimingFault; }
    if (in.estimator_reset) { yaw_blockers |= EstimatorReset; }
    if (in.vsp_unavailable) { yaw_blockers |= VspUnavailable; }
    if (in.bow_unavailable) { yaw_blockers |= BowUnavailable; }
    if (in.stern_unavailable) { yaw_blockers |= SternUnavailable; }
    if (in.heading_rearm_required) { yaw_blockers |= HeadingRearmRequired; }

    reasons |= yaw_blockers;
    uint16_t dp_blockers = yaw_blockers;
    if (in.requested_mode == 2) {
        if (!in.dp_enabled) { dp_blockers |= Disabled; }
        if (!in.position_valid) { dp_blockers |= PositionInvalid; }
        if (!in.velocity_valid) { dp_blockers |= VelocityInvalid; }
        if (!in.position_target_valid) { dp_blockers |= TargetInvalid; }
        // No duplicate public DP_RBITS: DP emits its own local blockers and
        // the supervisor merges them into the one mode-aware NAV_RBITS.
        // Mask the input to the defined DP-related reasons.
        dp_blockers |= in.dp_controller_reasons &
                       (HeadingInvalid | PositionInvalid | VelocityInvalid |
                        TargetInvalid | EstimatorReset | ConfigInvalid | TimingFault);
        reasons |= dp_blockers;
    }

    if (in.fault) {
        reasons |= OutputNotReady;
        _status.state = State::Fault;
        _status.reason_bits = reasons;
    } else if (yaw_blockers != 0 || !in.yaw_owned) {
        // No false HDG_ACTIVE before final actuator ownership is confirmed.
        if (!in.yaw_owned) { reasons |= OutputNotReady; }
        _status.state = State::HeadingWait;
        _status.reason_bits = reasons;
    } else if (in.requested_mode == 1) {
        _status.state = State::HeadingActive;
        _status.reason_bits = reasons;
    } else if (dp_blockers != 0 || !in.translation_owned) {
        // The yaw owner remains active during DP_WAIT.
        if (!in.translation_owned) { reasons |= OutputNotReady; }
        _status.state = State::DPWait;
        _status.reason_bits = reasons;
    } else {
        _status.state = State::DPActive;
        _status.reason_bits = reasons;
    }
}

} // namespace Vektor2
