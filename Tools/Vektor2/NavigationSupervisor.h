#pragma once

#include <stdint.h>

namespace Vektor2 {

// The only public navigation state and inhibit interface. The supervisor does
// not generate propulsion commands: the selected actuator owner must
// confirm authority before the supervisor reports an active automatic mode.
class NavigationSupervisor {
public:
    enum class State : uint8_t {
        Free = 0, HeadingWait = 1, HeadingActive = 2,
        DPWait = 3, DPActive = 4, Fault = 5
    };
    enum Reason : uint16_t {
        Disabled = 1U << 0,
        HeadingInvalid = 1U << 1, // AHRS heading estimate unavailable or invalid
        PositionInvalid = 1U << 2,
        VelocityInvalid = 1U << 3,
        RCInvalid = 1U << 4,
        PilotOverride = 1U << 5,
        OutputNotReady = 1U << 6,
        WaitNeutral = 1U << 7,
        TargetInvalid = 1U << 8,
        EstimatorReset = 1U << 9,
        ConfigInvalid = 1U << 10,
        TimingFault = 1U << 11,
        BowUnavailable = 1U << 12,
        SternUnavailable = 1U << 13,
        HeadingRearmRequired = 1U << 14,
        VspUnavailable = 1U << 15,
    };
    struct Input {
        int8_t requested_mode = 0;
        bool enabled = false;
        bool heading_enabled = false;
        bool dp_enabled = false;
        bool heading_valid = false;
        bool heading_command_valid = false;
        bool position_valid = false;
        bool velocity_valid = false;
        bool rc_valid = false;
        bool output_ready = false;
        bool controls_neutral = true;
        bool heading_target_valid = false;
        bool position_target_valid = false;
        bool pilot_override = false;
        bool estimator_reset = false;
        bool vsp_unavailable = false;
        bool bow_unavailable = false;
        bool stern_unavailable = false;
        bool heading_rearm_required = false;
        bool configuration_valid = true; // navigation/shared heading config
        uint16_t dp_controller_reasons = 0; // from NavigationDP::Output, DP-only
        bool timing_ok = true;
        // Must be reported by the final output authority, never inferred
        // from VNAV_MODE or from a nonzero controller demand.
        bool yaw_owned = false;
        bool translation_owned = false;
        bool fault = false;
    };
    struct Status {
        State state = State::Free;
        uint16_t reason_bits = 0;
    };

    void update(const Input& input);
    const Status& status() const { return _status; }

private:
    Status _status;
};

} // namespace Vektor2
