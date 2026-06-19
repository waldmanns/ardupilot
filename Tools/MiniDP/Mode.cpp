#include "Mode.h"

const char *MiniDP_ModeManager::mode_name(const MiniDP_Mode mode)
{
    switch (mode) {
    case MiniDP_Mode::MANUAL:
        return "MANUAL";
    case MiniDP_Mode::HEADING_HOLD:
        return "HEADING_HOLD";
    case MiniDP_Mode::DP_HOLD:
        return "DP_HOLD";
    case MiniDP_Mode::ACTUATOR_TEST:
        return "ACTUATOR_TEST";
    case MiniDP_Mode::FAILSAFE:
        return "FAILSAFE";
    }
    return "UNKNOWN";
}

const char *MiniDP_ModeManager::reason_name(const MiniDP_ModeReason reason)
{
    switch (reason) {
    case MiniDP_ModeReason::STARTUP:
        return "startup";
    case MiniDP_ModeReason::USER_REQUEST:
        return "user-request";
    case MiniDP_ModeReason::RC_REQUEST:
        return "rc-request";
    case MiniDP_ModeReason::MAVLINK_REQUEST:
        return "mavlink-request";
    case MiniDP_ModeReason::MANUAL_OVERRIDE:
        return "manual-override";
    case MiniDP_ModeReason::STATE_INVALID:
        return "state-invalid";
    case MiniDP_ModeReason::ESTIMATOR_RESET:
        return "estimator-reset";
    case MiniDP_ModeReason::FAILSAFE_TRIGGERED:
        return "failsafe-triggered";
    case MiniDP_ModeReason::FAILSAFE_CLEARED:
        return "failsafe-cleared";
    case MiniDP_ModeReason::ACTUATOR_TEST_REQUEST:
        return "actuator-test";
    }
    return "unknown";
}

const char *MiniDP_ModeManager::reject_name(const MiniDP_ModeReject rejection)
{
    switch (rejection) {
    case MiniDP_ModeReject::NONE:
        return "none";
    case MiniDP_ModeReject::YAW_INVALID:
        return "yaw-invalid";
    case MiniDP_ModeReject::POSITION_INVALID:
        return "position-invalid";
    case MiniDP_ModeReject::VELOCITY_INVALID:
        return "velocity-invalid";
    case MiniDP_ModeReject::ORIGIN_INVALID:
        return "origin-invalid";
    case MiniDP_ModeReject::EKF_UNHEALTHY:
        return "ekf-unhealthy";
    case MiniDP_ModeReject::ACTUATOR_TEST_NOT_AUTHORIZED:
        return "actuator-test-not-authorized";
    case MiniDP_ModeReject::FAILSAFE_LATCHED:
        return "failsafe-latched";
    case MiniDP_ModeReject::UNSUPPORTED_MODE:
        return "unsupported-mode";
    }
    return "unknown";
}

void MiniDP_ModeManager::init(const uint64_t time_us)
{
    current_mode = MiniDP_Mode::MANUAL;
    current_target = {};
    transition_sequence = 1;
    target_sequence = 0;
    transition_record = {
        transition_sequence,
        time_us,
        MiniDP_Mode::MANUAL,
        MiniDP_Mode::MANUAL,
        MiniDP_ModeReason::STARTUP,
    };
    is_initialised = true;
}

MiniDP_ModeReject MiniDP_ModeManager::entry_rejection(
    const MiniDP_Mode requested,
    const MiniDP_State &state,
    const bool actuator_test_authorized) const
{
    switch (requested) {
    case MiniDP_Mode::MANUAL:
    case MiniDP_Mode::FAILSAFE:
        return MiniDP_ModeReject::NONE;

    case MiniDP_Mode::HEADING_HOLD:
        return state.yaw_valid ?
            MiniDP_ModeReject::NONE :
            MiniDP_ModeReject::YAW_INVALID;

    case MiniDP_Mode::DP_HOLD:
        if (!state.ekf_healthy) {
            return MiniDP_ModeReject::EKF_UNHEALTHY;
        }
        if (!state.yaw_valid) {
            return MiniDP_ModeReject::YAW_INVALID;
        }
        if (!state.origin_valid) {
            return MiniDP_ModeReject::ORIGIN_INVALID;
        }
        if (!state.position_valid) {
            return MiniDP_ModeReject::POSITION_INVALID;
        }
        if (!state.velocity_valid) {
            return MiniDP_ModeReject::VELOCITY_INVALID;
        }
        return MiniDP_ModeReject::NONE;

    case MiniDP_Mode::ACTUATOR_TEST:
        return actuator_test_authorized ?
            MiniDP_ModeReject::NONE :
            MiniDP_ModeReject::ACTUATOR_TEST_NOT_AUTHORIZED;
    }

    return MiniDP_ModeReject::UNSUPPORTED_MODE;
}

void MiniDP_ModeManager::latch_target(
    const MiniDP_Mode requested,
    const MiniDP_State &state)
{
    current_target = {};

    if (requested == MiniDP_Mode::HEADING_HOLD ||
        requested == MiniDP_Mode::DP_HOLD) {
        current_target.yaw_valid = true;
        current_target.yaw_rad = state.yaw_rad;
        current_target.origin_id = state.origin_id;
        current_target.reset_counter = state.reset_counter;
        current_target.target_id = ++target_sequence;
    }

    if (requested == MiniDP_Mode::DP_HOLD) {
        current_target.position_valid = true;
        current_target.pos_n_m = state.pos_n_m;
        current_target.pos_e_m = state.pos_e_m;
    }
}

void MiniDP_ModeManager::transition_to(
    const MiniDP_Mode requested,
    const MiniDP_ModeReason reason,
    const MiniDP_State &state)
{
    const MiniDP_Mode previous = current_mode;
    current_mode = requested;
    latch_target(requested, state);
    transition_record = {
        ++transition_sequence,
        state.time_us,
        previous,
        requested,
        reason,
    };
}

MiniDP_ModeRequestResult MiniDP_ModeManager::request_mode(
    const MiniDP_Mode requested,
    const MiniDP_ModeReason reason,
    const MiniDP_State &state,
    const bool actuator_test_authorized,
    const bool force_target_update)
{
    if (!is_initialised) {
        init(state.time_us);
    }

    if (current_mode == MiniDP_Mode::FAILSAFE &&
        requested != MiniDP_Mode::FAILSAFE) {
        const bool clearing_to_manual =
            reason == MiniDP_ModeReason::FAILSAFE_CLEARED &&
            requested == MiniDP_Mode::MANUAL;
        if (!clearing_to_manual) {
            return {false, false, MiniDP_ModeReject::FAILSAFE_LATCHED};
        }
    }

    const MiniDP_ModeReject rejection =
        entry_rejection(requested, state, actuator_test_authorized);
    if (rejection != MiniDP_ModeReject::NONE) {
        return {false, false, rejection};
    }

    if (requested == current_mode) {
        if (force_target_update &&
            requested == MiniDP_Mode::DP_HOLD) {
            transition_to(requested, reason, state);
            return {true, true, MiniDP_ModeReject::NONE};
        }
        return {true, false, MiniDP_ModeReject::NONE};
    }

    transition_to(requested, reason, state);
    return {true, true, MiniDP_ModeReject::NONE};
}

MiniDP_ModeRequestResult MiniDP_ModeManager::manual_override(
    const MiniDP_State &state)
{
    return request_mode(
        MiniDP_Mode::MANUAL,
        MiniDP_ModeReason::MANUAL_OVERRIDE,
        state);
}

void MiniDP_ModeManager::update(const MiniDP_State &state)
{
    if (!is_initialised) {
        init(state.time_us);
    }

    switch (current_mode) {
    case MiniDP_Mode::MANUAL:
    case MiniDP_Mode::ACTUATOR_TEST:
    case MiniDP_Mode::FAILSAFE:
        return;

    case MiniDP_Mode::HEADING_HOLD:
        if (!state.yaw_valid) {
            transition_to(
                MiniDP_Mode::FAILSAFE,
                MiniDP_ModeReason::FAILSAFE_TRIGGERED,
                state);
        } else if (state.reset_counter != current_target.reset_counter) {
            transition_to(
                MiniDP_Mode::HEADING_HOLD,
                MiniDP_ModeReason::ESTIMATOR_RESET,
                state);
        }
        return;

    case MiniDP_Mode::DP_HOLD:
        if (!state.yaw_valid) {
            transition_to(
                MiniDP_Mode::FAILSAFE,
                MiniDP_ModeReason::FAILSAFE_TRIGGERED,
                state);
            return;
        }

        if (state.origin_id != current_target.origin_id ||
            state.reset_counter != current_target.reset_counter) {
            transition_to(
                MiniDP_Mode::HEADING_HOLD,
                MiniDP_ModeReason::ESTIMATOR_RESET,
                state);
            return;
        }

        if (!state.ekf_healthy ||
            !state.origin_valid ||
            !state.position_valid ||
            !state.velocity_valid) {
            transition_to(
                MiniDP_Mode::HEADING_HOLD,
                MiniDP_ModeReason::STATE_INVALID,
                state);
        }
        return;
    }
}
