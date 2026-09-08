#include "Authority.h"

const char *MiniDP_AuthorityArbiter::owner_name(const MiniDP_ControlOwner owner)
{
    switch (owner) {
    case MiniDP_ControlOwner::NONE:
        return "NONE";
    case MiniDP_ControlOwner::RC:
        return "RC";
    case MiniDP_ControlOwner::MAVLINK_MANUAL:
        return "MAVLINK_MANUAL";
    case MiniDP_ControlOwner::MAVLINK_TARGET:
        return "MAVLINK_TARGET";
    case MiniDP_ControlOwner::FAILSAFE:
        return "FAILSAFE";
    case MiniDP_ControlOwner::TEST:
        return "TEST";
    }
    return "UNKNOWN";
}

const char *MiniDP_AuthorityArbiter::reason_name(
    const MiniDP_AuthorityReason reason)
{
    switch (reason) {
    case MiniDP_AuthorityReason::STARTUP:
        return "startup";
    case MiniDP_AuthorityReason::RELEASED:
        return "released";
    case MiniDP_AuthorityReason::RC_REQUEST:
        return "rc-request";
    case MiniDP_AuthorityReason::MAVLINK_MANUAL_REQUEST:
        return "mavlink-manual-request";
    case MiniDP_AuthorityReason::MAVLINK_TARGET_REQUEST:
        return "mavlink-target-request";
    case MiniDP_AuthorityReason::RC_TAKEOVER:
        return "rc-takeover";
    case MiniDP_AuthorityReason::RC_LINK_LOSS:
        return "rc-link-loss";
    case MiniDP_AuthorityReason::RC_LOSS_MAVLINK_TAKEOVER:
        return "rc-loss-mavlink-takeover";
    case MiniDP_AuthorityReason::MAVLINK_LINK_LOSS:
        return "mavlink-link-loss";
    case MiniDP_AuthorityReason::MAVLINK_AUTH_REVOKED:
        return "mavlink-auth-revoked";
    case MiniDP_AuthorityReason::MAVLINK_TARGET_HOLD:
        return "mavlink-target-hold";
    case MiniDP_AuthorityReason::MAVLINK_LINK_RESTORED:
        return "mavlink-link-restored";
    case MiniDP_AuthorityReason::FAILSAFE_TRIGGERED:
        return "failsafe-triggered";
    case MiniDP_AuthorityReason::FAILSAFE_CLEARED:
        return "failsafe-cleared";
    case MiniDP_AuthorityReason::RC_KILL:
        return "rc-kill";
    case MiniDP_AuthorityReason::TEST_REQUEST:
        return "test-request";
    case MiniDP_AuthorityReason::TEST_AUTH_REVOKED:
        return "test-auth-revoked";
    case MiniDP_AuthorityReason::MAVLINK_MANUAL_TIMEOUT:
        return "mavlink-manual-timeout";
    }
    return "unknown";
}

const char *MiniDP_AuthorityArbiter::reject_name(
    const MiniDP_AuthorityReject rejection)
{
    switch (rejection) {
    case MiniDP_AuthorityReject::NONE:
        return "none";
    case MiniDP_AuthorityReject::RC_UNHEALTHY:
        return "rc-unhealthy";
    case MiniDP_AuthorityReject::MAVLINK_UNHEALTHY:
        return "mavlink-unhealthy";
    case MiniDP_AuthorityReject::MAVLINK_MANUAL_UNAUTHORIZED:
        return "mavlink-manual-unauthorized";
    case MiniDP_AuthorityReject::MAVLINK_TARGET_UNAUTHORIZED:
        return "mavlink-target-unauthorized";
    case MiniDP_AuthorityReject::RC_TAKEOVER_DISABLED:
        return "rc-takeover-disabled";
    case MiniDP_AuthorityReject::TEST_NOT_AUTHORIZED:
        return "test-not-authorized";
    case MiniDP_AuthorityReject::TEST_ACTIVE:
        return "test-active";
    case MiniDP_AuthorityReject::FAILSAFE_LATCHED:
        return "failsafe-latched";
    case MiniDP_AuthorityReject::KILL_ACTIVE:
        return "kill-active";
    case MiniDP_AuthorityReject::FAILSAFE_ACTIVE:
        return "failsafe-active";
    case MiniDP_AuthorityReject::UNSUPPORTED_OWNER:
        return "unsupported-owner";
    }
    return "unknown";
}

void MiniDP_AuthorityArbiter::init(const uint64_t time_us)
{
    current_owner = MiniDP_ControlOwner::NONE;
    transition_sequence = 1;
    target_link_loss_recorded = false;
    transition_record = {
        transition_sequence,
        time_us,
        MiniDP_ControlOwner::NONE,
        MiniDP_ControlOwner::NONE,
        MiniDP_AuthorityReason::STARTUP,
    };
    is_initialised = true;
}

void MiniDP_AuthorityArbiter::transition_to(
    const MiniDP_ControlOwner requested,
    const MiniDP_AuthorityReason reason,
    const uint64_t time_us)
{
    const MiniDP_ControlOwner previous = current_owner;
    current_owner = requested;
    if (requested != MiniDP_ControlOwner::MAVLINK_TARGET) {
        target_link_loss_recorded = false;
    }
    transition_record = {
        ++transition_sequence,
        time_us,
        previous,
        requested,
        reason,
    };
}

void MiniDP_AuthorityArbiter::enter_failsafe(
    const MiniDP_AuthorityReason reason,
    const uint64_t time_us)
{
    if (current_owner != MiniDP_ControlOwner::FAILSAFE) {
        transition_to(MiniDP_ControlOwner::FAILSAFE, reason, time_us);
    }
}

MiniDP_AuthorityReject MiniDP_AuthorityArbiter::request_rejection(
    const MiniDP_ControlOwner requested,
    const MiniDP_AuthorityStatus &status) const
{
    if (status.rc_kill) {
        return MiniDP_AuthorityReject::KILL_ACTIVE;
    }
    if (status.failsafe_active) {
        return MiniDP_AuthorityReject::FAILSAFE_ACTIVE;
    }
    if (current_owner == MiniDP_ControlOwner::FAILSAFE) {
        return MiniDP_AuthorityReject::FAILSAFE_LATCHED;
    }
    if (current_owner == MiniDP_ControlOwner::TEST &&
        requested != MiniDP_ControlOwner::NONE &&
        requested != MiniDP_ControlOwner::TEST &&
        requested != MiniDP_ControlOwner::FAILSAFE) {
        return MiniDP_AuthorityReject::TEST_ACTIVE;
    }

    switch (requested) {
    case MiniDP_ControlOwner::NONE:
    case MiniDP_ControlOwner::FAILSAFE:
        return MiniDP_AuthorityReject::NONE;

    case MiniDP_ControlOwner::RC:
        if (!status.rc_healthy) {
            return MiniDP_AuthorityReject::RC_UNHEALTHY;
        }
        if ((current_owner == MiniDP_ControlOwner::MAVLINK_MANUAL ||
             current_owner == MiniDP_ControlOwner::MAVLINK_TARGET) &&
            !policy.rc_takeover_from_mavlink) {
            return MiniDP_AuthorityReject::RC_TAKEOVER_DISABLED;
        }
        return MiniDP_AuthorityReject::NONE;

    case MiniDP_ControlOwner::MAVLINK_MANUAL:
        if (!status.mavlink_manual_authorized) {
            return MiniDP_AuthorityReject::MAVLINK_MANUAL_UNAUTHORIZED;
        }
        return status.mavlink_healthy ?
            MiniDP_AuthorityReject::NONE :
            MiniDP_AuthorityReject::MAVLINK_UNHEALTHY;

    case MiniDP_ControlOwner::MAVLINK_TARGET:
        if (!status.mavlink_target_authorized) {
            return MiniDP_AuthorityReject::MAVLINK_TARGET_UNAUTHORIZED;
        }
        return status.mavlink_healthy ?
            MiniDP_AuthorityReject::NONE :
            MiniDP_AuthorityReject::MAVLINK_UNHEALTHY;

    case MiniDP_ControlOwner::TEST:
        return status.actuator_test_authorized ?
            MiniDP_AuthorityReject::NONE :
            MiniDP_AuthorityReject::TEST_NOT_AUTHORIZED;
    }

    return MiniDP_AuthorityReject::UNSUPPORTED_OWNER;
}

MiniDP_AuthorityRequestResult MiniDP_AuthorityArbiter::request_owner(
    const MiniDP_ControlOwner requested,
    const MiniDP_AuthorityReason reason,
    const MiniDP_AuthorityStatus &status)
{
    if (!is_initialised) {
        init(status.time_us);
    }

    if (status.rc_kill) {
        const bool changed = current_owner != MiniDP_ControlOwner::FAILSAFE;
        enter_failsafe(MiniDP_AuthorityReason::RC_KILL, status.time_us);
        return {false, changed, MiniDP_AuthorityReject::KILL_ACTIVE};
    }
    if (status.failsafe_active) {
        const bool changed = current_owner != MiniDP_ControlOwner::FAILSAFE;
        enter_failsafe(
            MiniDP_AuthorityReason::FAILSAFE_TRIGGERED,
            status.time_us);
        return {false, changed, MiniDP_AuthorityReject::FAILSAFE_ACTIVE};
    }

    const MiniDP_AuthorityReject rejection =
        request_rejection(requested, status);
    if (rejection != MiniDP_AuthorityReject::NONE) {
        return {false, false, rejection};
    }

    if (requested == current_owner) {
        return {true, false, MiniDP_AuthorityReject::NONE};
    }

    transition_to(requested, reason, status.time_us);
    return {true, true, MiniDP_AuthorityReject::NONE};
}

MiniDP_AuthorityRequestResult MiniDP_AuthorityArbiter::clear_failsafe(
    const MiniDP_AuthorityStatus &status)
{
    if (!is_initialised) {
        init(status.time_us);
    }
    if (status.rc_kill) {
        const bool changed = current_owner != MiniDP_ControlOwner::FAILSAFE;
        enter_failsafe(MiniDP_AuthorityReason::RC_KILL, status.time_us);
        return {false, changed, MiniDP_AuthorityReject::KILL_ACTIVE};
    }
    if (status.failsafe_active) {
        const bool changed = current_owner != MiniDP_ControlOwner::FAILSAFE;
        enter_failsafe(
            MiniDP_AuthorityReason::FAILSAFE_TRIGGERED,
            status.time_us);
        return {false, changed, MiniDP_AuthorityReject::FAILSAFE_ACTIVE};
    }
    if (current_owner != MiniDP_ControlOwner::FAILSAFE) {
        return {true, false, MiniDP_AuthorityReject::NONE};
    }

    transition_to(
        MiniDP_ControlOwner::NONE,
        MiniDP_AuthorityReason::FAILSAFE_CLEARED,
        status.time_us);
    return {true, true, MiniDP_AuthorityReject::NONE};
}

void MiniDP_AuthorityArbiter::update(const MiniDP_AuthorityStatus &status)
{
    if (!is_initialised) {
        init(status.time_us);
    }

    if (status.rc_kill) {
        enter_failsafe(MiniDP_AuthorityReason::RC_KILL, status.time_us);
        return;
    }
    if (status.failsafe_active) {
        enter_failsafe(
            MiniDP_AuthorityReason::FAILSAFE_TRIGGERED,
            status.time_us);
        return;
    }

    switch (current_owner) {
    case MiniDP_ControlOwner::NONE:
    case MiniDP_ControlOwner::FAILSAFE:
        return;

    case MiniDP_ControlOwner::RC:
        if (status.rc_healthy) {
            return;
        }
        if (policy.mavlink_manual_takeover_on_rc_loss &&
            status.mavlink_healthy &&
            status.mavlink_manual_authorized) {
            transition_to(
                MiniDP_ControlOwner::MAVLINK_MANUAL,
                MiniDP_AuthorityReason::RC_LOSS_MAVLINK_TAKEOVER,
                status.time_us);
        } else {
            enter_failsafe(
                MiniDP_AuthorityReason::RC_LINK_LOSS,
                status.time_us);
        }
        return;

    case MiniDP_ControlOwner::MAVLINK_MANUAL:
        if (status.mavlink_healthy &&
            status.mavlink_manual_authorized) {
            return;
        }
        if (policy.rc_fallback_on_mavlink_loss && status.rc_healthy) {
            transition_to(
                MiniDP_ControlOwner::RC,
                status.mavlink_healthy ?
                    MiniDP_AuthorityReason::MAVLINK_AUTH_REVOKED :
                    MiniDP_AuthorityReason::MAVLINK_LINK_LOSS,
                status.time_us);
        } else {
            enter_failsafe(
                status.mavlink_healthy ?
                    MiniDP_AuthorityReason::MAVLINK_AUTH_REVOKED :
                    MiniDP_AuthorityReason::MAVLINK_LINK_LOSS,
                status.time_us);
        }
        return;

    case MiniDP_ControlOwner::MAVLINK_TARGET:
        if (!status.mavlink_target_authorized) {
            if (policy.rc_fallback_on_mavlink_loss && status.rc_healthy) {
                transition_to(
                    MiniDP_ControlOwner::RC,
                    MiniDP_AuthorityReason::MAVLINK_AUTH_REVOKED,
                    status.time_us);
            } else {
                enter_failsafe(
                    MiniDP_AuthorityReason::MAVLINK_AUTH_REVOKED,
                    status.time_us);
            }
            return;
        }

        if (status.mavlink_healthy) {
            if (target_link_loss_recorded) {
                transition_to(
                    MiniDP_ControlOwner::MAVLINK_TARGET,
                    MiniDP_AuthorityReason::MAVLINK_LINK_RESTORED,
                    status.time_us);
                target_link_loss_recorded = false;
            }
            return;
        }

        switch (policy.target_link_loss_action) {
        case MiniDP_TargetLinkLossAction::FAILSAFE:
            enter_failsafe(
                MiniDP_AuthorityReason::MAVLINK_LINK_LOSS,
                status.time_us);
            return;

        case MiniDP_TargetLinkLossAction::HOLD_TARGET:
            if (!target_link_loss_recorded) {
                transition_to(
                    MiniDP_ControlOwner::MAVLINK_TARGET,
                    MiniDP_AuthorityReason::MAVLINK_TARGET_HOLD,
                    status.time_us);
                target_link_loss_recorded = true;
            }
            return;

        case MiniDP_TargetLinkLossAction::FALLBACK_TO_RC:
            if (status.rc_healthy) {
                transition_to(
                    MiniDP_ControlOwner::RC,
                    MiniDP_AuthorityReason::MAVLINK_LINK_LOSS,
                    status.time_us);
            } else {
                enter_failsafe(
                    MiniDP_AuthorityReason::MAVLINK_LINK_LOSS,
                    status.time_us);
            }
            return;
        }
        return;

    case MiniDP_ControlOwner::TEST:
        if (!status.actuator_test_authorized) {
            transition_to(
                MiniDP_ControlOwner::NONE,
                MiniDP_AuthorityReason::TEST_AUTH_REVOKED,
                status.time_us);
        }
        return;
    }
}
