#pragma once

#include <stdint.h>

enum class MiniDP_ControlOwner : uint8_t {
    NONE = 0,
    RC,
    MAVLINK_MANUAL,
    MAVLINK_TARGET,
    FAILSAFE,
    TEST,
};

enum class MiniDP_TargetLinkLossAction : uint8_t {
    FAILSAFE = 0,
    HOLD_TARGET,
    FALLBACK_TO_RC,
};

enum class MiniDP_AuthorityReason : uint8_t {
    STARTUP = 0,
    RELEASED,
    RC_REQUEST,
    MAVLINK_MANUAL_REQUEST,
    MAVLINK_TARGET_REQUEST,
    RC_TAKEOVER,
    RC_LINK_LOSS,
    RC_LOSS_MAVLINK_TAKEOVER,
    MAVLINK_LINK_LOSS,
    MAVLINK_AUTH_REVOKED,
    MAVLINK_TARGET_HOLD,
    MAVLINK_LINK_RESTORED,
    FAILSAFE_TRIGGERED,
    FAILSAFE_CLEARED,
    RC_KILL,
    TEST_REQUEST,
    TEST_AUTH_REVOKED,
    MAVLINK_MANUAL_TIMEOUT,
};

enum class MiniDP_AuthorityReject : uint8_t {
    NONE = 0,
    RC_UNHEALTHY,
    MAVLINK_UNHEALTHY,
    MAVLINK_MANUAL_UNAUTHORIZED,
    MAVLINK_TARGET_UNAUTHORIZED,
    RC_TAKEOVER_DISABLED,
    TEST_NOT_AUTHORIZED,
    TEST_ACTIVE,
    FAILSAFE_LATCHED,
    KILL_ACTIVE,
    FAILSAFE_ACTIVE,
    UNSUPPORTED_OWNER,
};

struct MiniDP_AuthorityPolicy {
    bool rc_takeover_from_mavlink;
    bool rc_fallback_on_mavlink_loss;
    bool mavlink_manual_takeover_on_rc_loss;
    MiniDP_TargetLinkLossAction target_link_loss_action;
};

struct MiniDP_AuthorityStatus {
    uint64_t time_us;
    bool rc_healthy;
    bool mavlink_healthy;
    bool mavlink_manual_authorized;
    bool mavlink_target_authorized;
    bool actuator_test_authorized;
    bool rc_kill;
    bool failsafe_active;
};

struct MiniDP_AuthorityRequestResult {
    bool accepted;
    bool changed;
    MiniDP_AuthorityReject rejection;
};

struct MiniDP_AuthorityTransition {
    uint32_t sequence;
    uint64_t time_us;
    MiniDP_ControlOwner from;
    MiniDP_ControlOwner to;
    MiniDP_AuthorityReason reason;
};

class MiniDP_AuthorityArbiter {
public:
    void init(uint64_t time_us);
    void set_policy(const MiniDP_AuthorityPolicy &new_policy) { policy = new_policy; }
    void update(const MiniDP_AuthorityStatus &status);

    MiniDP_AuthorityRequestResult request_owner(
        MiniDP_ControlOwner requested,
        MiniDP_AuthorityReason reason,
        const MiniDP_AuthorityStatus &status);
    MiniDP_AuthorityRequestResult clear_failsafe(
        const MiniDP_AuthorityStatus &status);

    MiniDP_ControlOwner owner() const { return current_owner; }
    const MiniDP_AuthorityTransition &last_transition() const { return transition_record; }
    bool initialised() const { return is_initialised; }

    static const char *owner_name(MiniDP_ControlOwner owner);
    static const char *reason_name(MiniDP_AuthorityReason reason);
    static const char *reject_name(MiniDP_AuthorityReject rejection);

private:
    MiniDP_ControlOwner current_owner = MiniDP_ControlOwner::NONE;
    MiniDP_AuthorityPolicy policy{};
    MiniDP_AuthorityTransition transition_record{};
    uint32_t transition_sequence = 0;
    bool target_link_loss_recorded = false;
    bool is_initialised = false;

    MiniDP_AuthorityReject request_rejection(
        MiniDP_ControlOwner requested,
        const MiniDP_AuthorityStatus &status) const;
    void transition_to(
        MiniDP_ControlOwner requested,
        MiniDP_AuthorityReason reason,
        uint64_t time_us);
    void enter_failsafe(
        MiniDP_AuthorityReason reason,
        uint64_t time_us);
};
