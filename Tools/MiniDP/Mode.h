#pragma once

#include "State.h"

#include <stdint.h>

enum class MiniDP_Mode : uint8_t {
    MANUAL = 0,
    HEADING_HOLD = 1,
    DP_HOLD = 2,
    ACTUATOR_TEST = 3,
    FAILSAFE = 4,
};

enum class MiniDP_ModeReason : uint8_t {
    STARTUP = 0,
    USER_REQUEST,
    RC_REQUEST,
    MAVLINK_REQUEST,
    MANUAL_OVERRIDE,
    STATE_INVALID,
    ESTIMATOR_RESET,
    FAILSAFE_TRIGGERED,
    FAILSAFE_CLEARED,
    ACTUATOR_TEST_REQUEST,
};

enum class MiniDP_ModeReject : uint8_t {
    NONE = 0,
    YAW_INVALID,
    POSITION_INVALID,
    VELOCITY_INVALID,
    ORIGIN_INVALID,
    EKF_UNHEALTHY,
    GPS_QUALITY_INVALID,
    ACTUATOR_TEST_NOT_AUTHORIZED,
    FAILSAFE_LATCHED,
    UNSUPPORTED_MODE,
    TARGET_INVALID,
};

struct MiniDP_ModeConfig {
    float dp_hacc_max_m;
    float dp_sacc_max_m;
};

struct MiniDP_ModeRequestResult {
    bool accepted;
    bool changed;
    MiniDP_ModeReject rejection;
};

struct MiniDP_ModeTarget {
    bool yaw_valid;
    bool position_valid;
    float yaw_rad;
    float pos_n_m;
    float pos_e_m;
    uint32_t origin_id;
    uint32_t reset_counter;
    uint32_t target_id;
};

struct MiniDP_ModeTransition {
    uint32_t sequence;
    uint64_t time_us;
    MiniDP_Mode from;
    MiniDP_Mode to;
    MiniDP_ModeReason reason;
};

class MiniDP_ModeManager {
public:
    void init(uint64_t time_us);
    void set_config(const MiniDP_ModeConfig &new_config);
    void update(const MiniDP_State &state);

    MiniDP_ModeRequestResult request_mode(
        MiniDP_Mode requested,
        MiniDP_ModeReason reason,
        const MiniDP_State &state,
        bool actuator_test_authorized = false,
        bool force_target_update = false);
    MiniDP_ModeRequestResult request_dp_target(
        const MiniDP_ModeTarget &target,
        MiniDP_ModeReason reason,
        const MiniDP_State &state);

    MiniDP_ModeRequestResult manual_override(const MiniDP_State &state);

    MiniDP_Mode mode() const { return current_mode; }
    const MiniDP_ModeTarget &target() const { return current_target; }
    const MiniDP_ModeTransition &last_transition() const { return transition_record; }
    bool initialised() const { return is_initialised; }

    static const char *mode_name(MiniDP_Mode mode);
    static const char *reason_name(MiniDP_ModeReason reason);
    static const char *reject_name(MiniDP_ModeReject rejection);

private:
    MiniDP_Mode current_mode = MiniDP_Mode::MANUAL;
    MiniDP_ModeConfig cfg{10.0f, 2.0f};
    MiniDP_ModeTarget current_target{};
    MiniDP_ModeTransition transition_record{};
    uint32_t transition_sequence = 0;
    uint32_t target_sequence = 0;
    bool is_initialised = false;

    MiniDP_ModeReject gps_quality_rejection(const MiniDP_State &state) const;
    MiniDP_ModeReject entry_rejection(
        MiniDP_Mode requested,
        const MiniDP_State &state,
        bool actuator_test_authorized) const;
    void transition_to(
        MiniDP_Mode requested,
        MiniDP_ModeReason reason,
        const MiniDP_State &state);
    void latch_target(MiniDP_Mode requested, const MiniDP_State &state);
    void transition_to_dp_target(
        const MiniDP_ModeTarget &target,
        MiniDP_ModeReason reason,
        const MiniDP_State &state);
};
