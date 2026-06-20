#pragma once

#include "State.h"

#include <stdint.h>

enum class MiniDP_ArmGpsRequirement : int8_t {
    NONE = 0,
    FIX = 1,
    DP_READY = 2,
};

enum class MiniDP_ArmReject : uint8_t {
    NONE = 0,
    RC_KILL,
    FAILSAFE,
    GPS_FIX,
    GPS_QUALITY,
    STATE_INVALID,
};

struct MiniDP_ArmingConfig {
    bool arming_required;
    MiniDP_ArmGpsRequirement gps_requirement;
    float gps_hacc_max_m;
    float gps_sacc_max_m_s;
};

struct MiniDP_ArmResult {
    bool accepted;
    bool changed;
    MiniDP_ArmReject rejection;
};

class MiniDP_Arming {
public:
    void init();
    void set_config(const MiniDP_ArmingConfig &new_config);

    MiniDP_ArmResult arm(
        const MiniDP_State &state,
        bool rc_kill,
        bool failsafe,
        bool force);
    MiniDP_ArmResult disarm();

    bool armed() const { return is_armed; }
    bool outputs_allowed() const { return !cfg.arming_required || is_armed; }
    MiniDP_ArmReject last_rejection() const { return last_reject; }
    const MiniDP_ArmingConfig &config() const { return cfg; }

    static MiniDP_ArmingConfig default_config();
    static const char *gps_requirement_name(MiniDP_ArmGpsRequirement requirement);
    static const char *reject_name(MiniDP_ArmReject rejection);

private:
    MiniDP_ArmingConfig cfg{};
    bool is_armed = false;
    MiniDP_ArmReject last_reject = MiniDP_ArmReject::NONE;

    MiniDP_ArmReject arming_rejection(
        const MiniDP_State &state,
        bool rc_kill,
        bool failsafe,
        bool force) const;
};
