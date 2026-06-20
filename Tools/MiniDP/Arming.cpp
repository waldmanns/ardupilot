#include "Arming.h"

#include <AP_GPS/AP_GPS.h>

#include <math.h>

namespace {

float sanitize_non_negative(const float value)
{
    if (!isfinite(value) || value < 0.0f) {
        return 0.0f;
    }
    return value;
}

bool above_configured_limit(const float value, const float limit)
{
    if (limit <= 0.0f) {
        return false;
    }
    return !isfinite(value) || value > limit;
}

MiniDP_ArmGpsRequirement sanitize_gps_requirement(
    const MiniDP_ArmGpsRequirement requirement)
{
    switch (requirement) {
    case MiniDP_ArmGpsRequirement::NONE:
    case MiniDP_ArmGpsRequirement::FIX:
    case MiniDP_ArmGpsRequirement::DP_READY:
        return requirement;
    }
    return MiniDP_ArmGpsRequirement::NONE;
}

} // namespace

MiniDP_ArmingConfig MiniDP_Arming::default_config()
{
    MiniDP_ArmingConfig config{};
    config.arming_required = true;
    config.gps_requirement = MiniDP_ArmGpsRequirement::NONE;
    config.gps_hacc_max_m = 10.0f;
    config.gps_sacc_max_m_s = 2.0f;
    return config;
}

void MiniDP_Arming::init()
{
    cfg = default_config();
    is_armed = false;
    last_reject = MiniDP_ArmReject::NONE;
}

void MiniDP_Arming::set_config(const MiniDP_ArmingConfig &new_config)
{
    cfg = new_config;
    cfg.gps_requirement = sanitize_gps_requirement(cfg.gps_requirement);
    cfg.gps_hacc_max_m = sanitize_non_negative(cfg.gps_hacc_max_m);
    cfg.gps_sacc_max_m_s = sanitize_non_negative(cfg.gps_sacc_max_m_s);
}

MiniDP_ArmReject MiniDP_Arming::arming_rejection(
    const MiniDP_State &state,
    const bool rc_kill,
    const bool failsafe,
    const bool force) const
{
    if (rc_kill) {
        return MiniDP_ArmReject::RC_KILL;
    }
    if (failsafe) {
        return MiniDP_ArmReject::FAILSAFE;
    }
    if (force) {
        return MiniDP_ArmReject::NONE;
    }

    switch (cfg.gps_requirement) {
    case MiniDP_ArmGpsRequirement::NONE:
        return MiniDP_ArmReject::NONE;

    case MiniDP_ArmGpsRequirement::FIX:
    case MiniDP_ArmGpsRequirement::DP_READY:
        if (state.gps_fix_type < uint8_t(AP_GPS::GPS_OK_FIX_2D)) {
            return MiniDP_ArmReject::GPS_FIX;
        }
        if (above_configured_limit(state.gps_hacc_m, cfg.gps_hacc_max_m) ||
            above_configured_limit(state.gps_sacc_m, cfg.gps_sacc_max_m_s)) {
            return MiniDP_ArmReject::GPS_QUALITY;
        }
        break;
    }

    if (cfg.gps_requirement == MiniDP_ArmGpsRequirement::DP_READY &&
        (!state.ekf_healthy ||
         !state.yaw_valid ||
         !state.origin_valid ||
         !state.position_valid ||
         !state.velocity_valid)) {
        return MiniDP_ArmReject::STATE_INVALID;
    }

    return MiniDP_ArmReject::NONE;
}

MiniDP_ArmResult MiniDP_Arming::arm(
    const MiniDP_State &state,
    const bool rc_kill,
    const bool failsafe,
    const bool force)
{
    if (is_armed) {
        last_reject = MiniDP_ArmReject::NONE;
        return {true, false, MiniDP_ArmReject::NONE};
    }

    const MiniDP_ArmReject rejection =
        arming_rejection(state, rc_kill, failsafe, force);
    if (rejection != MiniDP_ArmReject::NONE) {
        last_reject = rejection;
        return {false, false, rejection};
    }

    is_armed = true;
    last_reject = MiniDP_ArmReject::NONE;
    return {true, true, MiniDP_ArmReject::NONE};
}

MiniDP_ArmResult MiniDP_Arming::disarm()
{
    if (!is_armed) {
        last_reject = MiniDP_ArmReject::NONE;
        return {true, false, MiniDP_ArmReject::NONE};
    }

    is_armed = false;
    last_reject = MiniDP_ArmReject::NONE;
    return {true, true, MiniDP_ArmReject::NONE};
}

const char *MiniDP_Arming::gps_requirement_name(
    const MiniDP_ArmGpsRequirement requirement)
{
    switch (requirement) {
    case MiniDP_ArmGpsRequirement::NONE:
        return "none";
    case MiniDP_ArmGpsRequirement::FIX:
        return "gps-fix";
    case MiniDP_ArmGpsRequirement::DP_READY:
        return "dp-ready";
    }
    return "unknown";
}

const char *MiniDP_Arming::reject_name(const MiniDP_ArmReject rejection)
{
    switch (rejection) {
    case MiniDP_ArmReject::NONE:
        return "none";
    case MiniDP_ArmReject::RC_KILL:
        return "rc-kill";
    case MiniDP_ArmReject::FAILSAFE:
        return "failsafe";
    case MiniDP_ArmReject::GPS_FIX:
        return "gps-fix";
    case MiniDP_ArmReject::GPS_QUALITY:
        return "gps-quality";
    case MiniDP_ArmReject::STATE_INVALID:
        return "state-invalid";
    }
    return "unknown";
}
