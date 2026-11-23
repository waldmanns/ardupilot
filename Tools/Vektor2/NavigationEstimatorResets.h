#pragma once

#include <stdint.h>

namespace Vektor2 {

// AHRS/EKF supplies these timestamps. First observation establishes a
// baseline; subsequent nonzero changes invalidate the captured DP targets.
// This tracks resets only: it does not estimate position or detect drift.
class NavigationEstimatorResets {
public:
    bool update(uint32_t yaw_ms, uint32_t pos_ne_ms, uint32_t vel_ne_ms)
    {
        const bool changed = _initialized &&
            ((yaw_ms != 0 && yaw_ms != _yaw_ms) ||
             (pos_ne_ms != 0 && pos_ne_ms != _pos_ne_ms) ||
             (vel_ne_ms != 0 && vel_ne_ms != _vel_ne_ms));
        _yaw_ms = yaw_ms;
        _pos_ne_ms = pos_ne_ms;
        _vel_ne_ms = vel_ne_ms;
        _initialized = true;
        return changed;
    }
private:
    bool _initialized = false;
    uint32_t _yaw_ms = 0;
    uint32_t _pos_ne_ms = 0;
    uint32_t _vel_ne_ms = 0;
};

} // namespace Vektor2
