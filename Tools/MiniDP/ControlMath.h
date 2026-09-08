#pragma once

#include <math.h>
#include <stdint.h>

namespace MiniDP_Math {
constexpr float pi = 3.14159265358979323846f;

// Bounded execution even for very large finite angles.
inline float wrap_pi(const float angle)
{
    return isfinite(angle) ? remainderf(angle, 2.0f * pi) : 0.0f;
}

inline uint32_t timeout_ms(const float seconds, const float fallback,
                           const float minimum, const float maximum)
{
    const float value = isfinite(seconds) ? seconds : fallback;
    return uint32_t(fminf(maximum, fmaxf(minimum, value)) * 1000.0f);
}
}
