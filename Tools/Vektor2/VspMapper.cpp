#include "VspMapper.h"

#include <math.h>
#include <stdlib.h>

namespace Vektor2 {
namespace {
constexpr float radians_per_degree = 0.01745329251994329577f;
VspMapper::Estimate invalid_estimate()
{
    return {false, {NAN, NAN}, NAN, NAN};
}
}

bool VspMapper::valid_config(const Config& c)
{
    return (c.rotation == Rotation::CW || c.rotation == Rotation::CCW) &&
        isfinite(c.installation_deg) && c.installation_deg >= 0 && c.installation_deg < 360 &&
        c.radius > 0 && c.x_center - int32_t(c.radius) >= 1000 &&
        c.x_center + int32_t(c.radius) <= 2000 &&
        c.y_center - int32_t(c.radius) >= 1000 && c.y_center + int32_t(c.radius) <= 2000;
}

VspMapper::Vector VspMapper::from_polar(float direction_deg, float intensity)
{
    if (!isfinite(direction_deg) || !isfinite(intensity) || intensity < 0) {
        return {NAN, NAN};
    }
    const float angle = fmodf(direction_deg, 360) * radians_per_degree;
    return {intensity * cosf(angle), intensity * sinf(angle)};
}

VspMapper::Estimate VspMapper::estimate(const Config& c, uint16_t x_us, uint16_t y_us)
{
    if (!valid_config(c)) {
        return invalid_estimate();
    }
    const int32_t dx = int32_t(x_us) - c.x_center;
    const int32_t dy = int32_t(y_us) - c.y_center;
    // Legacy ring outputs truncate absolute pulse widths to integer us. An
    // in-ring point can therefore land just outside the disk. Accept at most
    // one us of quantization on each axis, never arbitrary out-of-ring rods.
    const int32_t inner_x = abs(dx) > 0 ? abs(dx) - 1 : 0;
    const int32_t inner_y = abs(dy) > 0 ? abs(dy) - 1 : 0;
    if (x_us < 1000 || x_us > 2000 || y_us < 1000 || y_us > 2000 ||
        int64_t(inner_x)*inner_x + int64_t(inner_y)*inner_y > int64_t(c.radius)*c.radius) {
        return invalid_estimate();
    }
    const float distance = hypotf(float(dx), float(dy));
    const float normalization = fmaxf(float(c.radius), distance);
    const float sign = c.rotation == Rotation::CW ? 1 : -1;
    const float forward = sign * dx / normalization;
    const float starboard = sign * dy / normalization;
    const float angle = c.installation_deg * radians_per_degree;
    const Vector body {cosf(angle)*forward - sinf(angle)*starboard,
                       sinf(angle)*forward + cosf(angle)*starboard};
    float direction = NAN;
    const float intensity = fminf(1, distance / c.radius);
    if (dx != 0 || dy != 0) {
        direction = atan2f(body.starboard, body.forward) / radians_per_degree;
        if (direction < 0) {
            direction += 360;
        }
        if (direction >= 360) {
            direction = 0;
        }
    }
    return {true, body, direction, intensity};
}

VspMapper::Result VspMapper::map(const Config& c, Vector demand)
{
    Result result {false, false, 0, 0, invalid_estimate()};
    if (!valid_config(c) || !isfinite(demand.forward) || !isfinite(demand.starboard)) {
        return result;
    }
    // Normalize before hypot to avoid overflow for finite extreme demands.
    const float peak = fmaxf(fabsf(demand.forward), fabsf(demand.starboard));
    if (peak > 1) {
        demand.forward /= peak;
        demand.starboard /= peak;
        result.limited = true;
    }
    const float length = hypotf(demand.forward, demand.starboard);
    if (length > 1) {
        demand.forward /= length;
        demand.starboard /= length;
        result.limited = true;
    }
    const float angle = c.installation_deg * radians_per_degree;
    const float sign = c.rotation == Rotation::CW ? 1 : -1;
    int32_t dx = lroundf(sign * c.radius * (cosf(angle)*demand.forward + sinf(angle)*demand.starboard));
    int32_t dy = lroundf(sign * c.radius * (-sinf(angle)*demand.forward + cosf(angle)*demand.starboard));
    // Nearest-integer rounding can leave the disk. Move the larger coordinate
    // one pulse toward neutral until the quantized pair is feasible.
    while (dx*dx + dy*dy > int32_t(c.radius)*c.radius) {
        if (abs(dx) >= abs(dy)) {
            dx += dx > 0 ? -1 : 1;
        } else {
            dy += dy > 0 ? -1 : 1;
        }
        result.limited = true;
    }
    result.x_us = c.x_center + dx;
    result.y_us = c.y_center + dy;
    result.applied = estimate(c, result.x_us, result.y_us);
    result.valid = result.applied.valid;
    return result;
}
} // namespace Vektor2
