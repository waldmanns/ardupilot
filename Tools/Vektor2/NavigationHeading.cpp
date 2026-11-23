#include "NavigationHeading.h"

#include <cmath>

namespace Vektor2 {
namespace {
static const float rad_per_deg = 0.01745329251994329577f;
static const float deg_per_rad = 57.2957795130823209f;
float wrap_360(float degrees)
{
    float wrapped = std::fmod(degrees, 360.0f);
    return wrapped < 0 ? wrapped + 360.0f : wrapped;
}
float clamp(float value, float limit)
{
    return value < -limit ? -limit : (value > limit ? limit : value);
}
}

bool NavigationHeading::config_valid(const Config& c)
{
    return std::isfinite(c.kp) && c.kp > 0 &&
           std::isfinite(c.kd) && c.kd >= 0 &&
           std::isfinite(c.deadband_deg) && c.deadband_deg >= 0 &&
           std::isfinite(c.max_yaw_percent) && c.max_yaw_percent > 0 &&
           c.max_yaw_percent <= 100;
}

bool NavigationHeading::capture(float heading_deg)
{
    if (!std::isfinite(heading_deg)) { return false; }
    _target_deg = wrap_360(heading_deg);
    _target_valid = true;
    return true;
}

void NavigationHeading::clear_target()
{
    _target_valid = false;
    _output = {};
}

bool NavigationHeading::earth_yaw_rate(float q, float r, float roll_deg,
                                       float pitch_deg, float& result)
{
    if (!std::isfinite(q) || !std::isfinite(r) || !std::isfinite(roll_deg) ||
        !std::isfinite(pitch_deg) || std::fabs(pitch_deg) >= 75.0f) {
        return false;
    }
    const float roll = roll_deg * rad_per_deg;
    const float pitch = pitch_deg * rad_per_deg;
    result = ((std::sin(roll) * q + std::cos(roll) * r) / std::cos(pitch)) * deg_per_rad;
    return std::isfinite(result);
}

void NavigationHeading::update(float heading_deg, float yaw_rate_deg_s,
                               bool heading_valid, const Config& config,
                               bool controller_enabled)
{
    Output out{};
    out.target_valid = _target_valid;
    out.target_deg = _target_deg;
    out.actual_valid = heading_valid && std::isfinite(heading_deg);
    if (out.actual_valid) { out.actual_deg = wrap_360(heading_deg); }
    if (_target_valid && out.actual_valid) {
        out.error_deg = wrap_360(_target_deg - out.actual_deg + 180.0f) - 180.0f;
    }
    if (!controller_enabled || !_target_valid || !out.actual_valid ||
        !std::isfinite(yaw_rate_deg_s) || !config_valid(config)) {
        _output = out;
        return;
    }
    const float position_error = std::fabs(out.error_deg) <= config.deadband_deg ? 0 : out.error_deg;
    const float requested = config.kp * position_error - config.kd * yaw_rate_deg_s;
    if (!std::isfinite(requested)) { _output = out; return; }
    out.yaw_demand = clamp(requested, config.max_yaw_percent * 0.01f);
    out.valid = true;
    _output = out;
}

} // namespace Vektor2
