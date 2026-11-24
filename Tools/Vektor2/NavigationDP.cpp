#include "NavigationDP.h"
#include "NavigationSupervisor.h"

#include <cmath>

namespace Vektor2 {
namespace {
static const double rad_per_e7 = 3.14159265358979323846 / 1800000000.0;
static const double earth_radius_m = 6371000.0;
static const float rad_per_deg = 0.01745329251994329577f;
static const float deg_per_rad = 57.2957795130823209f;

float bound(float v, float low, float high)
{
    return v < low ? low : (v > high ? high : v);
}

bool dp_config_valid(const NavigationDP::Config& c)
{
    return std::isfinite(c.position_kp) && c.position_kp > 0 &&
           std::isfinite(c.velocity_kp) && c.velocity_kp > 0 &&
           std::isfinite(c.max_speed_ms) && c.max_speed_ms > 0 &&
           std::isfinite(c.max_accel_ms2) && c.max_accel_ms2 > 0 &&
           std::isfinite(c.radius_m) && c.radius_m >= 0 &&
           std::isfinite(c.velocity_filter_hz) && c.velocity_filter_hz >= 0;
}

float bearing(float n, float e)
{
    const float deg = std::atan2(e, n) * deg_per_rad;
    return deg < 0 ? deg + 360.0f : deg;
}
} // namespace

bool NavigationDP::config_valid(const Config& config)
{
    return dp_config_valid(config);
}

bool NavigationDP::point_valid(Point p)
{
    return p.lat_e7 >= -900000000 && p.lat_e7 <= 900000000 &&
           p.lon_e7 >= -1800000000 && p.lon_e7 <= 1800000000;
}

bool NavigationDP::capture(Point p)
{
    if (!point_valid(p)) { return false; }
    _target = p;
    _target_valid = true;
    _filter_seeded = false;
    return true;
}

void NavigationDP::clear_target()
{
    _target_valid = false;
    _filter_seeded = false;
    _output = {};
}

void NavigationDP::update(const Input& in, const Config& config, float dt_seconds)
{
    Output out{};   // Never leak a stale demand after invalid EKF/target/config.
    out.target = _target;
    out.target_valid = _target_valid;
    out.actual = in.actual;
    out.actual_valid = in.position_valid && point_valid(in.actual);

    // DP supplies its own mode-local reasons; the supervisor combines these
    // into the single NAV_RBITS mask only when Full DP is requested. Missing
    // target, position, velocity, heading, bad gains and timing can coexist.
    if (!_target_valid) { out.reason_bits |= NavigationSupervisor::TargetInvalid; }
    if (!out.actual_valid) { out.reason_bits |= NavigationSupervisor::PositionInvalid; }
    if (!in.velocity_valid || !std::isfinite(in.velocity_n_ms) ||
        !std::isfinite(in.velocity_e_ms)) {
        out.reason_bits |= NavigationSupervisor::VelocityInvalid;
    }
    if (!in.heading_valid || !std::isfinite(in.heading_deg)) {
        out.reason_bits |= NavigationSupervisor::HeadingInvalid;
    }
    if (!config_valid(config)) { out.reason_bits |= NavigationSupervisor::ConfigInvalid; }
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0 || dt_seconds > 0.2f) {
        out.reason_bits |= NavigationSupervisor::TimingFault;
    }
    if (in.estimator_reset) { out.reason_bits |= NavigationSupervisor::EstimatorReset; }
    if (!_target_valid || !out.actual_valid) {
        _filter_seeded = false;
        _output = out;
        return;
    }

    // Local tangent-plane error: use doubles on E7 coordinates to retain
    // centimetre precision. The controller is for station-keeping distances,
    // not transoceanic navigation. Wrap longitude across the dateline.
    const double lat_delta = double(_target.lat_e7) - double(in.actual.lat_e7);
    double lon_delta = double(_target.lon_e7) - double(in.actual.lon_e7);
    if (lon_delta > 1800000000.0) { lon_delta -= 3600000000.0; }
    if (lon_delta < -1800000000.0) { lon_delta += 3600000000.0; }
    const double mid_lat = (double(_target.lat_e7) + double(in.actual.lat_e7)) * 0.5;
    out.error_n_m = float(lat_delta * rad_per_e7 * earth_radius_m);
    out.error_e_m = float(lon_delta * rad_per_e7 * earth_radius_m *
                          std::cos(mid_lat * rad_per_e7));
    out.distance_m = std::hypot(out.error_n_m, out.error_e_m);
    out.bearing_deg = out.distance_m > 0 ? bearing(out.error_n_m, out.error_e_m) : 0;

    // The supervisor reports shared yaw blockers. Keep position diagnostics
    // available while holding translation demand until yaw control is ready.
    if (out.reason_bits != 0 || !in.heading_command_valid) {
        _filter_seeded = false;
        _output = out;
        return;
    }

    float vn = in.velocity_n_ms;
    float ve = in.velocity_e_ms;
    if (config.velocity_filter_hz > 0) {
        if (!_filter_seeded) {
            _filtered_vn = vn;
            _filtered_ve = ve;
            _filter_seeded = true;
        } else {
            const float rc = 1.0f / (2.0f * 3.14159265358979323846f * config.velocity_filter_hz);
            const float alpha = dt_seconds / (rc + dt_seconds);
            _filtered_vn += alpha * (vn - _filtered_vn);
            _filtered_ve += alpha * (ve - _filtered_ve);
        }
        vn = _filtered_vn;
        ve = _filtered_ve;
    } else {
        _filter_seeded = false;
    }
    out.velocity_n_ms = vn;
    out.velocity_e_ms = ve;

    // Position P -> limited desired N/E velocity; velocity P -> limited
    // acceleration. Within limits this is a simple horizontal PD controller.
    // The deadband removes position pull, but NOT drift damping.
    float wanted_n = 0;
    float wanted_e = 0;
    if (out.distance_m > config.radius_m) {
        wanted_n = config.position_kp * out.error_n_m;
        wanted_e = config.position_kp * out.error_e_m;
        const float speed = std::hypot(wanted_n, wanted_e);
        if (!std::isfinite(speed)) { out.reason_bits |= NavigationSupervisor::ConfigInvalid; _output = out; return; }
        if (speed > config.max_speed_ms) {
            wanted_n *= config.max_speed_ms / speed;
            wanted_e *= config.max_speed_ms / speed;
        }
    }
    float accel_n = config.velocity_kp * (wanted_n - vn);
    float accel_e = config.velocity_kp * (wanted_e - ve);
    float accel = std::hypot(accel_n, accel_e);
    if (!std::isfinite(accel)) { out.reason_bits |= NavigationSupervisor::ConfigInvalid; _output = out; return; }
    if (accel > config.max_accel_ms2) {
        accel_n *= config.max_accel_ms2 / accel;
        accel_e *= config.max_accel_ms2 / accel;
    }
    out.demand_n = bound(accel_n / config.max_accel_ms2, -1, 1);
    out.demand_e = bound(accel_e / config.max_accel_ms2, -1, 1);
    out.demand_magnitude = std::hypot(out.demand_n, out.demand_e);
    out.demand_bearing_deg = out.demand_magnitude > 0 ?
                              bearing(out.demand_n, out.demand_e) : 0;

    // ONLY yaw rotates horizontal earth demands into body surge/sway.
    // A pitching/rolling hull does not turn vertical motion into a DP drift.
    const float yaw = in.heading_deg * rad_per_deg;
    out.surge = std::cos(yaw) * out.demand_n + std::sin(yaw) * out.demand_e;
    out.sway = -std::sin(yaw) * out.demand_n + std::cos(yaw) * out.demand_e;
    out.valid = true;
    _output = out;
}

} // namespace Vektor2
