#pragma once

#include <stdint.h>

namespace Vektor2 {

// Vessel/layout independent horizontal DP controller. Pure C++11, no HAL,
// allocator or actuator access. Lat/lon describe the same vessel control point
// both when capturing and when observing. Commands are NORMALIZED requests,
// not measured force or thrust.
class NavigationDP {
public:
    struct Point {
        int32_t lat_e7 = 0;
        int32_t lon_e7 = 0;
        Point(int32_t lat = 0, int32_t lon = 0) : lat_e7(lat), lon_e7(lon) {}
    };
    struct Config {
        float position_kp = 0;    // (m/s)/m
        float velocity_kp = 0;    // (m/s^2)/(m/s)
        float max_speed_ms = 0;
        float max_accel_ms2 = 0;
        float radius_m = 0;
        float velocity_filter_hz = 0; // zero = use EKF estimate directly
    };
    struct Input {
        Point actual {};
        float velocity_n_ms = 0;
        float velocity_e_ms = 0;
        float heading_deg = 0;
        bool position_valid = false;
        bool velocity_valid = false;
        bool heading_valid = false;
        bool heading_command_valid = false; // shared yaw controller readiness
        bool estimator_reset = false;
    };
    struct Output {
        Point target {};
        Point actual {};
        bool target_valid = false;
        bool actual_valid = false;
        bool valid = false;   // all inputs + gains usable, never ownership
        uint16_t reason_bits = 0; // Shared NAV_RBITS vocabulary, DP-local conditions
        float distance_m = 0;
        float bearing_deg = 0;
        float error_n_m = 0;
        float error_e_m = 0;
        float velocity_n_ms = 0;
        float velocity_e_ms = 0;
        float demand_n = 0;
        float demand_e = 0;
        float surge = 0;     // +forward, normalized [-1,1]
        float sway = 0;      // +starboard, normalized [-1,1]
        float demand_magnitude = 0;
        float demand_bearing_deg = 0;
    };

    bool capture(Point point);
    void clear_target();
    bool target_valid() const { return _target_valid; }
    const Output& output() const { return _output; }
    void update(const Input& input, const Config& config, float dt_seconds);
    static bool point_valid(Point point);
    static bool config_valid(const Config& config);

private:
    Point _target {};
    bool _target_valid = false;
    bool _filter_seeded = false;
    float _filtered_vn = 0;
    float _filtered_ve = 0;
    Output _output {};
};

} // namespace Vektor2
