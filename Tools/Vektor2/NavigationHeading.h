#pragma once

namespace Vektor2 {

// Shared heading hold controller for both Heading Hold and Full DP.
// This module never owns/commands any physical actuator.
class NavigationHeading {
public:
    struct Config {
        float kp = 0; // normalized yaw per degree of heading error
        float kd = 0; // normalized yaw per degree/s of earth yaw rate
        float max_yaw_percent = 100;
        float deadband_deg = 0;
    };
    struct Output {
        bool target_valid = false;
        bool actual_valid = false;
        bool valid = false;
        float target_deg = 0;
        float actual_deg = 0;
        float error_deg = 0;
        float yaw_demand = 0; // normalized, NOT actuator command
    };
    static bool config_valid(const Config& config);
    bool capture(float heading_deg);
    void clear_target();
    bool target_valid() const { return _target_valid; }
    const Output& output() const { return _output; }
    void update(float heading_deg, float yaw_rate_deg_s, bool heading_valid,
                const Config& config, bool controller_enabled = true);
    // Convert body gyro q,r into horizontal earth-heading rate; roll/pitch
    // movements alone are not necessarily yaw. Angles are in degrees.
    static bool earth_yaw_rate(float gyro_y_rad_s, float gyro_z_rad_s,
                               float roll_deg, float pitch_deg, float& result_deg_s);
private:
    bool _target_valid = false;
    float _target_deg = 0;
    Output _output {};
};

} // namespace Vektor2
