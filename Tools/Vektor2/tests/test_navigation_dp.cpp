#include "NavigationEstimatorResets.h"
#include "NavigationDP.h"
#include "NavigationHeading.h"
#include "NavigationSupervisor.h"
#include <cassert>
#include <cmath>
#include <limits>

using namespace Vektor2;
using Reason = NavigationSupervisor;
static bool near(float a, float b, float eps = 0.002f) { return std::fabs(a - b) < eps; }

int main()
{
    {
        Vektor2::NavigationEstimatorResets resets;
        assert(!resets.update(0, 0, 0));
        assert(!resets.update(0, 0, 0));
        assert(resets.update(100, 0, 0)); // first observed yaw reset
        assert(!resets.update(100, 0, 0));
        assert(resets.update(100, 200, 0)); // EKF position reset
        assert(resets.update(100, 200, 300)); // EKF velocity reset
        assert(!resets.update(100, 200, 300));
    }
    {
        Vektor2::NavigationHeading hdg;
        Vektor2::NavigationHeading::Config config;
        config.kp = .1f;
        config.kd = 0;
        assert(hdg.capture(359));
        hdg.update(1, 0, true, config, false); // observable, no authority
        assert(hdg.output().actual_valid);
        assert(!hdg.output().valid);
        assert(std::fabs(hdg.output().error_deg + 2) < .001f);
        hdg.update(1, 0, true, config, true);
        assert(hdg.output().valid);
    }

    NavigationDP dp;
    NavigationDP::Config cfg;
    cfg.position_kp = 0.5f; cfg.velocity_kp = 1.0f;
    cfg.max_speed_ms = 2.0f; cfg.max_accel_ms2 = 1.0f;
    cfg.radius_m = 0.1f;
    NavigationDP::Input in;
    in.actual = {500000000, 140000000}; // 50 N, 14 E
    in.position_valid = in.velocity_valid = in.heading_valid = true;
    in.heading_deg = 0.0f;
    assert(NavigationDP::config_valid(cfg));
    assert(dp.capture(in.actual));
    dp.update(in, cfg, 0.01f);
    assert(dp.output().valid);
    assert(dp.output().reason_bits == 0);
    assert(near(dp.output().distance_m, 0));
    assert(near(dp.output().surge, 0));
    assert(near(dp.output().sway, 0));

    // A fixed hull control point with roll/pitch motion is not DP drift.
    // Neither angle is a direct input; only yaw changes the earth/body axes.
    in.heading_deg = 35.0f;
    dp.update(in, cfg, 0.01f);
    assert(near(dp.output().surge, 0) && near(dp.output().sway, 0));

    // A real 1 m N displacement generates southward correction.
    in.actual.lat_e7 += 90;
    in.heading_deg = 0;
    dp.update(in, cfg, 0.01f);
    assert(near(dp.output().error_n_m, -1.00075f, 0.02f));
    assert(dp.output().surge < -0.45f && near(dp.output().sway, 0));
    assert(near(dp.output().bearing_deg, 180));

    // Yaw-only rotation puts earth north demand on vessel sway at +90 deg.
    in.heading_deg = 90;
    dp.update(in, cfg, 0.01f);
    assert(near(dp.output().surge, 0, 0.002f));
    assert(dp.output().sway > 0.45f);

    // Velocity damping can request braking opposite the position-error vector.
    in.actual = {500000000, 140000000};
    in.velocity_n_ms = 1.5f;
    in.heading_deg = 0;
    dp.update(in, cfg, 0.01f);
    assert(near(dp.output().distance_m, 0));
    assert(near(dp.output().surge, -1.0f));
    assert(near(dp.output().demand_magnitude, 1.0f));
    assert(near(dp.output().demand_bearing_deg, 180.0f));

    // Saturate vector norm as a whole, rather than each axis independently.
    in.velocity_n_ms = 4;
    in.velocity_e_ms = 4;
    dp.update(in, cfg, 0.01f);
    assert(near(dp.output().demand_magnitude, 1.0f));
    assert(near(dp.output().surge, -0.7071068f));
    assert(near(dp.output().sway, -0.7071068f));

    // The DP controller emits multi-cause NAV_RBITS-compatible reasons.
    // Invalid inputs must zero demand, never reuse a previous command.
    in.position_valid = false;
    in.velocity_valid = false;
    in.heading_valid = false;
    in.estimator_reset = true;
    cfg.max_accel_ms2 = 0;
    assert(!NavigationDP::config_valid(cfg));
    dp.update(in, cfg, 0.3f);
    const uint16_t expected = Reason::PositionInvalid | Reason::VelocityInvalid |
                              Reason::HeadingInvalid | Reason::EstimatorReset |
                              Reason::ConfigInvalid | Reason::TimingFault;
    assert(!dp.output().valid && near(dp.output().surge, 0));
    assert((dp.output().reason_bits & expected) == expected);
    in.position_valid = true;
    in.velocity_valid = true;
    in.heading_valid = true;
    in.estimator_reset = false;
    cfg.max_accel_ms2 = 1;
    dp.update(in, cfg, 0.3f);
    assert(!dp.output().valid && dp.output().reason_bits == Reason::TimingFault);
    dp.update(in, cfg, 0.01f);
    assert(dp.output().valid && dp.output().reason_bits == 0);
    dp.clear_target();
    dp.update(in, cfg, 0.01f);
    assert(!dp.output().target_valid && !dp.output().valid);
    assert(dp.output().reason_bits == Reason::TargetInvalid);
    // An estimate marked valid with NaN velocity is still invalid.
    assert(dp.capture({500000000, 140000000}));
    in.velocity_n_ms = std::numeric_limits<float>::quiet_NaN();
    dp.update(in, cfg, 0.01f);
    assert(dp.output().reason_bits == Reason::VelocityInvalid);
    in.velocity_n_ms = 0;

    // Dateline wrapping is local, not a 40,000 km correction request.
    assert(dp.capture({1000000, 1799999990}));
    in.actual = {1000000, -1799999990};
    in.velocity_n_ms = in.velocity_e_ms = 0;
    dp.update(in, cfg, 0.01f);
    assert(dp.output().distance_m < 1.0f);
    assert(dp.output().error_e_m < 0);
    assert(!dp.capture({910000000, 0}));

    // Basic optional velocity smoothing, with reset on stale measurements.
    assert(dp.capture({500000000, 140000000}));
    in.actual = {500000000, 140000000};
    cfg.velocity_filter_hz = 1.0f;
    dp.update(in, cfg, 0.01f);
    assert(near(dp.output().velocity_n_ms, 0));
    in.velocity_n_ms = 1;
    dp.update(in, cfg, 0.01f);
    assert(dp.output().velocity_n_ms > 0 && dp.output().velocity_n_ms < 1);
    in.velocity_valid = false;
    dp.update(in, cfg, 0.01f);
    in.velocity_valid = true;
    dp.update(in, cfg, 0.01f);
    assert(near(dp.output().velocity_n_ms, 1));

    NavigationHeading hdg;
    NavigationHeading::Config hc;
    hc.kp = 0.1f; hc.kd = 0.01f; hc.max_yaw_percent = 80;
    assert(NavigationHeading::config_valid(hc));
    assert(hdg.capture(359));
    hdg.update(1, 0, true, hc);
    assert(hdg.output().valid);
    assert(near(hdg.output().error_deg, -2));
    assert(near(hdg.output().yaw_demand, -0.2f));
    hdg.update(350, 0, true, hc);
    assert(near(hdg.output().yaw_demand, 0.8f));
    hdg.update(359, 5, true, hc);
    assert(near(hdg.output().yaw_demand, -0.05f));
    hdg.update(0, 0, false, hc);
    assert(!hdg.output().valid && near(hdg.output().yaw_demand, 0));
    assert(hdg.target_valid());
    hdg.clear_target();
    assert(!hdg.target_valid());

    // Correct Euler yaw rate from body gyro during a tilted attitude.
    float yaw_rate = 0;
    assert(NavigationHeading::earth_yaw_rate(0.2f, 0, 30, 0, yaw_rate));
    assert(near(yaw_rate, 0.1f * 57.2957795f));
    assert(NavigationHeading::earth_yaw_rate(0, 0.2f, 0, 0, yaw_rate));
    assert(near(yaw_rate, 0.2f * 57.2957795f));
    assert(!NavigationHeading::earth_yaw_rate(0, 0.2f, 0, 80, yaw_rate));
    assert(!hdg.capture(std::numeric_limits<float>::quiet_NaN()));
    return 0;
}
