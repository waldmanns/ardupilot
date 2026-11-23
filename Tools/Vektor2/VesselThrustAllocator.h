#pragma once

#include "VspMapper.h"

namespace Vektor2 {

// Pure, host-testable vessel-level allocator. No RC, routing, PWM, motor or
// navigation-state ownership. A call can NEVER engage an actuator.
class VesselThrustAllocator {
public:
    enum class Layout : unsigned char { AFT = 1, VWT = 2 };

    // Body frame: +surge forward, +sway starboard; +yaw is the moment
    // x * Fy - y * Fx. Geometry is relative to the chosen vessel reference
    // point. Thrust capacities use consistent but UNCALIBRATED force units.
    struct Demand {
        float surge = 0;
        float sway = 0;
        float yaw = 0;
        Demand(float surge_in = 0, float sway_in = 0, float yaw_in = 0) :
            surge(surge_in), sway(sway_in), yaw(yaw_in) {}
    };
    struct Vsp {
        float x = 0;
        float y = 0;
        float force = 0;  // positive force capacity at normalized vector norm 1
        Vsp(float x_in = 0, float y_in = 0, float force_in = 0) :
            x(x_in), y(y_in), force(force_in) {}
    };
    struct Geometry {
        Layout layout = Layout::AFT;
        Vsp vsp1 {};
        Vsp vsp2 {};
        float bow_x = 0;
        float bow_force = 0; // positive transverse force capacity, AFT only
    };
    struct Result {
        bool valid = false;       // true only if actuator geometry is solvable
        bool saturated = false;   // demanded vector was uniformly reduced
        bool bow_allocated = false; // AFT bow was available to this computation
        float demand_scale = 0;  // achieved demand / requested demand, 0..1
        VspMapper::Vector vsp1 {0, 0};
        VspMapper::Vector vsp2 {0, 0};
        float bow = 0;           // -1..1, 0 if manually owned/unavailable/VWT
        Demand achieved {};      // achieved normalized force/moment request
    };

    // bow_available MUST be supplied by a future confirmed ownership layer;
    // it is not inferred from an RC potentiometer position. Both VSPs are
    // assumed available. Callers must NOT send results directly to hardware.
    static Result allocate(const Geometry& geometry, const Demand& demand,
                           bool bow_available);
};

} // namespace Vektor2
