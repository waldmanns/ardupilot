#include "VesselThrustAllocator.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using A = Vektor2::VesselThrustAllocator;

static bool near(float x, float y, float eps = 0.00002f)
{
    return std::fabs(x - y) < eps;
}
static bool neutral(const A::Result& out)
{
    return out.vsp1.forward == 0 && out.vsp1.starboard == 0 &&
           out.vsp2.forward == 0 && out.vsp2.starboard == 0 && out.bow == 0;
}
static void bounded(const A::Result& out)
{
    assert(std::hypot(out.vsp1.forward, out.vsp1.starboard) <= 1.00001);
    assert(std::hypot(out.vsp2.forward, out.vsp2.starboard) <= 1.00001);
    assert(std::fabs(out.bow) <= 1.00001);
}
static A::Geometry aft()
{
    A::Geometry g;
    g.layout = A::Layout::AFT;
    g.vsp1 = {-2.0f, -1.0f, 1.0f};
    g.vsp2 = {-2.0f, 1.0f, 1.0f};
    g.bow_x = 4.0f;
    g.bow_force = 0.7f;
    return g;
}
static void tracks(const A::Geometry& g, A::Demand d, bool bow_available)
{
    const auto out = A::allocate(g, d, bow_available);
    assert(out.valid && !out.saturated && near(out.demand_scale, 1));
    assert(near(out.achieved.surge, d.surge));
    assert(near(out.achieved.sway, d.sway));
    assert(near(out.achieved.yaw, d.yaw));
    bounded(out);
}

int main()
{
    const auto g = aft();
    auto out = A::allocate(g, {}, true);
    assert(out.valid && neutral(out) && !out.saturated && out.bow_allocated);
    out = A::allocate(g, {}, false);
    assert(out.valid && neutral(out) && !out.bow_allocated);

    // All three force/moment axes, including counter-thrust and independently
    // vectored drives. AFT must work both with and without bow ownership.
    tracks(g, {0.3f, 0, 0}, false);
    tracks(g, {0, 0.3f, 0}, false);
    tracks(g, {0, 0, 0.3f}, false);
    tracks(g, {-0.2f, 0.16f, -0.25f}, true);
    tracks(g, {0.15f, 0.16f, 0.10f}, false);
    out = A::allocate(g, {0, 0.2f, 0.5f}, true);
    assert(out.valid && !out.saturated && std::fabs(out.bow) > 0.00001f);
    out = A::allocate(g, {0, 0.2f, 0.5f}, false);
    assert(out.valid && out.bow == 0 && !out.bow_allocated);

    // Bow availability switches the allocation geometry without forcing
    // either VSP into manual control; manual bow command is never modified.
    auto g_vwt = g;
    g_vwt.layout = A::Layout::VWT;
    g_vwt.vsp1.x = g_vwt.vsp2.x = 1.5f;
    g_vwt.bow_force = std::numeric_limits<float>::quiet_NaN();
    tracks(g_vwt, {0.15f, 0.1f, 0.25f}, true);
    out = A::allocate(g_vwt, {0.15f, 0.1f, 0.25f}, true);
    assert(out.bow == 0 && !out.bow_allocated);
    assert(near(out.vsp1.forward + out.vsp2.forward, 0.15f));

    // Saturation scales every axis together, with achieved wrench reported.
    // This is an intentionally conservative policy, not a mode priority.
    const A::Demand huge {30, -24, 40};
    out = A::allocate(g, huge, true);
    assert(out.valid && out.saturated && out.demand_scale > 0 && out.demand_scale < 1);
    bounded(out);
    assert(near(out.achieved.surge, huge.surge*out.demand_scale));
    assert(near(out.achieved.sway, huge.sway*out.demand_scale));
    assert(near(out.achieved.yaw, huge.yaw*out.demand_scale));

    // No authority to produce yaw with coincident VSP locations and no bow.
    auto singular = g;
    singular.vsp2.x = singular.vsp1.x;
    singular.vsp2.y = singular.vsp1.y;
    out = A::allocate(singular, {0.2f, 0, 0.1f}, false);
    assert(!out.valid && neutral(out));
    // An AFT bow at a distinct x restores rank if it is owned by automation.
    out = A::allocate(singular, {0.2f, 0, 0.1f}, true);
    assert(out.valid);

    auto invalid = g;
    invalid.vsp1.force = -1;
    out = A::allocate(invalid, {0.2f, 0, 0}, true);
    assert(!out.valid && neutral(out));
    invalid = g;
    invalid.vsp1.x = std::numeric_limits<float>::quiet_NaN();
    out = A::allocate(invalid, {0.2f, 0, 0}, true);
    assert(!out.valid && neutral(out));
    invalid = g;
    invalid.bow_force = 0;
    out = A::allocate(invalid, {0.2f, 0, 0}, true);
    assert(!out.valid && neutral(out));
    out = A::allocate(invalid, {0.2f, 0, 0}, false);
    assert(out.valid && out.bow == 0);
    out = A::allocate(g, {0, std::numeric_limits<float>::quiet_NaN(), 0}, false);
    assert(!out.valid && neutral(out));
    invalid = g;
    invalid.layout = static_cast<A::Layout>(0);
    out = A::allocate(invalid, {0.2f, 0, 0}, false);
    assert(!out.valid && neutral(out));

    // Sweep the signs of all axes; the output stays finite and bounded.
    for (int i = -6; i <= 6; ++i) {
        for (int j = -6; j <= 6; ++j) {
            const A::Demand d {i*0.03f, j*0.03f, (i-j)*0.04f};
            for (bool bow : {false, true}) {
                tracks(g, d, bow);
            }
        }
    }
    std::cout << "Vessel thrust allocator host tests passed\n";
}
