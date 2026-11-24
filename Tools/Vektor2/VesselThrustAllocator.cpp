#include "VesselThrustAllocator.h"

#include <math.h>

namespace Vektor2 {
namespace {

// No heap allocation. Solve a small symmetric 3x3 system with pivoting.
// Reject missing yaw authority and ill-conditioned layouts instead of
// fabricating a partial command from a rank-deficient actuator set.
bool solve_3x3(const double m[3][3], const double v[3], double result[3])
{
    double a[3][4] = {};
    double largest = 0;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            a[i][j] = m[i][j];
            if (fabs(a[i][j]) > largest) { largest = fabs(a[i][j]); }
        }
        a[i][3] = v[i];
    }
    if (!isfinite(largest) || largest <= 0) { return false; }

    for (unsigned k = 0; k < 3; ++k) {
        unsigned pivot = k;
        for (unsigned i = k + 1; i < 3; ++i) {
            if (fabs(a[i][k]) > fabs(a[pivot][k])) { pivot = i; }
        }
        // 1e-10 is a conservative cutoff for layout degeneracy, after
        // normalizing yaw by the characteristic vessel lever arm.
        if (fabs(a[pivot][k]) <= largest * 1.0e-10) { return false; }
        if (pivot != k) {
            for (unsigned j = k; j < 4; ++j) {
                const double temp = a[k][j];
                a[k][j] = a[pivot][j];
                a[pivot][j] = temp;
            }
        }
        for (unsigned i = k + 1; i < 3; ++i) {
            const double multiplier = a[i][k] / a[k][k];
            for (unsigned j = k + 1; j < 4; ++j) {
                a[i][j] -= multiplier * a[k][j];
            }
            a[i][k] = 0;
        }
    }
    for (int i = 2; i >= 0; --i) {
        double sum = a[i][3];
        for (unsigned j = unsigned(i) + 1; j < 3; ++j) {
            sum -= a[i][j] * result[j];
        }
        result[i] = sum / a[i][i];
        if (!isfinite(result[i])) { return false; }
    }
    return true;
}

bool valid_vsp(const VesselThrustAllocator::Vsp& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.force) &&
           v.force > 0;
}

} // namespace

VesselThrustAllocator::Result VesselThrustAllocator::allocate(
    const Geometry& g, const Demand& demand, bool bow_available)
{
    Result out;
    if ((g.layout != Layout::AFT && g.layout != Layout::VWT) ||
        !valid_vsp(g.vsp1) || !valid_vsp(g.vsp2) ||
        !isfinite(demand.surge) || !isfinite(demand.sway) || !isfinite(demand.yaw)) {
        return out;
    }
    const bool use_bow = g.layout == Layout::AFT && bow_available;
    if (use_bow && (!isfinite(g.bow_x) || !isfinite(g.bow_force) || g.bow_force <= 0)) {
        return out;
    }

    // Each column maps one normalized actuator axis to surge/sway/yaw.
    // Zero the unused bow column, rather than mixing automatic allocation
    // with independent manual bow control.
    const double arm = fmax(1.0, fmax(fmax(fabs(double(g.vsp1.x)), fabs(double(g.vsp1.y))),
                                  fmax(fmax(fabs(double(g.vsp2.x)), fabs(double(g.vsp2.y))),
                                       use_bow ? fabs(double(g.bow_x)) : 0.0)));
    const double c1 = g.vsp1.force;
    const double c2 = g.vsp2.force;
    const double cb = use_bow ? double(g.bow_force) : 0.0;
    const double a[3][5] = {
        {c1, 0, c2, 0, 0},
        {0, c1, 0, c2, cb},
        {-g.vsp1.y*c1/arm, g.vsp1.x*c1/arm,
         -g.vsp2.y*c2/arm, g.vsp2.x*c2/arm,
         use_bow ? g.bow_x*cb/arm : 0.0}
    };
    double gram[3][3] = {};
    for (unsigned r = 0; r < 3; ++r) {
        for (unsigned c = 0; c < 3; ++c) {
            for (unsigned j = 0; j < 5; ++j) {
                gram[r][c] += a[r][j] * a[c][j];
            }
        }
    }
    const double request[3] = {demand.surge, demand.sway, double(demand.yaw)/arm};
    double multipliers[3] = {};
    if (!solve_3x3(gram, request, multipliers)) { return out; }

    // Minimum Euclidean actuator effort solution. The two VSP force vectors
    // are independent; the transverse bow vector is an optional fifth axis.
    double u[5] = {};
    for (unsigned j = 0; j < 5; ++j) {
        for (unsigned r = 0; r < 3; ++r) {
            u[j] += a[r][j] * multipliers[r];
        }
        if (!isfinite(u[j])) { return Result{}; }
    }
    const double peak = fmax(1.0, fmax(fmax(hypot(u[0], u[1]), hypot(u[2], u[3])), fabs(u[4])));
    if (!isfinite(peak)) { return Result{}; }
    const double scale = 1.0 / peak;
    for (double& axis : u) { axis *= scale; }

    out.valid = true;
    out.bow_allocated = use_bow;
    out.saturated = peak > 1.0 + 1.0e-9;
    out.demand_scale = float(scale);
    out.vsp1 = {float(u[0]), float(u[1])};
    out.vsp2 = {float(u[2]), float(u[3])};
    out.bow = float(u[4]);
    out.achieved = achieved(g, out);
    return out;
}

VesselThrustAllocator::Demand VesselThrustAllocator::achieved(const Geometry& g, const Result& r)
{
    const double x1 = double(g.vsp1.force) * r.vsp1.forward;
    const double y1 = double(g.vsp1.force) * r.vsp1.starboard;
    const double x2 = double(g.vsp2.force) * r.vsp2.forward;
    const double y2 = double(g.vsp2.force) * r.vsp2.starboard;
    const double bow = r.bow_allocated ? double(g.bow_force) * r.bow : 0;
    const double bow_moment = r.bow_allocated ? g.bow_x * bow : 0;
    return {float(x1 + x2), float(y1 + y2 + bow),
            float(g.vsp1.x * y1 - g.vsp1.y * x1 + g.vsp2.x * y2 - g.vsp2.y * x2 + bow_moment)};
}

} // namespace Vektor2
