#include "VspMapper.h"
#include "LibVSP.h"
#include <fenv.h>
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <initializer_list>

using M = Vektor2::VspMapper;
static void near(float a, float b, float tolerance = 0.0001f)
{
    if (!isfinite(a) || fabsf(a-b) > tolerance) {
        fprintf(stderr, "actual=%g expected=%g tolerance=%g\n", a, b, tolerance);
        assert(false);
    }
}
int main()
{
    // Legacy pure-Y rod commands must remain defined with SITL FPE traps.
    feclearexcept(FE_ALL_EXCEPT);
    const auto up = mapCoordinates({1500, 1525}, setupLimiter(25, 1500, 1500), 0);
    const auto down = mapCoordinates({1500, 1475}, setupLimiter(25, 1500, 1500), 0);
    assert(up.x_coordinate == 1500 && up.y_coordinate == 1525);
    assert(down.x_coordinate == 1500 && down.y_coordinate == 1475);
    assert(!(fetestexcept(FE_DIVBYZERO | FE_INVALID)));
    M::Config c {M::Rotation::CCW, 0, 1500, 1500, 500};
    // User's physical truth table: CCW left rod = forward; up = port.
    const uint16_t ccw[][2] = {{1000,1500}, {1500,1000}, {2000,1500}, {1500,2000}};
    for (unsigned rotation = 0; rotation <= 1; rotation++) {
        c.rotation = M::Rotation(rotation);
        for (unsigned i = 0; i < 4; i++) {
            const auto r = M::map(c, M::from_polar(i*90, 1));
            assert(r.valid);
            assert(r.x_us == (rotation == 1 ? ccw[i][0] : 3000-ccw[i][0]));
            assert(r.y_us == (rotation == 1 ? ccw[i][1] : 3000-ccw[i][1]));
            near(r.applied.direction_deg, i*90);
            near(r.applied.intensity, 1);
        }
    }
    c.rotation = M::Rotation::CCW;
    auto r = M::map(c, {1,1});
    assert(r.valid && r.limited && r.x_us < 1500 && r.y_us < 1500);
    near(r.applied.direction_deg, 45, 0.2f);
    assert(r.applied.intensity <= 1);
    r = M::map(c, {FLT_MAX, FLT_MAX});
    assert(r.valid && r.limited);
    near(r.applied.direction_deg, 45, 0.2f);
    r = M::map(c, {0,0});
    assert(r.valid && r.x_us == 1500 && r.y_us == 1500);
    assert(isnan(r.applied.direction_deg)); near(r.applied.intensity, 0);
    near(M::from_polar(-90, 1).starboard, -1);
    near(M::from_polar(450, 1).starboard, 1);
    assert(!M::map(c, M::from_polar(0, -1)).valid);
    assert(!M::map(c, {NAN,0}).valid);
    assert(!M::map(c, {0,INFINITY}).valid);
    assert(!M::estimate(c, 2000, 2000).valid);
    assert(!M::estimate(c, 0, 65535).valid);
    // Actual legacy limiter truncation can put a diagonal outside the ring.
    const M::Config small {M::Rotation::CCW, 0, 1500, 1500, 25};
    const auto diagonal = mapCoordinates({1475,1475}, setupLimiter(25,1500,1500), 0);
    assert(diagonal.x_coordinate == 1482 && diagonal.y_coordinate == 1482);
    const auto rounded = M::estimate(small, diagonal.x_coordinate, diagonal.y_coordinate);
    assert(rounded.valid);
    near(rounded.direction_deg, 45);
    near(rounded.intensity, 1);
    near(hypotf(rounded.vector.forward, rounded.vector.starboard), 1);
    assert(!M::estimate(small, 1470, 1470).valid);
    assert(!M::estimate(small, 1527, 1500).valid);
    for (int angle = 0; angle < 360; angle++) {
        const auto rods = mapCoordinates({1475,1475}, setupLimiter(25,1500,1500), angle);
        const auto estimated = M::estimate(small, rods.x_coordinate, rods.y_coordinate);
        assert(estimated.valid && estimated.intensity <= 1);
    }
    c.installation_deg = 90;
    r = M::map(c, {1,0});
    assert(r.valid && r.x_us == 1500 && r.y_us == 2000);
    near(r.applied.vector.forward, 1);
    near(r.applied.vector.starboard, 0);
    // Sweep mounting, magnitude, angle and both rotations, including small
    // radii where integer quantization is significant. All rods stay in disk.
    for (int radius : {1, 25, 199}) {
        c.radius = radius; c.x_center = 1430; c.y_center = 1580;
        for (unsigned rotation = 0; rotation <= 1; rotation++) {
            c.rotation = M::Rotation(rotation);
            for (int mount = 0; mount < 360; mount += 37) {
                c.installation_deg = mount;
                for (int angle = 0; angle < 360; angle++) {
                    for (float magnitude : {0.0f, 0.3f, 1.0f, 2.0f}) {
                        const auto demand = M::from_polar(angle, magnitude);
                        const auto expected = M::from_polar(angle, fminf(magnitude, 1));
                        r = M::map(c, demand);
                        assert(r.valid);
                        assert(r.applied.intensity <= 1.000001f);
                        near(r.applied.vector.forward, expected.forward, 1.5f/radius);
                        near(r.applied.vector.starboard, expected.starboard, 1.5f/radius);
                    }
                }
            }
        }
    }
    c = {M::Rotation(-1), 0, 1500, 1500, 25};
    assert(!M::map(c, {0,0}).valid);
    c.rotation = M::Rotation(2); assert(!M::valid_config(c));
    c.rotation = M::Rotation(256); assert(!M::valid_config(c));
    c.rotation = M::Rotation::CW;
    c.radius = 0; assert(!M::valid_config(c));
    c.radius = -1; assert(!M::valid_config(c));
    c.radius = 501; assert(!M::valid_config(c));
    c.radius = 25; c.x_center = 1000; assert(!M::valid_config(c));
    c.x_center = 1500; c.installation_deg = NAN; assert(!M::valid_config(c));
    c.installation_deg = 360; assert(!M::valid_config(c));
    puts("VSP mapper tests passed");
}
