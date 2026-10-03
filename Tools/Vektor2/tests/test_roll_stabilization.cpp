// Host tests for the production controller AND both production VSP functions.
// Run through test_roll_stabilization.py; no AHRS, HAL or physical PWM needed.
#include "RollStabilization.h"
#include "Telemetry.h"
#include "VSP1.h"
#include "VSP2.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <limits>
#include <initializer_list>

using namespace Vektor2;

// The real VSP files declare their telemetry here too. Output tests never send
// it; this transport stub avoids bringing MAVLink into a host control test.
void Vektor2::NamedFloatTelemetry::send_float(const char*, float) const {}

static bool same(Coordinates a, Coordinates b)
{
    return a.x_coordinate == b.x_coordinate && a.y_coordinate == b.y_coordinate;
}

struct Fixture {
    RollStabilization controller;
    RollSettings settings;
    RollDriveSettings drives[2];
    RollSample sample;
    Limiter limits = setupLimiter(100, 1500, 1500);
    Coordinates neutral {1500, 1500};

    Fixture()
    {
        settings.enabled = true;
        settings.d = 0.1f;
        settings.filter_hz = 20;
        settings.on_ms = 100;
        settings.off_ms = 100;
        settings.slew = 100;
        settings.maneuver_limit = 1;
        sample.attitude_valid = sample.control_valid = true;
        sample.rate_dps = -5; // positive correction at ZERO roll angle
        drives[0].angle_deg = 0;
        drives[0].sign = 1;
        drives[0].maximum = 1;
        drives[1].angle_deg = 180;
        drives[1].sign = -1;
        drives[1].maximum = 1;
    }

    void tick(uint32_t time) { controller.update(time, settings, drives, sample); }
    void ready(uint32_t start = 0) { tick(start); tick(start + 100); }
    Coordinates apply(uint8_t drive = 0) { return controller.apply(drive, neutral, limits, true); }
};

static void disabled_equivalence()
{
    RollStabilization disabled;
    // Compare against the unchanged pre-VRS pipeline for both drivers, including
    // out-of-input-range values, centre offsets and every quadrant of THR_ANG.
    for (int angle : {0, 1, 45, 90, 179, 180, 270, 359}) {
        for (int radius : {0, 25, 100, 500}) {
            for (int x : {0, 999, 1000, 1200, 1499, 1500, 1501, 1800, 2000, 2001, 65535}) {
                for (int y : {0, 1000, 1250, 1500, 1750, 2000, 65535}) {
                    const uint16_t input[3] {uint16_t(x), uint16_t(y), 1732};
                    const Coordinates mapped {
                        vsp_common_map(x, 1000, 2000, 1473-radius, 1473+radius),
                        vsp_common_map(y, 1000, 2000, 1531-radius, 1531+radius)};
                    const Coordinates expected = mapCoordinates(mapped, setupLimiter(radius, 1473, 1531), angle);
                    const Components::VSP1Params first {int16_t(radius), 1473, 1531, 0, int16_t(angle)};
                    const Components::VSP2Params second {int16_t(radius), 1473, 1531, 0, int16_t(angle)};
                    uint16_t a[3], b[3], c[3], d[3];
                    Components::vsp1_loop(input, a, first);
                    Components::vsp2_loop(input, b, second);
                    Components::vsp1_loop(input, c, first, &disabled);
                    Components::vsp2_loop(input, d, second, &disabled);
                    for (auto* output : {a, b, c, d}) {
                        assert(output[0] == expected.x_coordinate);
                        assert(output[1] == expected.y_coordinate);
                        assert(output[2] == input[2]);
                    }
                }
            }
        }
    }
    // In particular, disabling halfway through a pulse has NO residual ramp.
    Fixture f;
    f.ready();
    assert(!same(f.apply(), f.neutral));
    f.settings.enabled = false;
    f.tick(110);
    assert(same(f.apply(), f.neutral));
    assert(f.controller.state() == RollStabilization::State::Disabled);
    f.settings.enabled = true;
    f.tick(120);
    assert(same(f.apply(), f.neutral));
    puts("PASS: disabled equivalence across 9,856 driver cases and live disable");
}

static void signs_timing_and_input()
{
    Fixture f;
    f.ready();
    assert(f.apply().x_coordinate == 1550); // D works at level
    assert(same(f.apply(1), f.neutral));
    f.tick(190);
    assert(f.apply().x_coordinate == 1550);
    f.tick(200); // exactly the maximum duration
    assert(same(f.apply(), f.neutral));
    assert(f.controller.state() == RollStabilization::State::Recovery);
    f.sample.rate_dps = 5;
    f.tick(210);
    assert(same(f.apply(1), f.neutral)); // cannot bypass recovery with other VSP
    f.tick(299);
    assert(same(f.apply(1), f.neutral));
    f.tick(300);
    assert(f.apply(1).x_coordinate < 1500);

    f.sample.attitude_valid = false;
    f.tick(310);
    assert(same(f.apply(1), f.neutral));
    assert(f.controller.inhibited() & RollStabilization::Attitude);
    f.sample.attitude_valid = true;
    f.tick(320);
    assert(same(f.apply(1), f.neutral));
    f.tick(420);
    assert(!same(f.apply(1), f.neutral));
    f.sample.control_valid = false;
    f.tick(430);
    assert(same(f.apply(1), f.neutral));
    assert(f.controller.inhibited() & RollStabilization::Input);

    Fixture missing;
    missing.ready();
    assert(same(missing.controller.apply(0, missing.neutral, missing.limits, false), missing.neutral));
    Fixture stalled;
    stalled.ready();
    stalled.apply();
    stalled.tick(201);
    assert(same(stalled.apply(), stalled.neutral));
    assert(stalled.controller.inhibited() & RollStabilization::Timing);

    Fixture wrap;
    wrap.ready(UINT32_MAX - 50);
    assert(!same(wrap.apply(), wrap.neutral));
    wrap.tick(149);
    assert(same(wrap.apply(), wrap.neutral));
    puts("PASS: sign allocation, on/off deadlines, missing input, sensor loss, stalls, clock wrap");
}

static void hysteresis_and_reversal()
{
    Fixture f;
    f.settings.p = 1;
    f.settings.d = 0;
    f.settings.deadband = 0.2f;
    f.sample.roll_deg = -0.15f;
    f.ready();
    assert(same(f.apply(), f.neutral));
    f.sample.roll_deg = -0.21f;
    f.tick(110);
    assert(!same(f.apply(), f.neutral));
    f.sample.roll_deg = -0.15f;
    f.tick(120);
    assert(!same(f.apply(), f.neutral)); // hysteresis keeps existing demand active
    f.sample.roll_deg = 0.3f;
    f.tick(130);
    assert(same(f.apply(), f.neutral)); // terminate wrong-sign pulse immediately
    assert(same(f.apply(1), f.neutral)); // opposite drive must wait too
    assert(f.controller.state() == RollStabilization::State::Recovery);
    f.tick(230);
    assert(!same(f.apply(1), f.neutral));
    f.sample.roll_deg = 0.09f;
    f.tick(240);
    assert(same(f.apply(1), f.neutral)); // below half-threshold releases
    f.sample.roll_deg = std::numeric_limits<float>::quiet_NaN();
    f.tick(250);
    assert(same(f.apply(), f.neutral));
    assert(f.controller.inhibited() & RollStabilization::Attitude);
    puts("PASS: demand hysteresis, reversal during a burst, and nonfinite attitude");
}

static void geometry()
{
    unsigned checked = 0;
    // Property sweep includes every allowed integer angle, asymmetric centres,
    // integer PWM quantization, and rings that meet or cross PWM endpoints.
    for (int angle = 0; angle < 360; angle++) {
        for (int center : {1050, 1500, 1950}) {
            for (int x = -90; x <= 90; x += 15) {
                for (int y = -90; y <= 90; y += 15) {
                    if (x*x + y*y > 10000 || center+x < 1000 || center+x > 2000 ||
                        center+y < 1000 || center+y > 2000) {
                        continue;
                    }
                    Fixture f;
                    f.drives[0].angle_deg = angle;
                    f.sample.rate_dps = -100; // saturate demand
                    f.limits = setupLimiter(100, center, center);
                    f.neutral = {center+x, center+y};
                    f.ready();
                    const Coordinates out = f.apply();
                    const float dx = out.x_coordinate - f.neutral.x_coordinate;
                    const float dy = out.y_coordinate - f.neutral.y_coordinate;
                    const float ux = cosf(angle * 0.017453292519943295f);
                    const float uy = sinf(angle * 0.017453292519943295f);
                    assert(dx*ux + dy*uy >= -0.0001f); // never reversed
                    assert(fabsf(dx*uy - dy*ux) <= 0.708f); // only PWM quantization error
                    const int ox = out.x_coordinate - center;
                    const int oy = out.y_coordinate - center;
                    assert(ox*ox + oy*oy <= 10000);
                    assert(out.x_coordinate >= 1000 && out.x_coordinate <= 2000);
                    assert(out.y_coordinate >= 1000 && out.y_coordinate <= 2000);
                    checked++;
                }
            }
        }
    }
    Fixture bad;
    bad.ready();
    const Coordinates outside {1600, 1501};
    assert(same(bad.controller.apply(0, outside, bad.limits, true), outside));
    assert(bad.controller.inhibited() & RollStabilization::Baseline);
    printf("PASS: %u direction/ring/endpoint geometry cases, invalid baseline bypass\n", checked);
}

static void live_maneuver_and_configuration()
{
    Fixture f;
    f.drives[0].angle_deg = 90;
    f.ready();
    const uint16_t input[3] {1600, 1500, 1765};
    const Components::VSP1Params p {100, 1500, 1500, 0, 90};
    uint16_t base[3], out[3];
    Components::vsp1_loop(input, base, p);
    Components::vsp1_loop(input, out, p, &f.controller);
    assert(out[0] == base[0]); // no second application of THR_ANG
    assert(out[1] > base[1]); // correction still on post-limiter +Y
    assert(out[2] == 1765);
    f.tick(110);
    const uint16_t moved[3] {1500, 1600, 1634};
    Components::vsp1_loop(moved, base, p);
    Components::vsp1_loop(moved, out, p, &f.controller);
    assert(out[0] == base[0]); // maneuver changed during an ongoing burst
    assert(out[1] > base[1]);
    assert(out[2] == 1634);
    f.drives[0].angle_deg = 180;
    f.tick(120);
    assert(same(f.apply(), f.neutral)); // live angle edit resets pulse

    Fixture slew;
    slew.settings.slew = 1;
    slew.ready();
    assert(slew.apply().x_coordinate == 1510);
    slew.tick(110);
    // An abrupt full maneuver must suppress VRS immediately, despite slow slew.
    Coordinates edge {1600, 1500};
    assert(same(slew.controller.apply(0, edge, slew.limits, true), edge));

    Fixture invalid;
    invalid.settings.d = std::numeric_limits<float>::quiet_NaN();
    invalid.ready();
    assert(same(invalid.apply(), invalid.neutral));
    assert(invalid.controller.inhibited() & RollStabilization::Configuration);
    invalid.settings.d = 0.1f;
    invalid.drives[1].sign = 1;
    invalid.tick(110);
    assert(same(invalid.apply(), invalid.neutral));
    assert(invalid.controller.inhibited() & RollStabilization::Configuration);
    puts("PASS: simultaneous maneuvering, post-limiter direction, RPM passthrough, slew and live configuration");
}

static float decay(bool enabled)
{
    Fixture f;
    f.settings.enabled = enabled;
    f.settings.on_ms = 250;
    f.settings.off_ms = 100;
    f.settings.slew = 5;
    f.settings.deadband = 0.001f;
    f.limits = setupLimiter(500, 1500, 1500);
    float roll = 10, rate = 0, energy = 0;
    for (uint32_t time = 0; time < 20000; time += 10) {
        f.sample.roll_deg = roll;
        f.sample.rate_dps = rate;
        f.tick(time);
        float moment = 0;
        for (uint8_t drive = 0; drive < 2; drive++) {
            f.apply(drive);
            moment += f.drives[drive].sign * f.controller.status(drive).command;
        }
        // Deliberately simple damped roll oscillator. This exercises closed-loop
        // sign/phase and duty accounting; it is NOT a hydrodynamic vessel model.
        const float acceleration = -9*roll - 0.12f*rate + 20*moment;
        rate += acceleration * 0.01f;
        roll += rate * 0.01f;
        energy += roll*roll * 0.01f;
        assert(isfinite(roll));
        assert(f.controller.status(0).duty >= 0 && f.controller.status(0).duty <= 1);
        assert(f.controller.status(1).duty >= 0 && f.controller.status(1).duty <= 1);
    }
    return energy;
}

int main()
{
    disabled_equivalence();
    signs_timing_and_input();
    hysteresis_and_reversal();
    geometry();
    live_maneuver_and_configuration();
    const float uncontrolled = decay(false);
    const float controlled = decay(true);
    assert(controlled < uncontrolled * 0.6f);
    printf("PASS: illustrative roll oscillator energy %.2f -> %.2f degree^2 seconds\n", uncontrolled, controlled);
}
