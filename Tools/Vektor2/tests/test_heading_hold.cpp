#include "HeadingHold.h"
#include "RollStabilization.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <initializer_list>
#include <limits>

using namespace Vektor2;

struct Fixture {
    HeadingHold controller;
    HeadingSettings settings;
    HeadingSample sample;
    HeadingDrive drives[2];
    uint32_t time = 0;

    Fixture()
    {
        settings.enabled = true;
        settings.idle_ms = 100;
        settings.p = 0.1f;
        settings.heading_deadband = 0;
        settings.maximum = 1;
        settings.slew = 100;
        sample.compass_valid = sample.attitude_valid = sample.control_valid = true;
        sample.heading_deg = 90;
        for (auto& drive : drives) {
            drive.inputs[0] = drive.inputs[1] = 1500;
            drive.inputs[2] = 1750;
            drive.baseline = {1500, 1500};
            drive.limits = setupLimiter(100, 1500, 1500);
            drive.sources_valid = true;
        }
    }

    void tick(uint32_t t)
    {
        time = t;
        controller.update(t, settings, sample, drives);
    }
    void next() { tick(time + 10); }
    void ready(uint32_t t = 0) { tick(t); tick(t + 100); }
    void demand() { sample.heading_deg -= 5; next(); }
    void zero() const
    {
        assert(controller.correction_us(0) == 0 && controller.correction_us(1) == 0);
        assert(controller.integral() == 0);
    }
};

static void state_and_capture()
{
    Fixture f;
    f.tick(0);
    assert(f.controller.state() == HeadingHold::State::Waiting);
    assert(f.controller.wait_ms() == 100);
    f.sample.heading_deg = 100;
    f.tick(90);
    assert(f.controller.wait_ms() == 10);
    f.sample.heading_deg = 110;
    f.tick(100);
    assert(f.controller.state() == HeadingHold::State::Hold);
    assert(f.controller.target() == 110);
    f.zero();
    f.demand();
    assert(f.controller.correction_us(0) >= 49);
    assert(f.controller.correction_us(1) >= 49);

    // Every individual input, on either side of its deadzone, releases BOTH.
    for (uint8_t n = 0; n < 2; n++) {
        for (uint8_t axis = 0; axis < 2; axis++) {
            for (int sign : {-1, 1}) {
                Fixture g;
                g.ready();
                g.drives[n].inputs[axis] = 1500 + sign * 30;
                g.demand();
                assert(g.controller.state() == HeadingHold::State::Hold);
                g.drives[n].inputs[axis] += sign;
                g.next();
                assert(g.controller.state() == HeadingHold::State::Manual);
                assert(g.controller.inhibited() == HeadingHold::Stick);
                g.zero();
                g.drives[n].inputs[axis] = 1500;
                g.next();
                assert(g.controller.wait_ms() == 100);
                g.tick(g.time + 90);
                assert(g.controller.state() == HeadingHold::State::Waiting);
                g.drives[n].inputs[axis] = 1531;
                g.next();
                g.drives[n].inputs[axis] = 1500;
                g.sample.heading_deg = 200;
                g.next();
                g.tick(g.time + 90);
                g.sample.heading_deg = 210;
                g.next();
                assert(g.controller.target() == 210);
                g.zero();
            }
        }
    }
    puts("PASS: all four deadzones, immediate paired release, interrupted wait, end-of-wait capture");
}

static void wrapping_signs_and_damping()
{
    for (int first : {0, 1}) {
        for (int second : {0, 1}) {
            Fixture f;
            f.settings.invert[0] = first;
            f.settings.invert[1] = second;
            f.sample.heading_deg = 359;
            f.ready();
            f.sample.heading_deg = 1;
            f.next();
            assert(fabsf(f.controller.error() + 2) < 0.001f);
            assert((f.controller.correction_us(0) > 0) == bool(first));
            assert((f.controller.correction_us(1) > 0) == bool(second));
            assert(fabsf(f.controller.correction_us(0)) >= 19);
            // Inversion never edits any original rod input or its geometry.
            assert(f.drives[0].inputs[1] == 1500 && f.drives[1].baseline.y_coordinate == 1500);
        }
    }
    Fixture f;
    f.settings.p = 0;
    f.settings.d = 0.01f;
    f.settings.heading_deadband = 10;
    f.sample.rate_dps = 10;
    f.ready();
    f.next();
    assert(f.controller.error() == 0);
    assert(f.controller.correction_us(0) <= -9); // rate damping at target
    f.sample.rate_dps = 0;
    f.next();
    assert(f.controller.rate() > 0 && f.controller.rate() < 10);
    puts("PASS: 359/0 wrap, four inversion combinations, independent rate damping and LPF");
}

static void invalidity_and_recovery()
{
    for (int cause = 0; cause < 10; cause++) {
        Fixture f;
        f.ready();
        f.demand();
        switch (cause) {
        case 0: f.sample.compass_valid = false; break;
        case 1: f.sample.attitude_valid = false; break;
        case 2: f.sample.control_valid = false; break;
        case 3: f.drives[1].inputs[2] = 0; break;
        case 4: f.drives[0].sources_valid = false; break;
        case 5: f.sample.heading_deg = std::numeric_limits<float>::quiet_NaN(); break;
        case 6: f.sample.rate_dps = std::numeric_limits<float>::infinity(); break;
        case 7: f.sample.routes_valid = false; break;
        case 8: f.sample.yaw_reset = true; break;
        case 9: f.tick(f.time + 101); break;
        }
        if (cause != 9) {
            f.next();
        }
        assert(f.controller.state() == HeadingHold::State::Inhibited);
        assert(f.controller.inhibited() != 0);
        f.zero();
        f.sample = HeadingSample{};
        f.sample.compass_valid = f.sample.attitude_valid = f.sample.control_valid = true;
        f.sample.heading_deg = 240;
        f.drives[0].sources_valid = true;
        f.drives[1].inputs[2] = 1800;
        f.next();
        assert(f.controller.state() == HeadingHold::State::Waiting);
        f.zero();
        f.tick(f.time + 100);
        assert(f.controller.target() == 240);
    }
    Fixture f;
    f.ready(UINT32_MAX - 50);
    f.demand();
    assert(f.controller.correction_us(0) > 0);
    f.settings.enabled = false;
    f.next();
    assert(!f.controller.enabled() && f.controller.state() == HeadingHold::State::Disabled);
    f.zero();
    f.settings.enabled = true;
    f.next();
    assert(f.controller.state() == HeadingHold::State::Waiting);
    f.tick(f.time + 100);
    f.demand();
    f.sample.routes_changed = true;
    f.next();
    assert(f.controller.state() == HeadingHold::State::Waiting);
    f.zero();
    puts("PASS: sensor/input/route loss, invalid numbers, yaw reset, stalls, clock wrap, live disable");
}

static void limits_and_integral()
{
    Fixture f;
    f.settings.i = 0.2f;
    f.ready();
    // Drive 2 has only 10% travel left; both drives must yield together.
    f.drives[1].baseline.y_coordinate = 1590;
    f.demand();
    assert(f.controller.correction_us(0) == 10 && f.controller.correction_us(1) == 10);
    assert(fabsf(f.controller.integral()) < 1.0e-6f);
    assert(f.controller.inhibited() & HeadingHold::Travel);
    assert(f.controller.travel_scale() < 0.21f);
    for (int n = 0; n < 1000; n++) {
        f.next();
        assert(fabsf(f.controller.integral()) < 1.0e-6f);
    }
    f.drives[1].baseline.y_coordinate = 1600;
    f.next();
    f.zero(); // reduced travel overrides slew immediately

    Fixture g;
    g.settings.p = 0;
    g.settings.i = 1;
    g.settings.imax = 0.25f;
    g.settings.slew = 0.5f;
    g.ready();
    g.demand();
    assert(g.controller.integral() > 0 && g.controller.integral() <= 0.00501f);
    for (int n = 0; n < 100; n++) {
        g.next();
    }
    assert(g.controller.integral() <= 0.25001f);
    assert(g.controller.correction_us(0) >= 24);
    g.sample.heading_deg = 95;
    for (int n = 0; n < 100; n++) {
        g.next();
    }
    assert(g.controller.integral() < 0);
    g.settings.invert[1] = 2;
    g.next();
    assert(g.controller.inhibited() & HeadingHold::Configuration);
    g.zero();
    puts("PASS: paired saturation, immediate travel reduction, anti-windup, I-only slew, IMAX");
}

static void geometry_sweep()
{
    unsigned cases = 0;
    for (int radius : {25, 100, 500}) {
        for (int cx : {1000, 1473, 1500, 2000}) {
            for (int cy : {1000, 1500, 1531, 2000}) {
                for (int angle : {0, 45, 90, 180, 270, 359}) {
                    for (int x : {1250, 1499, 1500, 1501, 1750}) {
                        for (int y : {1250, 1500, 1750}) {
                            for (int inverse : {0, 1}) {
                                Fixture f;
                                f.settings.stick_deadband = 250;
                                f.settings.invert[1] = inverse;
                                for (auto& drive : f.drives) {
                                    drive.inputs[0] = x;
                                    drive.inputs[1] = y;
                                    drive.limits = setupLimiter(radius, cx, cy);
                                    const Coordinates mapped {
                                        vsp_common_map(x, 1000, 2000, cx-radius, cx+radius),
                                        vsp_common_map(y, 1000, 2000, cy-radius, cy+radius)};
                                    drive.baseline = mapCoordinates(mapped, drive.limits, angle);
                                }
                                f.ready();
                                f.demand();
                                for (uint8_t n = 0; n < 2; n++) {
                                    const auto& drive = f.drives[n];
                                    const int32_t delta = f.controller.correction_us(n);
                                    if (delta != 0) {
                                        const int64_t dx = drive.baseline.x_coordinate - cx;
                                        const int64_t dy = drive.baseline.y_coordinate + delta - cy;
                                        assert(dx*dx + dy*dy <= int64_t(radius)*radius);
                                        assert(drive.baseline.y_coordinate + delta >= 1000);
                                        assert(drive.baseline.y_coordinate + delta <= 2000);
                                    }
                                }
                                cases++;
                            }
                        }
                    }
                }
            }
        }
    }
    printf("PASS: %u geometry cases, offset centers, ring and PWM endpoints, mapped axes\n", cases);
}

static void roll_priority()
{
    RollStabilization roll;
    RollSettings settings;
    settings.enabled = true;
    settings.d = 0.1f;
    settings.on_ms = settings.off_ms = 100;
    settings.slew = 100;
    settings.maneuver_limit = 1;
    RollDriveSettings drives[2];
    drives[0].angle_deg = 270; // negative Y conflicts with positive heading Y
    drives[0].sign = 1;
    drives[0].maximum = 0.5f;
    RollSample sample;
    sample.attitude_valid = sample.control_valid = true;
    sample.rate_dps = -5;
    roll.update(0, settings, drives, sample);
    roll.update(100, settings, drives, sample);
    const Coordinates base {1500, 1520};
    const Limiter limits = setupLimiter(100, 1500, 1500);
    auto result = roll.apply(0, base, limits, true, 1);
    assert(result.x_coordinate == base.x_coordinate && result.y_coordinate == base.y_coordinate);
    assert(roll.inhibited() & RollStabilization::Heading);
    // With no protected heading command, the existing VRS ray remains usable.
    roll.update(200, settings, drives, sample);
    result = roll.apply(0, base, limits, true);
    assert(result.y_coordinate < base.y_coordinate);
    puts("PASS: heading priority rejects opposing roll ray; VRS resumes without heading demand");
}

int main()
{
    state_and_capture();
    wrapping_signs_and_damping();
    invalidity_and_recovery();
    limits_and_integral();
    geometry_sweep();
    roll_priority();
}
