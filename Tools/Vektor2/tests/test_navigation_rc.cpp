// One assignable NavMode VFN slot selects observation-only navigation maths.
// No controller demand or action path is connected to propulsion outputs.
#include "Functions.h"
#include "NavigationSupervisor.h"
#include <cassert>
#include <cstdint>
#include <cstdio>

using namespace Vektor2;

struct Fixture {
    Functions actions;
    FunctionSettings settings[Functions::count]{};
    FunctionBinding bindings[Functions::count]{};
    FunctionInput inputs[Functions::count]{};
    uint32_t now = 0;
    bool rc_valid = true;

    void slot(uint8_t idx, Functions::Type type = Functions::Type::NavMode,
              uint16_t low = 1300, uint16_t high = 1700)
    {
        settings[idx].enable = 1;
        settings[idx].mode = 0; // NavMode is always level mode
        settings[idx].type = int8_t(type);
        settings[idx].low = low;
        settings[idx].high = high;
        bindings[idx].routed = true;
        inputs[idx].valid = true;
        inputs[idx].pulse_us = 1000;
    }

    Functions::NavigationSelection tick()
    {
        actions.prepare(now, settings, bindings);
        actions.sample(inputs);
        now += 10;
        return actions.navigation_selection(rc_valid);
    }
};

static void three_position_rc_loss_fails_off()
{
    Fixture f;
    auto s = f.tick();
    assert(!s.assigned && s.valid && s.mode == 0);
    f.slot(0); // REQUIRES a 3-pos switch, one VFN slot, one RC route.
    s = f.tick();
    assert(s.assigned && s.valid && s.mode == 0);
    f.inputs[0].pulse_us = 1500;
    assert(f.tick().mode == 1); // physically confirmed center setting
    f.inputs[0].pulse_us = 2000;
    assert(f.tick().mode == 2);
    assert(f.actions.status(0).flags & Functions::Requested);
    for (int i = 0; i < 10; i++) {
        assert(f.tick().mode == 2);
    }
    f.inputs[0].pulse_us = 1000;
    assert(f.tick().mode == 0);
    assert(!(f.actions.status(0).flags & Functions::Requested));
    f.inputs[0].pulse_us = 2000;
    assert(f.tick().mode == 2);
    f.rc_valid = false;
    s = f.tick();
    assert(s.assigned && s.mode == 0);
    f.rc_valid = true;
    f.inputs[0].pulse_us = 1700; // no stable position after RC loss
    assert(f.tick().mode == 0); // guard must NOT resurrect old DP
    f.inputs[0].valid = false;
    s = f.tick();
    assert(s.assigned && !s.valid && s.mode == 0);
    assert(f.actions.status(0).errors & Functions::Input);
    f.inputs[0].valid = true;
    f.inputs[0].pulse_us = 2000;
    assert(f.tick().mode == 2);
    f.inputs[0].pulse_us = 799;
    assert(!f.tick().valid && f.actions.navigation_selection(true).mode == 0);
    f.inputs[0].pulse_us = 2201;
    assert(!f.tick().valid && f.actions.navigation_selection(true).mode == 0);
    f.inputs[0].pulse_us = 1000;
    assert(f.tick().mode == 0);
}

static void three_position_boundaries_and_no_change_intervals()
{
    Fixture f;
    f.slot(0); // LOW=1300, HIGH=1700, +/-50us guard bands.
    struct Case { uint16_t pulse; int8_t mode; };
    // 800..1249 = Free. 1250..1349 = no change.
    // 1350..1649 = Heading. 1650..1749 = no change.
    // 1750..2200 = DP. In no-change regions retain previous mode.
    const Case ascending[] = {
        {800, 0}, {1000, 0}, {1249, 0}, {1250, 0}, {1299, 0},
        {1300, 0}, {1349, 0}, {1350, 1}, {1500, 1}, {1649, 1},
        {1650, 1}, {1699, 1}, {1700, 1}, {1749, 1}, {1750, 2},
        {2000, 2}, {2200, 2}
    };
    for (const auto& c : ascending) {
        f.inputs[0].pulse_us = c.pulse;
        const auto s = f.tick();
        assert(s.assigned && s.valid && !s.conflict && s.mode == c.mode);
        const bool nonfree = c.mode != 0;
        assert(bool(f.actions.status(0).flags & Functions::Matched) == nonfree);
        assert(bool(f.actions.status(0).flags & Functions::Requested) == nonfree);
    }
    // Reversing through BOTH no-change bands does not switch early.
    const Case descending[] = {
        {1749, 2}, {1700, 2}, {1650, 2}, {1649, 1},
        {1350, 1}, {1349, 1}, {1300, 1}, {1250, 1}, {1249, 0}
    };
    for (const auto& c : descending) {
        f.inputs[0].pulse_us = c.pulse;
        assert(f.tick().mode == c.mode);
        assert(bool(f.actions.status(0).flags & Functions::Requested) == (c.mode != 0));
    }
    // Change thresholds, invalidate the previous latch and recapture only
    // when a new stable switch position is observed.
    f.inputs[0].pulse_us = 2000;
    assert(f.tick().mode == 2);
    f.settings[0].low = 1400;
    f.settings[0].high = 1800;
    f.inputs[0].pulse_us = 1400; // new LOW boundary, now no change
    assert(f.tick().mode == 0);
    f.inputs[0].pulse_us = 1500;
    assert(f.tick().mode == 1);
    f.inputs[0].pulse_us = 2000;
    assert(f.tick().mode == 2);
}

static void invalid_config_and_migration_fail_off()
{
    Fixture f;
    f.slot(0);
    f.inputs[0].pulse_us = 2000;
    assert(f.tick().mode == 2);
    f.settings[0].mode = int8_t(Functions::Mode::Toggle);
    auto s = f.tick();
    assert(!s.valid && s.mode == 0);
    assert(f.actions.status(0).errors & Functions::Configuration);
    f.settings[0].mode = int8_t(Functions::Mode::Level);
    f.settings[0].low = f.settings[0].high;
    s = f.tick();
    assert(!s.valid && s.mode == 0);
    f.settings[0].low = 1300;
    assert(f.tick().mode == 2);
    // Reject NavMode configurations that cannot fit 3 stable positions
    // and two +/-50us no-change switch-transition zones.
    f.settings[0].high = 1399;
    s = f.tick();
    assert(!s.valid && s.mode == 0);
    assert(f.actions.status(0).errors & Functions::Configuration);
    f.settings[0].high = 1700;
    assert(f.tick().mode == 2);
    f.settings[0].low = 850;
    s = f.tick();
    assert(!s.valid && s.mode == 0);
    f.settings[0].low = 1300;
    assert(f.tick().mode == 2);
    f.bindings[0].routed = false;
    s = f.tick();
    assert(!s.valid && s.mode == 0);
    f.bindings[0].routed = true;
    f.bindings[0].error = true;
    s = f.tick();
    assert(!s.valid && s.mode == 0);
    f.bindings[0].error = false;
    assert(f.tick().mode == 2);
    f.settings[0].enable = 0;
    s = f.tick();
    assert(s.assigned && !s.valid && s.mode == 0);
    f.settings[0].enable = 1;
    assert(f.tick().mode == 2);

    // A stored M3.8 TYPE 5/6 must NOT be silently reinterpreted as TYPE 7.
    f.settings[0].type = int8_t(Functions::Type::RetiredNavHeading);
    s = f.tick();
    assert(s.assigned && !s.valid && s.mode == 0);
    assert(f.actions.status(0).errors & Functions::Configuration);
    f.settings[0].type = int8_t(Functions::Type::RetiredNavDP);
    s = f.tick();
    assert(s.assigned && !s.valid && s.mode == 0);
    assert(f.actions.status(0).errors & Functions::Configuration);
    f.settings[0].type = int8_t(Functions::Type::NavMode);
    assert(f.tick().mode == 2);

    // Even two valid NavMode slots are forbidden: one action, one RC source.
    f.slot(1);
    f.inputs[1].pulse_us = 2000;
    s = f.tick();
    assert(s.conflict && s.assigned && s.mode == 0);
    f.settings[1].type = int8_t(Functions::Type::None);
    f.settings[1].enable = 0;
    assert(f.tick().mode == 2);
    // A legacy type cannot coexist silently with the new NavMode.
    f.slot(1, Functions::Type::RetiredNavDP);
    s = f.tick();
    assert(s.conflict && !s.valid && s.mode == 0);
    f.settings[1].type = 0;
    f.settings[1].enable = 0;
    assert(f.tick().mode == 2);

    f.settings[0].type = 0;
    f.settings[0].enable = 0;
    s = f.tick();
    assert(!s.assigned && s.valid && s.mode == 0);
}

static void timing_fault_fails_off()
{
    Fixture f;
    f.slot(0);
    f.inputs[0].pulse_us = 2000;
    assert(f.tick().mode == 2);
    f.now += 101;
    auto s = f.tick();
    assert(!s.valid && s.mode == 0);
    assert(f.actions.status(0).errors & Functions::Timing);
    assert(f.tick().mode == 2);
}

static void calculations_are_not_actuator_authority()
{
    NavigationSupervisor supervisor;
    NavigationSupervisor::Input in{};
    in.requested_mode = 2;
    in.enabled = in.heading_enabled = in.dp_enabled = true;
    in.rc_valid = in.heading_valid = in.position_valid = in.velocity_valid = true;
    in.heading_command_valid = true;
    in.heading_target_valid = in.position_target_valid = true;
    // Regardless of valid controller maths, no output owner exists in M3.8.
    in.output_ready = in.yaw_owned = in.translation_owned = false;
    supervisor.update(in);
    assert(supervisor.status().state == NavigationSupervisor::State::HeadingWait);
    assert(supervisor.status().reason_bits & NavigationSupervisor::OutputNotReady);
    in.requested_mode = 1;
    supervisor.update(in);
    assert(supervisor.status().state == NavigationSupervisor::State::HeadingWait);
    in.requested_mode = 0;
    supervisor.update(in);
    assert(supervisor.status().state == NavigationSupervisor::State::Free);
}

int main()
{
    three_position_rc_loss_fails_off();
    three_position_boundaries_and_no_change_intervals();
    invalid_config_and_migration_fail_off();
    timing_fault_fails_off();
    calculations_are_not_actuator_authority();
    puts("M3.8.1 NavMode 3-position guard-band and observation-only tests passed");
}
