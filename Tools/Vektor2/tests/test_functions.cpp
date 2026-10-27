#include "Functions.h"

#include <assert.h>
#include <stdio.h>

using namespace Vektor2;

struct Fixture {
    Functions functions;
    FunctionSettings settings[Functions::count];
    FunctionBinding bindings[Functions::count];
    FunctionInput inputs[Functions::count];
    uint32_t time = 0;
    bool roll = true;
    bool heading = true;

    void assign(uint8_t slot = 0, Functions::Type type = Functions::Type::Roll)
    {
        settings[slot].enable = 1;
        settings[slot].type = int8_t(type);
        bindings[slot].routed = true;
        inputs[slot].valid = true;
        inputs[slot].pulse_us = 1800;
    }
    void tick()
    {
        functions.prepare(time, settings, bindings);
        // This is the production routing order: apply old requests, then
        // calculate producers once, then sample their current outputs.
        roll = functions.permits(Functions::Type::Roll);
        heading = functions.permits(Functions::Type::Heading);
        functions.sample(inputs);
        time += 10;
    }
    const Functions::Status& status(uint8_t slot = 0) const { return functions.status(slot); }
    bool requested(uint8_t slot = 0) const { return status(slot).flags & Functions::Requested; }
};

static void levels_and_ranges()
{
    Fixture unused;
    unused.tick();
    assert(unused.roll && unused.heading && !unused.functions.configured());
    const int16_t ranges[][2] {{800, 800}, {800, 2200}, {1700, 2000}, {2200, 2200}};
    for (const auto& range : ranges) {
        Fixture f;
        f.assign(9);
        f.settings[9].low = range[0];
        f.settings[9].high = range[1];
        bool previous = false;
        for (uint16_t pulse = 799; pulse <= 2201; pulse++) {
            f.inputs[9].pulse_us = pulse;
            f.tick();
            const bool valid = pulse >= 800 && pulse <= 2200;
            const bool matched = valid && pulse >= range[0] && pulse <= range[1];
            assert(f.roll == previous && f.heading);
            assert(f.requested(9) == matched);
            assert(bool(f.status(9).flags & Functions::Matched) == matched);
            assert(bool(f.status(9).flags & Functions::Valid) == valid);
            assert(f.status(9).errors == (valid ? 0 : Functions::Input));
            previous = matched;
        }
    }
    puts("PASS: full inclusive 800..2200 range, single-value intervals, slot 10 and one-cycle latency");
}

static void toggles_and_resets()
{
    Fixture f;
    f.assign();
    f.settings[0].mode = int8_t(Functions::Mode::Toggle);
    f.tick();
    assert(!f.roll && !f.requested()); // Boot in range cannot invent an entry.
    f.inputs[0].pulse_us = 1600;
    f.tick();
    assert(f.status().flags & Functions::Armed);
    f.inputs[0].pulse_us = 1700;
    f.tick();
    assert(!f.roll && f.requested());
    for (int n = 0; n < 10; n++) {
        f.tick();
        assert(f.roll && f.requested());
    }
    f.inputs[0].pulse_us = 2001;
    f.tick();
    assert(f.requested());
    f.inputs[0].pulse_us = 2000;
    f.tick();
    assert(f.roll && !f.requested());
    f.tick();
    assert(!f.roll);

    // An unrelated slot edit does not discard an existing toggle request.
    f.inputs[0].pulse_us = 1600;
    f.tick();
    f.inputs[0].pulse_us = 1800;
    f.tick();
    f.assign(1, Functions::Type::Heading);
    f.tick();
    assert(f.roll && f.requested());

    f.inputs[0].valid = false;
    f.tick();
    assert(!f.requested() && f.status().errors == Functions::Input);
    f.inputs[0].valid = true;
    f.tick();
    assert(!f.roll && !f.requested());
    f.inputs[0].pulse_us = 1500;
    f.tick();
    f.inputs[0].pulse_us = 1800;
    f.tick();
    assert(f.requested());
    f.bindings[0].revision++;
    f.tick();
    assert(!f.roll && !f.requested());
    f.inputs[0].pulse_us = 1500;
    f.tick();
    f.inputs[0].pulse_us = 1800;
    f.tick();
    f.settings[0].high = 1900;
    f.tick();
    assert(!f.roll && !f.requested());
    f.settings[0].enable = 0;
    f.tick();
    assert(!f.roll && !(f.status().flags & (Functions::Requested | Functions::Armed)));
    assert((f.status().flags & Functions::Valid) && f.status().pulse_us == 1800);
    f.settings[0].enable = 1;
    f.tick();
    assert(!f.requested());
    puts("PASS: toggle entries, boot/recovery arming, input loss, slot/route edits and disable reset");
}

static void arbitration_and_faults()
{
    Fixture f;
    f.assign();
    f.assign(1);
    f.inputs[0].valid = false;
    f.tick();
    f.tick();
    assert(f.roll); // One invalid source cannot overrule another valid request.
    f.settings[1].enable = 0;
    f.tick();
    assert(!f.roll); // Assigned disabled slots are not legacy pass-through.
    f.bindings[0].routed = false;
    f.tick();
    assert(f.status().errors & Functions::Unrouted);
    f.bindings[0].routed = true;
    f.bindings[0].error = true;
    f.inputs[0].valid = true;
    f.tick();
    assert(!f.roll && f.status().errors == Functions::Routing);
    f.bindings[0].error = false;
    f.tick();
    f.tick();
    assert(f.roll);
    for (uint8_t field = 0; field < 7; field++) {
        const auto saved = f.settings[0];
        switch (field) {
        case 0: f.settings[0].enable = 2; break;
        case 1: f.settings[0].mode = 2; break;
        case 2: f.settings[0].low = 799; break;
        case 3: f.settings[0].high = 2201; break;
        case 4: f.settings[0].low = 2001; break;
        case 5: f.settings[0].type = 5; break;
        case 6: f.settings[0].type = -1; break;
        }
        f.tick();
        assert(!f.requested() && f.status().errors == Functions::Configuration);
        f.settings[0] = saved;
        f.tick();
        f.tick();
        assert(f.roll);
    }
    f.settings[0].type = f.settings[1].type = 0;
    f.settings[0].enable = f.settings[1].enable = 0;
    f.tick();
    assert(f.roll && f.heading && !f.functions.configured());
    puts("PASS: OR arbitration, independent actions, invalid settings/routing and legacy fallback");
}

static void timing_and_feedback()
{
    Fixture f;
    f.assign();
    f.tick();
    f.time = 100;
    f.tick();
    assert(f.roll && !f.status().errors);
    f.time = 201;
    f.tick();
    assert(!f.roll && f.status().errors == Functions::Timing && !f.requested());
    f.tick();
    assert(!f.roll && f.requested());
    f.tick();
    assert(f.roll);

    Fixture wrap;
    wrap.assign();
    wrap.time = UINT32_MAX - 5;
    wrap.tick();
    wrap.tick();
    assert(wrap.roll && !wrap.status().errors);

    // A producer dependent on its own action is a discrete feedback path:
    // one producer run and one evaluation per cycle, never recursive reruns.
    Fixture feedback;
    feedback.assign();
    for (uint32_t n = 0; n < 20; n++) {
        feedback.functions.prepare(n * 10, feedback.settings, feedback.bindings);
        const bool gate = feedback.functions.permits(Functions::Type::Roll);
        assert(gate == bool(n % 2));
        feedback.inputs[0].pulse_us = gate ? 1500 : 1800;
        feedback.functions.sample(feedback.inputs);
    }
    puts("PASS: 100ms timing limit, clock wrap and bounded one-cycle feedback");
}

static void thruster_gates()
{
    Fixture f;
    assert(f.functions.permits(Functions::Type::Bow));
    assert(f.functions.permits(Functions::Type::Stern));
    f.assign();
    f.settings[0].type = int8_t(Functions::Type::Bow);
    f.tick();
    f.tick();
    assert(f.functions.permits(Functions::Type::Bow));
    f.inputs[0].valid = false;
    f.tick();
    assert(!f.functions.permits(Functions::Type::Bow));
    assert(f.functions.permits(Functions::Type::Stern));
    f.settings[0].type = int8_t(Functions::Type::Stern);
    f.settings[0].enable = 0;
    f.tick();
    assert(f.functions.permits(Functions::Type::Bow));
    assert(!f.functions.permits(Functions::Type::Stern));
    puts("PASS: independent bow/stern gates, loss and disabled assignment");
}

int main()
{
    thruster_gates();
    levels_and_ranges();
    toggles_and_resets();
    arbitration_and_faults();
    timing_and_feedback();
    return 0;
}
