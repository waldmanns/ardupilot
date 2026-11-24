#include "ThrusterControl.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
using namespace Vektor2;

static void manual_and_gates()
{
    ThrusterControl bow, stern;
    ThrusterSettings s;
    s.enable = 1;
    s.mid = 1450;
    bow.update(0, s, false, true, true, 1800);
    assert(bow.output() == 1450 && bow.state() == ThrusterControl::State::Disabled);
    bow.update(10, s, true, true, true, 1800);
    assert(bow.output() == 1450 && bow.state() == ThrusterControl::State::NeutralWait);
    bow.update(20, s, true, true, true, 1450);
    bow.update(30, s, true, true, true, 1800);
    assert(bow.output() == 1459 && bow.manual_override());
    stern.update(30, s, false, true, true, 1800);
    assert(stern.output() == 1450); // independent gate
    bow.update(40, s, true, true, false, 1800);
    assert(bow.output() == 1450 && bow.state() == ThrusterControl::State::InputLost);
    bow.update(50, s, true, true, true, 1800);
    assert(bow.state() == ThrusterControl::State::NeutralWait);
    bow.update(60, s, true, true, true, 1450);
    bow.update(70, s, true, true, true, 2000);
    assert(bow.output() == 1459);
    bow.update(180, s, true, true, true, 2000);
    assert(bow.output() == 1450 && bow.state() == ThrusterControl::State::InputLost);
    bow.update(190, s, true, true, true, 2000);
    assert(bow.state() == ThrusterControl::State::NeutralWait);
    s.maximum = NAN;
    bow.update(200, s, true, true, true, 1450);
    assert(bow.state() == ThrusterControl::State::Configuration && bow.output() == 1450);
    const uint32_t invalid_revision = bow.revision();
    bow.update(210, s, true, true, true, 1450);
    assert(bow.revision() == invalid_revision); // invalid bow must not repeatedly reset shared heading
    s.maximum = 1;
    bow.update(UINT32_MAX - 5, s, false, true, true, 1450);
    bow.update(4, s, true, true, true, 1450);
    bow.update(14, s, true, true, true, 1800);
    assert(bow.output() == 1459);
    bow.reset(); // input-route edit must re-arm at neutral
    bow.update(24, s, true, true, true, 1800);
    assert(bow.state() == ThrusterControl::State::NeutralWait);
    puts("PASS: independent enables, neutral interlock, manual slew, source loss, gaps, edits and clock wrap");
}

static void heading_actuator_path()
{
    ThrusterSettings t;
    t.enable = t.heading_enable = 1;
    t.heading_maximum = 0.4f;
    ThrusterControl bow, stern;
    bow.update(0, t, true, true, true, 1500);
    t.heading_sign = -1;
    stern.update(0, t, true, true, true, 1500);
    bow.apply_heading(0.5f, true);
    stern.apply_heading(0.5f, true);
    assert(bow.output() == 1600 && stern.output() == 1400);
    assert(bow.state() == ThrusterControl::State::Heading);
    stern.update(10, t, false, true, true, 1500);
    stern.apply_heading(0.5f, true);
    assert(stern.output() == 1500 && bow.output() == 1600);
    bow.follow_heading_output();
    t.heading_sign = 1;
    bow.update(10, t, true, true, true, 1500);
    assert(bow.output() == 1590 && bow.state() == ThrusterControl::State::Manual);
    assert(fabsf(bow.heading_equivalent() - 0.45f) < 0.001f);
    bow.update(20, t, true, true, false, 1500);
    assert(bow.output() == 1500 && bow.state() == ThrusterControl::State::InputLost);
    puts("PASS: thruster actuator yaw signs, limits and readiness gate");
}
int main() { manual_and_gates(); heading_actuator_path(); }
