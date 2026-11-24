#include "VspVectorControl.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

using C = Vektor2::VspVectorControl;
using M = Vektor2::VspMapper;
int main()
{
    C control;
    C::Settings s {0, 30, 2, {M::Rotation::CCW, 0, 1500, 1500, 100}};
    uint16_t x = 0, y = 0;
    uint32_t now = 0, revision = 0;
    auto update = [&](uint16_t f, uint16_t r, bool valid = true) {
        now += 10;
        return control.update(now, revision, s, f, r, valid, x, y);
    };
    assert(!update(2000, 1500)); // untouched legacy startup
    s.mode = 1;
    assert(update(2000, 1500) && x == 1500 && y == 1500);
    assert(control.state() == C::State::Waiting);
    assert(update(1500, 1500) && control.state() == C::State::Active);
    update(2000, 1500); assert(x == 1498 && y == 1500); // 2% per tick
    for (int i=0; i<60; i++) { update(2000,1500); }
    assert(x == 1400 && y == 1500);
    const auto generation = control.revision();
    update(2050,1500);
    assert(x == 1400 && y == 1500 && control.heading_ready());
    update(1000,1800);
    const auto endpoint = control.requested();
    update(950,1800);
    assert(control.heading_ready() && control.revision() == generation);
    assert(control.requested().forward == endpoint.forward &&
           control.requested().starboard == endpoint.starboard);
    update(1500,1500); assert(x == 1500 && y == 1500); // immediate neutral
    update(1500,2000);
    assert(x == 1500 && y == 1498);
    update(2000,1500,false); assert(x == 1500 && y == 1500);
    assert(control.state() == C::State::InputLost);
    update(2000,1500); assert(control.state() == C::State::Waiting);
    update(1500,1500);
    update(2000,1500); assert(x < 1500);
    revision++;
    update(2000,1500); assert(x == 1500 && control.state() == C::State::Waiting);
    update(1500,1500);
    now += 101;
    update(2000,1500); assert(x == 1500 && control.state() == C::State::Waiting);
    s.mapper.rotation = M::Rotation::CW;
    update(2000,1500); assert(x == 1500);
    update(1500,1500);
    update(2000,1500); assert(x == 1502);
    s.mapper.installation_deg = 90;
    update(1500,1500);
    update(2000,1500); assert(x == 1500 && y == 1498);
    s.mode = 0;
    assert(update(2000,1500) && x == 1500 && y == 1500);
    assert(update(1500,1500)); // one complete neutral cycle before legacy
    assert(!update(2000,1500));
    s.mode = 2;
    assert(update(2000,1500) && control.state() == C::State::Invalid);
    s.mode = 1; s.slew = NAN;
    update(1500,1500); assert(control.state() == C::State::Invalid);
    s.slew = 2; s.mapper.radius = 0;
    update(1500,1500); assert(control.state() == C::State::Invalid);
    s.mapper.radius = 100;
    update(1500,1500); assert(control.state() == C::State::Active);
    update(0,1500); assert(control.state() == C::State::InputLost);
    // Selected heading vectors bypass manual slew once, but takeover clears
    // automatic history before applying the manual rate limit.
    C shared;
    s = {1, 30, 2, {M::Rotation::CW, 0, 1500, 1500, 100}};
    shared.prepare(0,0,s,1500,1500,true); shared.apply(x,y);
    assert(!shared.heading_ready());
    shared.prepare(10,0,s,1500,1500,true);
    const M::Vector heading {0.2f,0};
    assert(shared.heading_ready() && shared.apply(x,y,&heading));
    assert(x == 1520 && y == 1500); // no second slew stage
    shared.prepare(20,0,s,1500,2000,true);
    assert(!shared.apply(x,y,&heading)); // sticks win, even if caller supplies assistance
    assert(x == 1500 && y == 1502);
    shared.prepare(30,0,s,1500,1500,true); shared.apply(x,y);
    assert(x == 1500 && y == 1500);
    shared.prepare(40,0,s,1500,1500,true); shared.apply(x,y,&heading);
    shared.prepare(50,0,s,1500,1500,false);
    assert(!shared.apply(x,y,&heading) && x == 1500 && y == 1500);
    shared.prepare(60,0,s,1500,1500,true); shared.apply(x,y);
    shared.follow_allocation(heading);
    shared.update(70,0,s,1500,1500,true,x,y);
    assert(x == 1518 && y == 1500); // Free slews from the published allocation
    shared.update(80,0,s,1500,2000,true,x,y);
    assert(x > 1500 && y > 1500 && shared.heading_ready());
    shared.update(90,0,s,1500,1500,false,x,y);
    assert(x == 1500 && y == 1500); // source loss still stops the handover
    puts("PASS: shared manual/heading map stage, single slew and immediate automatic release");
    puts("PASS: vector ownership, neutral transitions, slew, loss, stalls, route/config edits and mapping");
}
