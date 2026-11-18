#include "HeadingHold.h"
#include "VspMapper.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <initializer_list>
using namespace Vektor2;
struct Fixture {
    HeadingHold h;
    HeadingSettings s;
    HeadingSample sample;
    HeadingActuation actuator;
    uint32_t now = 0;
    Fixture() {
        s.enabled = true; s.idle_ms = 100; s.p = 0.1f; s.heading_deadband = 0;
        s.maximum = 1; s.slew = 100;
        sample.compass_valid = sample.attitude_valid = sample.control_valid = true;
        sample.heading_deg = 90;
        actuator.available = true;
    }
    void tick(uint32_t t) { now = t; h.update(t,s,sample,actuator); }
    void next() { tick(now+10); }
    void ready(uint32_t t=0) { tick(t); tick(t+100); }
    void demand() { sample.heading_deg -= 5; next(); }
    void zero() { assert(h.command() == 0 && h.integral() == 0); }
};
int main()
{
    Fixture f;
    f.tick(0); assert(f.h.state() == HeadingHold::State::Waiting);
    f.tick(90); assert(f.h.wait_ms() == 10);
    f.sample.heading_deg = 110;
    f.tick(100); assert(f.h.target() == 110 && f.h.state() == HeadingHold::State::Hold);
    f.demand(); assert(f.h.command() > 0.49f);
    f.actuator.manual = true; f.next();
    assert(f.h.state() == HeadingHold::State::Manual); f.zero();
    f.actuator.manual = false; f.next();
    f.tick(f.now+90); assert(f.h.state() == HeadingHold::State::Waiting);
    f.actuator.manual = true; f.next(); f.actuator.manual = false; f.next();
    f.sample.heading_deg = 210; f.tick(f.now+100);
    assert(f.h.target() == 210); f.zero();
    puts("PASS: capture at end of uninterrupted neutral interval and immediate manual release");

    Fixture wrap;
    wrap.sample.heading_deg = 359; wrap.ready(); wrap.sample.heading_deg = 1; wrap.next();
    assert(fabsf(wrap.h.error()+2) < 0.001f && wrap.h.command() < -0.19f);
    Fixture damping;
    damping.s.p = 0; damping.s.d = 0.01f; damping.s.heading_deadband = 10;
    damping.sample.rate_dps = 10; damping.ready(); damping.next();
    assert(damping.h.command() < -0.09f);
    damping.sample.rate_dps = 0; damping.next();
    assert(damping.h.rate() > 0 && damping.h.rate() < 10);
    puts("PASS: heading wrap, rate damping inside deadband and rate filtering");

    for (int cause=0; cause<12; cause++) {
        Fixture g; g.ready(); g.demand();
        switch(cause) {
        case 0: g.sample.compass_valid = false; break;
        case 1: g.sample.attitude_valid = false; break;
        case 2: g.sample.control_valid = false; break;
        case 3: g.actuator.available = false; break;
        case 4: g.actuator.configuration_valid = false; break;
        case 5: g.sample.heading_deg = NAN; break;
        case 6: g.sample.rate_dps = INFINITY; break;
        case 7: g.sample.routes_valid = false; break;
        case 8: g.sample.yaw_reset = true; break;
        case 9: g.tick(g.now+101); break;
        case 10: g.actuator.slew = NAN; break;
        case 11: g.actuator.maximum = -1; break;
        }
        if (cause != 9) { g.next(); }
        assert(g.h.state() == HeadingHold::State::Inhibited); g.zero();
        g.sample = Fixture{}.sample; g.actuator = Fixture{}.actuator;
        g.next(); assert(g.h.state() == HeadingHold::State::Waiting);
        g.tick(g.now+100); assert(g.h.state() == HeadingHold::State::Hold);
    }
    Fixture clock; clock.ready(UINT32_MAX-50); clock.demand(); assert(clock.h.command() > 0);
    clock.s.enabled = false; clock.next(); clock.zero(); assert(!clock.h.enabled());
    clock.s.enabled = true; clock.ready(clock.now+10); clock.demand();
    clock.actuator.revision++; clock.next(); clock.zero(); assert(clock.h.state() == HeadingHold::State::Waiting);
    puts("PASS: sensor, control, actuator and route faults, configuration edits, clock wrap and recovery");

    Fixture limit; limit.s.i = 0.2f; limit.actuator.maximum = 0.1f; limit.ready(); limit.demand();
    assert(fabsf(limit.h.command()-0.1f) < 1e-6f);
    assert(limit.h.inhibited() & HeadingHold::Travel);
    for (int i=0;i<1000;i++) { limit.next(); assert(fabsf(limit.h.integral()) < 1e-6f); }
    Fixture integral; integral.s.p=0; integral.s.i=1; integral.s.imax=0.25f;
    integral.actuator.slew=0.5f; integral.ready(); integral.demand();
    assert(integral.h.integral() > 0 && integral.h.command() <= 0.00501f);
    for (int i=0;i<100;i++) { integral.next(); }
    assert(integral.h.integral() <= 0.25001f && integral.h.command() >= 0.249f);
    integral.sample.heading_deg=95;
    for (int i=0;i<100;i++) { integral.next(); }
    assert(integral.h.integral() < 0);
    puts("PASS: common envelope/slew anti-windup, I-only startup and unwinding");

    for (int dir : {0,1}) {
        for (int angle : {0,45,90,180,270,359}) {
            for (int radius : {25,100,400}) {
                VspMapper::Config c {VspMapper::Rotation(dir), float(angle), 1500,1500,int16_t(radius)};
                for (float effort : {-1.0f,-0.2f,0.0f,0.2f,1.0f}) {
                    auto port=VspMapper::map(c,{effort,0});
                    auto starboard=VspMapper::map(c,{-effort,0});
                    assert(port.valid && starboard.valid);
                    assert(fabsf(port.applied.vector.forward-effort) < 1.5f/radius);
                    assert(fabsf(starboard.applied.vector.forward+effort) < 1.5f/radius);
                    assert(fabsf(port.applied.vector.starboard) < 1.5f/radius);
                }
            }
        }
    }
    puts("PASS: opposed body-frame yaw vectors through CW/CCW and mounting-angle mapper variants");
}
