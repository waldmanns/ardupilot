#include "VspAllocator.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using A=Vektor2::VspAllocator;
using V=Vektor2::VesselThrustAllocator;
static bool eq(float a,float b) { return std::fabs(a-b)<1e-4f; }
static void check(const A::Config& c,const A::Result& r) {
    const auto& g=c.geometry;
    assert(eq(r.achieved.surge,g.vsp1.force*r.vsp1.forward+g.vsp2.force*r.vsp2.forward));
    assert(eq(r.achieved.sway,g.vsp1.force*r.vsp1.starboard+g.vsp2.force*r.vsp2.starboard+(c.bow?g.bow_force*r.bow:0)));
    assert(eq(r.achieved.yaw,g.vsp1.force*(g.vsp1.x*r.vsp1.starboard-g.vsp1.y*r.vsp1.forward)+g.vsp2.force*(g.vsp2.x*r.vsp2.starboard-g.vsp2.y*r.vsp2.forward)+(c.bow?g.bow_x*g.bow_force*r.bow:0)));
    assert(std::hypot(r.vsp1.forward,r.vsp1.starboard)<=1.00001);
    assert(std::hypot(r.vsp2.forward,r.vsp2.starboard)<=1.00001);
}
int main() {
    A::Config c;
    c.geometry.layout=V::Layout::VWT;
    c.geometry.vsp1={1.5,-1,1}; c.geometry.vsp2={1.5,1,1};
    c.max_surge=c.max_sway=c.max_yaw=1;
    c.slew1=c.slew2=2;
    A::Input i; i.selected=i.pair_ready=i.centered=i.allowed=true; i.dt=.01f; i.mode=2; i.bow_ready=false;
    A a;
    i.demand={0,.8f,0};
    assert(a.update(c,i)); assert(!a.active()); assert(!(a.status()&A::DPOwned));
    assert(a.status()&A::SlewLimited); a.confirm(); assert(a.active()); assert(a.status()&A::DPOwned);
    for(int n=0;n<100;n++){assert(a.update(c,i));check(c,a.command());a.confirm();}
    auto r=a.command(); assert(!r.bow_allocated && r.bow==0); assert(eq(r.achieved.sway,.8f)); assert(eq(r.achieved.yaw,0));
    assert(r.vsp1.forward<0 && r.vsp2.forward>0);
    i.demand={1,0,0}; for(int n=0;n<100;n++){assert(a.update(c,i));check(c,a.command());a.confirm();}
    assert(eq(a.command().achieved.surge,1));
    i.demand={0,0,1}; for(int n=0;n<100;n++){assert(a.update(c,i));check(c,a.command());a.confirm();}
    assert(eq(a.command().achieved.yaw,1));
    i.demand={1,1,1}; for(int n=0;n<100;n++){assert(a.update(c,i));check(c,a.command());a.confirm();}
    i.demand={1,1,-1}; assert(a.update(c,i)); assert(a.status()&A::Saturated); a.confirm();
    const auto dp_command = a.command();
    i.mode=1; i.demand={-1,0,0};
    assert(a.update(c,i) && !a.guard() && !a.rearm() && a.dp_active());
    assert(std::hypot(a.command().vsp1.forward-dp_command.vsp1.forward,
                      a.command().vsp1.starboard-dp_command.vsp1.starboard) <= .02001f);
    a.confirm(); assert(a.active() && !a.dp_active());
    i.mode=2; assert(a.update(c,i) && !a.guard() && !a.rearm() && !a.dp_active());
    a.confirm(); assert(a.dp_active());
    i.mode=0; assert(!a.update(c,i) && !a.guard() && !a.rearm() && !a.active());
    i.mode=2; i.centered=false; assert(!a.update(c,i) && !a.rearm());
    i.mode=1; i.manual[0]={.4f,.3f}; i.manual[1]={.2f,.1f};
    assert(a.update(c,i) && !a.guard() && !a.rearm());
    assert(eq(a.command().vsp1.forward,.4f) && eq(a.command().vsp2.starboard,.1f));
    check(c,a.command()); a.confirm();
    i.allowed=false; assert(!a.update(c,i)); assert(a.guard() && a.rearm() && !a.active());
    i.allowed=true; assert(!a.update(c,i)); i.mode=0; assert(!a.update(c,i)); assert(a.guard()&&!a.rearm());
    i.mode=1; i.manual[0]={}; i.manual[1]={}; assert(a.update(c,i)); a.confirm();
    c.revision2++; assert(!a.update(c,i)); assert(a.rearm()); i.mode=0; a.update(c,i);
    i.mode=2; i.centered=true; i.override=true; assert(!a.update(c,i)); assert(a.status()&A::PilotOverride); i.override=false; i.mode=0; a.update(c,i);
    c.geometry.layout=V::Layout::AFT; c.geometry.vsp1.x=c.geometry.vsp2.x=-2; c.bow=true; c.geometry.bow_x=3; c.geometry.bow_force=.7f; c.bow_slew=2;
    i.mode=2; i.demand={0,.8f,0}; assert(!a.update(c,i)); assert(a.status()&A::BowUnavailable); i.bow_ready=true;
    for(int n=0;n<100;n++){assert(a.update(c,i));check(c,a.command());a.confirm();}
    assert(a.command().bow>0 && eq(a.command().achieved.yaw,0)); assert(a.status()&A::BowUsed);
    a.reject(); assert(!a.active() && a.guard()); assert(a.status()&A::OutputRejected);
    assert(!a.update(c,i)); assert(a.status()&A::OutputRejected); i.mode=0; a.update(c,i);
    assert(a.bow_guard());
    c.bow=false; i.mode=2; i.bow_ready=false; for(int n=0;n<100;n++){assert(a.update(c,i));check(c,a.command());a.confirm();}
    assert(a.command().vsp1.forward>0 && a.command().vsp2.forward<0); assert(!a.command().bow_allocated);
    c.geometry.vsp1.y=0; assert(!a.update(c,i)); assert(a.guard());
    puts("PASS: VWT and AFT allocation, direct mode handovers, slew, publication, override, and fault rearm");
}
