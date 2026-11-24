#pragma once
#include "VesselThrustAllocator.h"
#include <math.h>
#include <stdint.h>

namespace Vektor2 {
// Coordinated navigation owner. All commands are normalized; only Routing can
// confirm publication. No HAL or AP_Param dependencies.
class VspAllocator {
public:
    enum Status : uint16_t {
        Selected=1, PairReady=2, GeometryValid=4, BowUsed=8,
        Saturated=16, SlewLimited=32, HeadingOwned=64, DPOwned=128,
        RearmRequired=256, OutputRejected=512, NeutralGuard=1024,
        PilotOverride=2048, BowUnavailable=4096
    };
    struct Config {
        VesselThrustAllocator::Geometry geometry;
        float max_surge=0, max_sway=0, max_yaw=0;
        float slew1=0, slew2=0, bow_slew=0;
        bool bow=false;
        uint32_t revision1=0, revision2=0, bow_revision=0;
    };
    struct Input {
        int8_t mode=0;
        bool selected=false, pair_ready=false, centered=false, bow_ready=false;
        bool allowed=false, override=false;
        float dt=0;
        VesselThrustAllocator::Demand demand; // normalized controller inputs
        VspMapper::Vector manual[2] {};
        float manual_bow = 0;
    };
    using Result = VesselThrustAllocator::Result;
    bool active() const { return _owner != 0; }
    bool dp_active() const { return _owner == 2; }
    bool guard() const { return (_status & NeutralGuard)!=0; }
    bool bow_guard() const { return _guard_bow || _pending_bow; }
    uint16_t status() const { return _status; }
    bool rearm() const { return _rearm; }
    const Result& command() const { return _command; }
    static bool valid(const Config& c) {
        const auto& g=c.geometry;
        const bool layout=(g.layout==VesselThrustAllocator::Layout::AFT && g.vsp1.x<0 && g.vsp2.x<0) ||
                          (g.layout==VesselThrustAllocator::Layout::VWT && g.vsp1.x>=0 && g.vsp2.x>=0 && !c.bow);
        return layout && g.vsp1.y<0 && g.vsp2.y>0 && positive(c.max_surge) && positive(c.max_sway) && positive(c.max_yaw) &&
            positive(c.slew1) && positive(c.slew2) && (!c.bow || (g.bow_x>0 && positive(c.bow_slew))) &&
            VesselThrustAllocator::allocate(g, {}, c.bow).valid;
    }
    bool update(const Config& c, const Input& in) {
        _status=0; _pending=0; _guard_bow=_pending_bow;
        if(in.selected) _status|=Selected;
        if(!in.selected && !active() && !_rearm) { _command={}; return false; }
        if(in.pair_ready) _status|=PairReady;
        const bool geometry_ok=valid(c);
        if(geometry_ok) _status|=GeometryValid;
        if(c.bow && !in.bow_ready) _status|=BowUnavailable;
        if(in.override) _status|=PilotOverride;
        if(in.mode==0) {
            if(_rearm) _status|=NeutralGuard;
            _owner=0; _rearm=false; _rejected=false; _command={}; _pending_bow=false;
            return false;
        }
        const bool eligible=in.selected && in.pair_ready && geometry_ok && (!c.bow || in.bow_ready) &&
            in.allowed && !in.override && positive(in.dt) && in.dt<=0.1f && (in.mode==1 || in.mode==2);
        if ((active() && (!eligible || !same(c,_config))) || in.override) {
            _rearm=true; _owner=0; _command={};
        }
        if (_rearm) { _status|=RearmRequired|NeutralGuard; if (_rejected) _status|=OutputRejected; return false; }
        if(!eligible || (!active() && in.mode==2 && !in.centered)) { _command={}; return false; }
        const auto target=VesselThrustAllocator::allocate(c.geometry,
            {in.demand.surge*c.max_surge,in.demand.sway*c.max_sway,in.demand.yaw*c.max_yaw},c.bow);
        if(!target.valid) { reject(); return false; }
        if (!active()) {
            _config = c;
            _command = {};
            _command.vsp1 = in.manual[0];
            _command.vsp2 = in.manual[1];
            _command.bow = c.bow ? in.manual_bow : 0;
        }
        _pending_bow=c.bow;
        // Publish the manual frame once when taking ownership. Subsequent
        // frames slew from that frame, with no extra step on the entry cycle.
        const float dt = active() ? in.dt : 0;
        float k=1;
        limit(k,hypotf(target.vsp1.forward-_command.vsp1.forward,target.vsp1.starboard-_command.vsp1.starboard),c.slew1*dt);
        limit(k,hypotf(target.vsp2.forward-_command.vsp2.forward,target.vsp2.starboard-_command.vsp2.starboard),c.slew2*dt);
        if(c.bow) limit(k,fabsf(target.bow-_command.bow),c.bow_slew*dt);
        auto next=target;
        next.vsp1={mix(_command.vsp1.forward,target.vsp1.forward,k),mix(_command.vsp1.starboard,target.vsp1.starboard,k)};
        next.vsp2={mix(_command.vsp2.forward,target.vsp2.forward,k),mix(_command.vsp2.starboard,target.vsp2.starboard,k)};
        next.bow=mix(_command.bow,target.bow,k);
        next.achieved = VesselThrustAllocator::achieved(c.geometry, next);
        _command=next; _pending=in.mode;
        if(c.bow) _status|=BowUsed;
        if(target.saturated) _status|=Saturated;
        if(k<1) _status|=SlewLimited;
        return true;
    }
    void confirm() { if (!_pending) return; _owner=_pending; _pending=0; _status|=HeadingOwned; if(_owner==2) _status|=DPOwned; }
    void reject() { _pending=0; _owner=0; _rearm=true; _rejected=true; _command={}; _status&=~(HeadingOwned|DPOwned|BowUsed); _status|=OutputRejected|RearmRequired|NeutralGuard; }
private:
    static bool positive(float v) { return isfinite(v)&&v>0; }
    static void limit(float& k,float distance,float step) { if(distance>step) k=fminf(k,step/distance); }
    static float mix(float a,float b,float k) { return a+(b-a)*k; }
    // Any change in a valid allocator setting requires returning to Free mode.
    static bool unchanged(float a, float b) { return a <= b && a >= b; }
    static bool same(const Config& a,const Config& b) {
        const auto& x = a.geometry;
        const auto& y = b.geometry;
        return x.layout == y.layout &&
               unchanged(x.vsp1.x, y.vsp1.x) && unchanged(x.vsp1.y, y.vsp1.y) && unchanged(x.vsp1.force, y.vsp1.force) &&
               unchanged(x.vsp2.x, y.vsp2.x) && unchanged(x.vsp2.y, y.vsp2.y) && unchanged(x.vsp2.force, y.vsp2.force) &&
               a.bow == b.bow && (!a.bow || (unchanged(x.bow_x, y.bow_x) && unchanged(x.bow_force, y.bow_force) &&
                   a.bow_revision == b.bow_revision && unchanged(a.bow_slew, b.bow_slew))) &&
               unchanged(a.max_surge, b.max_surge) && unchanged(a.max_sway, b.max_sway) && unchanged(a.max_yaw, b.max_yaw) &&
               unchanged(a.slew1, b.slew1) && unchanged(a.slew2, b.slew2) && a.revision1 == b.revision1 && a.revision2 == b.revision2;
    }
    Config _config{};
    Result _command{};
    uint16_t _status=0;
    int8_t _owner=0,_pending=0;
    bool _rearm=false,_pending_bow=false,_guard_bow=false,_rejected=false;
};
}
