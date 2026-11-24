#include "AzipodControl.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>
using namespace Vektor2;
using namespace Vektor2::Components;

static float diff(float a, float b) { return std::fabs(std::remainder(a-b,360.0f)); }
static void solver_tests()
{
    const auto forward = azipod_map(2000,1500,25);
    assert(forward.angle == 0 && forward.magnitude == 1);
    assert(azipod_map(1500,1500,25).magnitude == 0);
    assert(azipod_map(1510,1510,25).magnitude == 0);
    assert(azipod_map(2000,2000,25).magnitude == 1);
    assert(diff(azipod_map(1500,1000,25).angle,270) < 0.001f);
    assert(azipod_map(999,2000,25).magnitude == 0);
    unsigned cases = 0;
    for (int range : {180,270,120,210}) {
        for (int offset : {-177,0,63,180,360}) {
            for (int rev : {0,1}) {
                for (int opt : {0,1}) {
                    for (int project : {0,1}) {
                        AzipodSettings s;
                        s.angle_min = -range/2 + 15;
                        s.angle_max = s.angle_min + range;
                        s.offset = offset; s.reversible = rev;
                        s.reverse_opt = opt; s.unreachable = project;
                        for (int a = -180; a < 180; ++a) {
                            const auto result = azipod_solve(s,{float(a),0.75f},s.angle_min,1);
                            ++cases;
                            assert(result.angle >= s.angle_min && result.angle <= s.angle_max);
                            assert(std::fabs(result.thrust) <= 0.75f);
                            assert(rev || result.thrust >= 0);
                            // Exact reachability computed independently by searching angle equivalents.
                            bool reachable = false;
                            for (int k=-3;k<=3;++k) {
                                const int f = a-offset+360*k;
                                reachable |= f>=s.angle_min && f<=s.angle_max;
                                if (rev) { reachable |= f-180>=s.angle_min && f-180<=s.angle_max; }
                            }
                            assert(result.reachable == reachable);
                            if (rev && range >= 180) { assert(reachable); }
                            if (reachable) {
                                assert(std::fabs(std::fabs(result.thrust)-0.75f) < 1e-6f);
                                assert(diff(result.angle+offset+(result.thrust<0?180:0),a)<0.001f);
                            } else if (!project) { assert(result.thrust == 0); }
                            else if (result.thrust != 0) {
                                float delta=diff(result.angle+offset+(result.thrust<0?180:0),a);
                                assert(delta < 90.001f);
                                assert(std::fabs(std::fabs(result.thrust)-0.75f*std::cos(delta*0.01745329252f))<0.0001f);
                            }
                        }
                    }
                }
            }
        }
    }
    AzipodSettings s; s.angle_min=-135; s.angle_max=135; s.reversible=1;
    auto result=azipod_solve(s,{120,1},0,1);
    assert(result.thrust<0 && result.angle==-60);
    s.reverse_opt=0; result=azipod_solve(s,{120,1},0,1);
    assert(result.thrust>0 && result.angle==120);
    result=azipod_solve(s,{180,1},0,1); assert(result.thrust<0 && result.angle==0);
    s.reverse_opt=1; s.hysteresis=15;
    result=azipod_solve(s,{95,1},0,1); assert(result.thrust>0);
    result=azipod_solve(s,{100,1},0,1); assert(result.thrust<0);
    // No crossing mechanical stops via a circular shortcut.
    result=azipod_solve(s,{130,1},-135,-1); assert(result.thrust<0 && result.angle==-50);
    s.angle_min=20; s.angle_max=-20; assert(!s.valid());
    assert(!azipod_solve(s,{0,1},0,1).reachable);
    s.angle_min=-90; s.angle_max=90; s.motor_slew=std::numeric_limits<float>::quiet_NaN();
    assert(!s.valid());
    std::printf("PASS solver: %u angle/config combinations\n",cases);
}

struct Rig {
    AzipodControl control;
    AzipodSettings s;
    uint16_t input[2]{1500,1500}, output[2]{};
    bool valid[2]{true,true};
    bool routes=true, rc=true;
    uint32_t now=100, revision=1;
    void tick(unsigned count=1) {
        for(unsigned i=0;i<count;++i) {
            now+=10;
            control.run(now,revision,input,valid,routes,rc,output);
            assert(output[0]==0 || (output[0]>=1000 && output[0]<=2000));
            assert(output[1]==0 || (output[1]>=1000 && output[1]<=2000));
        }
    }
    void set(uint16_t x,uint16_t y){input[0]=x;input[1]=y;}
    void home(){set(1500,1500);tick(500);assert(output[1]==control.motor_stop());}
};
static void control_tests()
{
    for (int range:{180,270}) for (int rev:{0,1}) for (int opt:{0,1}) {
        Rig r; r.s.angle_min=-range/2; r.s.angle_max=range/2;
        r.s.reversible=rev;r.s.reverse_opt=opt;r.s.rate=180;r.s.motor_slew=2;
        r.control.configure(r.s);
        const auto stop=r.control.motor_stop();
        r.set(2000,1500);r.tick(500);assert(r.output[1]==stop); // nonneutral boot
        r.home(); r.set(2000,1500);r.tick(60);assert(r.output[1]==2000);
        r.set(1000,1500);r.tick();assert(r.output[1]==stop);
        r.tick(49);assert(r.output[1]==stop);
        r.tick(60);assert(r.output[1]==(rev?1000:stop));
        r.set(1500,1500);r.tick();assert(r.output[1]==stop);
        r.set(1500,2000);r.tick();assert(r.output[1]==stop); // steering movement or reversal delay
        r.tick(160);assert(r.output[0]>=1000 && r.output[0]<=2000);
        assert(r.output[1]!=stop);
        const auto last=r.output[0];r.set(1500,1500);r.tick();assert(r.output[0]==last && r.output[1]==stop);
        r.set(2000,1500);r.tick(200);assert(r.output[1]!=stop);
        r.rc=false;r.tick();assert(r.output[1]==stop);
        r.rc=true;r.tick(300);assert(r.output[1]==stop); // no restart without neutral
        r.home();r.set(2000,1500);r.tick(200);assert(r.output[1]!=stop);
        r.valid[1]=false;r.tick();assert(r.output[1]==stop);
        r.valid[1]=true;r.tick(100);assert(r.output[1]==stop);
        r.home();r.set(2000,1500);r.tick(200);assert(r.output[1]!=stop);
        r.routes=false;r.tick();assert(r.output[0]==0 && r.output[1]==stop);
        r.routes=true;r.tick(500);assert(r.output[1]==stop);
        r.home();r.set(2000,1500);r.tick(200);assert(r.output[1]!=stop);
        ++r.revision;r.tick();assert(r.output[1]==stop);
        r.tick(500);assert(r.output[1]==stop);
        r.home();r.set(2000,1500);r.tick(200);assert(r.output[1]!=stop);
        r.now+=101;r.tick();assert(r.output[1]==stop);
        r.tick(500);assert(r.output[1]==stop);
    }
    Rig r;r.s.angle_max=360;r.s.angle_min=0;r.control.configure(r.s);r.tick();
    assert(r.output[0]==0 && r.output[1]==0);
    std::puts("PASS controller: 8 hardware combinations and safety transitions");
}
static void reason_tests()
{
    using C = AzipodControl;
    Rig r;
    r.s.rate=180; r.s.reversible=1; r.control.configure(r.s);
    r.set(2000,1500); r.tick(); assert(r.control.reasons()==C::StartupCentering);
    r.tick(500); assert(r.control.reasons()==C::NeutralRequired);
    r.home(); assert(r.control.reasons()==0);
    r.set(2000,1500); r.tick(120); assert(r.control.reasons()==0 && r.output[1]==2000);
    r.set(1000,1500); r.tick(); assert(r.control.reasons()==C::ReversalDelay);
    r.tick(10); assert(r.control.reasons()==C::ReversalDelay);
    r.tick(150); assert(r.control.reasons()==0 && r.output[1]==1000);
    r.set(1500,2000); r.tick(); assert(r.control.reasons() & C::AlignmentPending);
    r.rc=false; r.valid[0]=false; r.tick();
    assert(r.control.reasons()==(C::RcInvalid|C::InputInvalid|C::NeutralRequired));
    r.rc=true; r.valid[0]=true; r.tick(); assert(r.control.reasons()==C::NeutralRequired);
    r.home(); assert(r.control.reasons()==0);
    r.set(999,1500); r.tick(); assert(r.control.reasons()==(C::InputInvalid|C::NeutralRequired));
    r.routes=false; r.tick(); assert(r.control.reasons()==(C::OutputRouteMissing|C::InputInvalid));
    r.routes=true; r.home(); assert(r.control.reasons()==0);
    r.control.output_failed(); assert(r.control.reasons()==C::OutputRejected);
    r.tick(); assert(r.control.reasons()==C::StartupCentering);
    r.control.calibrate(true,1800,10,true,r.output); assert(r.control.reasons()==0);
    r.control.calibrate(false,1800,10,false,r.output); assert(r.control.reasons()==C::OutputRouteMissing);
    r.control.output_failed(); assert(r.control.reasons()==(C::OutputRouteMissing|C::OutputRejected));
    Rig bad; bad.s.angle_min=90; bad.control.configure(bad.s); bad.tick();
    assert(bad.control.reasons()==C::CalibrationInvalid);
    bad.control.calibrate(true,1500,0,true,bad.output); assert(bad.control.reasons()==C::CalibrationInvalid);
    // Forward-only: astern unreachable; projected 135-degree vector can use +90.
    Rig f; f.s.rate=180; f.control.configure(f.s); f.home();
    f.set(1000,1500); f.tick(); assert(f.control.reasons()==C::DirectionUnreachable);
    f.set(1500,1500); f.tick(); assert(f.control.reasons()==0);
    f.s.unreachable=1; f.control.configure(f.s); f.home();
    f.set(1000,2000); f.tick();
    assert(f.control.reasons()==(C::DirectionUnreachable|C::DirectionProjected|C::AlignmentPending));
    f.tick(200); assert(f.control.reasons()==(C::DirectionUnreachable|C::DirectionProjected));
    assert(f.output[1]>1000);
    f.set(1500,1500); f.tick(); assert(f.control.reasons()==0);
    std::puts("PASS all 11 diagnostic reason bits and clearing");
}
int main(){solver_tests();control_tests();reason_tests();}
