#include "VTelemetryPropulsion.h"
#include "Logic.h"
#include "Routing.h"
#include "RouteParameters.h"
#include "Telemetry.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>

using namespace Vektor2;
static uint32_t now_ms = 100;
static uint16_t rc_values[6] = {2000, 1500, 1700, 1500, 1500, 1500};
static unsigned roll_updates = 0;
static int rejected_pwm = -1;
static int unavailable_pwm = -1;
namespace AP_HAL {
uint32_t millis() { return now_ms; }
uint64_t micros64() { return uint64_t(now_ms) * 1000; }
}
namespace Vektor2 {
// Only HAL adapters are replaced. Route decoding, dispatch, vector/legacy
// controllers, function evaluation and thruster control are production code.
void RcInput::update() {
    _channel_count = 6;
    for (uint8_t i = 0; i < _channel_count; ++i) { _channels[i] = rc_values[i]; }
    _last_update_us = AP_HAL::micros64();
}
const AP_Param::GroupInfo PwmChannelParameters::var_info[] = { AP_GROUPEND };
const AP_Param::GroupInfo PwmOut::var_info[] = { AP_GROUPEND };
bool PwmOut::write_us(uint8_t channel, uint16_t value) {
    assert(channel < max_pwm_channels && value >= 1000 && value <= 2000);
    if (channel == rejected_pwm) { return false; }
    _enabled_mask |= 1UL << channel;
    _values_us[channel] = value;
    return true;
}
void PwmOut::disable(uint8_t channel) {
    _enabled_mask &= ~(1UL << channel);
    _values_us[channel] = 0;
}
bool PwmOut::heading_output_ready(uint8_t channel) const { return channel != unavailable_pwm; }
void PwmOut::begin_batch() {}
void PwmOut::end_batch() {}
void NamedFloatTelemetry::send_float(const char*, float) const {}
void RollStabilizationBackend::update(const RollStabilizationParameters&,
                                      const RollDriveParameters&, const RollDriveParameters&, bool) {
    ++roll_updates;
}
}

struct CountVsp {
    static int instances;
    CountVsp() { ++instances; }
    ~CountVsp() { --instances; }
    uint64_t payload[20] {};
};
int CountVsp::instances = 0;
struct CountAzi {
    static int instances;
    CountAzi() { ++instances; }
    ~CountAzi() { --instances; }
};
int CountAzi::instances = 0;

static void storage_test()
{
    {
        MainPropulsionStorage<CountVsp, CountAzi> storage;
        assert(!storage.initialized() && !storage.vsp() && !storage.azi());
        storage.initialize(0);
        assert(storage.vsp() && !storage.azi() && CountVsp::instances == 1 && CountAzi::instances == 0);
        storage.initialize(1);
        assert(storage.vsp() && !storage.azi());
    }
    assert(CountVsp::instances == 0 && CountAzi::instances == 0);
    {
        MainPropulsionStorage<CountVsp, CountAzi> storage;
        storage.initialize(1);
        assert(!storage.vsp() && storage.azi() && CountVsp::instances == 0 && CountAzi::instances == 1);
    }
    assert(CountAzi::instances == 0);
    static_assert(runtime_slot_count == 4, "Four slots only");
    static_assert(uint8_t(ComponentId::VSP1) == 0 && uint8_t(ComponentId::VSP2) == 1 &&
                  uint8_t(ComponentId::ThrusterBow) == 2 && uint8_t(ComponentId::ThrusterStern) == 3 &&
                  uint8_t(ComponentId::AZI1) == 4 && uint8_t(ComponentId::AZI2) == 5, "Stable IDs");
    static_assert(sizeof(MainPropulsionStorage<VspBackend,AzipodBackend>) <
                  (sizeof(VspBackend) > sizeof(AzipodBackend) ? sizeof(VspBackend) : sizeof(AzipodBackend)) + 16, "Storage should reserve only larger backend plus tag/alignment");
    static_assert(azipod_count(AzipodLayout::AFT) == 2, "AFT installs two drives");
    static_assert(azipod_count(AzipodLayout::Unspecified) == 0, "No unspecified drives");
    unsigned calibration_count = 0;
    for (auto* p = AzipodParameterStore::var_info; p->name; ++p) {
        assert(std::strcmp(p->name, "ENABLE") != 0);
        ++calibration_count;
    }
    assert(calibration_count == 12);
    for (auto type : {MainPropulsionType::VSP, MainPropulsionType::Azipod, MainPropulsionType::None}) {
      for (auto layout : {AzipodLayout::Unspecified, AzipodLayout::AFT, AzipodLayout(255)}) {
        bool occupied[runtime_slot_count] {};
        unsigned count = 0;
        for (unsigned i = 0; i < 256; ++i) {
            const auto id = ComponentId(i);
            const auto slot = component_runtime_slot(type, layout, id);
            if (component_selected(type, layout, id)) {
                assert(slot < runtime_slot_count && !occupied[slot]);
                occupied[slot] = true;
                ++count;
            } else { assert(slot == runtime_slot_count); }
        }
        assert(count == (type == MainPropulsionType::VSP ? 4U :
            type == MainPropulsionType::Azipod ? 2U + azipod_count(layout) : 2U));
      }
    }
}

static void route(RoutingParameters& params, unsigned index, int16_t src, int16_t dst)
{
    params.slots[index].source.set(src);
    params.slots[index].destination.set(dst);
}

static void boot_test(int8_t type, int8_t layout = 1)
{
    Logic logic;
    AP_Param::setup_object_defaults(&logic, Logic::var_info); // emulate sketch defaults
    assert(logic.main_prop_type.get() == 0 && logic.azi_conf.get() == 0);
    logic.main_prop_type.set(type);
    logic.azi_conf.set(layout);
    logic.thruster_bow_params.enable.set(1);
    logic.thruster_stern_params.enable.set(1);
    logic.initialize_main_propulsion();
    const bool vsp = type == 0;
    const bool azi = type == 1 && layout == 1;
    assert(logic.vsp_selected() == vsp);
    assert(logic.component_selected(ComponentId::AZI1) == azi);
    assert(logic.component_selected(ComponentId::AZI2) == azi);
    assert(logic.azipod_layout() == (azi ? AzipodLayout::AFT : AzipodLayout::Unspecified));
    // Editing requested parameters never swaps the live backend or geometry.
    logic.main_prop_type.set(vsp ? 1 : 0);
    logic.azi_conf.set(layout == 1 ? 0 : 1);
    logic.initialize_main_propulsion();
    assert(logic.vsp_selected() == vsp && logic.component_selected(ComponentId::AZI1) == azi);
    assert(logic.azipod_layout() == (azi ? AzipodLayout::AFT : AzipodLayout::Unspecified));

    VTelemetryCore::Sample sample;
    auto read = [&](uint16_t id) { return VTelemetryCore::read_propulsion_source(id, sample, logic); };
    assert(read(270) && sample.number == (vsp ? 0 : type == 1 ? 1 : 255));
    assert(read(271) == vsp);
    assert(read(272) == (type == 1));
    if (type == 1) { assert(sample.number == (azi ? 1 : 0)); }
    assert(read(285) == azi && read(295) == azi);
    assert(read(286) == azi && read(296) == azi);
    assert(logic.azi_tel_hz.get() == 5);
    for (uint8_t n=0; n<2; ++n) {
        for (uint8_t field=0; field<7; ++field) {
            assert(strlen(VTelemetryCore::azipod_named_key(n,field))<=10);
        }
    }
    assert(strcmp(VTelemetryCore::azipod_named_key(0,6), "AZI1_RB")==0);
    assert(strcmp(VTelemetryCore::azipod_named_key(1,6), "AZI2_RB")==0);
    assert(!read(280) && !read(290));
    Routing routing;
    RoutingParameters params;
    PwmOut pwm;
    RcInput rc;
    route(params, 0, 1, 101); route(params, 1, 2, 102); route(params, 2, 3, 103);
    route(params, 3, 101, 9); route(params, 4, 102, 10); route(params, 5, 103, 11);
    route(params, 6, 1, 111); route(params, 7, 2, 112); route(params, 8, 111, 12);
    route(params, 9, 1, 141); route(params, 10, 2, 142);
    route(params, 11, 141, 13); route(params, 12, 142, 14);
    route(params, 13, 3, 151); route(params, 14, 4, 152);
    route(params, 15, 151, 15); route(params, 16, 152, 16);
    route(params, 17, 5, 121); route(params, 18, 121, 7);
    route(params, 19, 6, 131); route(params, 20, 131, 8);
    // Actuator outputs are only routed to PWM during AZ2.
    assert(params.sync(routing, pwm, true));
    const unsigned previous_roll_updates = roll_updates;
    rc_values[4] = rc_values[5] = 1500;
    auto cycle = [&]() {
        now_ms += 10; rc.update(); routing.prepare(rc, logic, pwm, {});
        assert(routing.publish_outputs(rc, logic, pwm, 0) == 0);
    };
    cycle();
    for (uint16_t base : {280, 290}) {
        for (uint16_t field = 0; field < 6; ++field) { assert(read(base + field) == azi); }
        if (azi) {
            assert(read(base) && sample.number == 1500);
            assert(read(base+1) && sample.number == 1000);
            assert(read(base+2) && sample.number == 0);
            assert(read(base+3) && sample.number == 0);
            assert(read(base+4) && sample.number == 0);
            assert(read(base+5) && sample.number == 1);
        }
    }
    assert(logic.input_valid(ComponentId::VSP1, 0) == vsp);
    assert(logic.input_valid(ComponentId::AZI1, 0) == azi);
    assert(logic.input_valid(ComponentId::AZI2, 0) == azi);
    assert(logic.input(ComponentId::AZI1, 0) == (azi ? 2000 : 0));
    assert(logic.input(ComponentId::AZI2, 0) == (azi ? 1700 : 0));
    assert(!logic.input_valid(ComponentId::VSP2, 2));
    assert(logic.output_valid(ComponentId::VSP1, 0) == vsp);
    assert(pwm.value_us(8) == (vsp ? 1525 : 0));
    assert(pwm.value_us(9) == (vsp ? 1500 : 0));
    assert(pwm.value_us(10) == (vsp ? 1700 : 0));
    assert(pwm.value_us(11) == (vsp ? 1525 : 0));
    for (auto id : {ComponentId::AZI1, ComponentId::AZI2}) {
        assert(logic.output_valid(id) == azi);
        assert(logic.output(id, 0) == (azi ? 1500 : 0));
        assert(logic.output(id, 1) == (azi ? 1000 : 0));
        assert(!logic.set_input(id, 2, 1800)); // no third port
    }
    for (uint8_t ch = 12; ch < 16; ++ch) { assert(pwm.enabled(ch) == azi); }
    if (azi) {
        assert(pwm.value_us(12) == 1500 && pwm.value_us(13) == 1000);
        assert(pwm.value_us(14) == 1500 && pwm.value_us(15) == 1000);
    }
    assert(!logic.output_valid(ComponentId::Count) && !logic.set_input(ComponentId(255), 0, 2000));
    assert(!logic.roll_enabled() && !logic.roll_controller());
    assert(roll_updates == previous_roll_updates + (vsp ? 1U : 0U));
    assert(pwm.value_us(6) == 1500 && pwm.value_us(7) == 1500);
    rc_values[4] = 1900; rc_values[5] = 1100;
    for (int i = 0; i < 15; ++i) { cycle(); }
    assert(pwm.value_us(6) > 1500 && pwm.value_us(7) < 1500);
    assert(!(logic.thruster(0).reasons() & ThrusterControl::OutputRouteMissing));
    assert(!(logic.thruster(1).reasons() & ThrusterControl::OutputRouteMissing));

    if (azi) {
        // Hold neutral through startup, then demonstrate independent control.
        rc_values[0] = rc_values[2] = 1500;
        for (int i = 0; i < 240; ++i) { cycle(); }
        rc_values[0] = 2000;
        for (int i = 0; i < 50; ++i) { cycle(); }
        assert(pwm.value_us(13) > 1000 && pwm.value_us(15) == 1000);
        assert(read(284) && sample.number > 0);
        assert(read(294) && sample.number == 0);
        assert(read(285) && sample.number == 3);
        assert(read(286) && sample.number == 0);
        // Layout alone also makes the second drive independently operational.
        rc_values[0] = 1500; rc_values[2] = 2000;
        for (int i = 0; i < 50; ++i) { cycle(); }
        assert(pwm.value_us(13) == 1000 && pwm.value_us(15) > 1000);
        rc_values[0] = 2000; rc_values[2] = 1500;
        for (int i = 0; i < 50; ++i) { cycle(); }
        assert(pwm.value_us(13) > 1000 && pwm.value_us(15) == 1000);
        // Steering rejection at publication must inhibit motor immediately.
        rejected_pwm = 12; cycle();
        assert(pwm.value_us(12) == 0 && pwm.value_us(13) == 1000);
        assert(read(285) && sample.number == 0);
        assert(read(286) && uint16_t(sample.number) & Components::AzipodControl::OutputRejected);
        assert(read(296) && sample.number == 0);
        for (uint16_t id = 280; id <= 284; ++id) { assert(!read(id)); }
        assert(read(295) && sample.number == 3);
        rejected_pwm = -1; cycle();
        assert(pwm.value_us(13) == 1000);
        rc_values[0] = 1500;
        for (int i = 0; i < 240; ++i) { cycle(); }
        rc_values[0] = 2000;
        for (int i = 0; i < 50; ++i) { cycle(); }
        assert(pwm.value_us(13) > 1000);
        unavailable_pwm = 12; cycle();
        assert(pwm.value_us(12) == 0 && pwm.value_us(13) == 1000);
        assert(read(286) && uint16_t(sample.number) & Components::AzipodControl::OutputRouteMissing);
        unavailable_pwm = -1; cycle();
        assert(pwm.value_us(13) == 1000);
        rc_values[0] = 1500;
        for (int i = 0; i < 240; ++i) { cycle(); }
        rc_values[0] = 2000;
        for (int i = 0; i < 50; ++i) { cycle(); }
        assert(pwm.value_us(13) > 1000);
        // Global RC timeout inhibits thrust even with HoldLast policy.
        now_ms += 600;
        routing.prepare(rc, logic, pwm, {}); routing.publish_outputs(rc, logic, pwm, 0);
        assert(pwm.value_us(13) == 1000);
        assert(read(285) && sample.number == 1); // loop gap forces rehoming
        assert(read(286) && (uint16_t(sample.number) & Components::AzipodControl::RcInvalid));
        assert(read(284) && sample.number == 0);
        rc_values[2] = 1700;
    }
    // Dormant output keeps its destination claimed, no silent fallback.
    Route duplicate;
    duplicate.enabled = true;
    duplicate.source = SourceEndpoint::rc(0);
    duplicate.consumer = ConsumerEndpoint::pwm(12);
    assert(!routing.set_route(22, duplicate));
    // Reject transformed and non-PWM actuator routes.
    Route bad;
    bad.enabled = true; bad.source = SourceEndpoint::component(ComponentId::AZI1, 1);
    bad.consumer = ConsumerEndpoint::component(ComponentId::VSP2, 2);
    assert(!routing.set_route(22, bad));
    bad.consumer = ConsumerEndpoint::pwm(16); bad.reversed = true;
    assert(!routing.set_route(22, bad));
    // Reject nonexistent AZI port in parameter decoding.
    route(params, 22, 143, 17);
    assert(!params.sync(routing, pwm));
    cycle();
    assert(!pwm.enabled(16));
    // After rewiring, previously driven PWM is explicitly released.
    route(params, 3, 141, 9);
    route(params, 22, 0, 0);
    assert(params.sync(routing, pwm));
    cycle();
    assert(!pwm.enabled(8));
}

static void calibration_test()
{
    Logic logic;
    AP_Param::setup_object_defaults(&logic, Logic::var_info);
    logic.main_prop_type.set(1); logic.azi_conf.set(1);
    logic.azi2_params.reversible.set(1);
    logic.initialize_main_propulsion();
    Routing routing; RoutingParameters params; PwmOut pwm; RcInput rc;
    route(params, 0, 141, 1); route(params, 1, 142, 2);
    route(params, 2, 151, 3); route(params, 3, 152, 4);
    assert(params.sync(routing, pwm, true));
    using Result = Routing::CalibrationResult;
    auto cmd = [&](float op, float pod, float steering, float motor, float token = 42, uint8_t sys = 255) {
        const float f[] = {op, pod, steering, motor, token, 0, 0};
        return routing.azipod_calibration(now_ms, sys, 1, 0, f, logic, pwm);
    };
    auto cycle = [&]() {
        now_ms += 10; // deliberately no valid RC or input routes
        routing.prepare(rc, logic, pwm, {}); routing.publish_outputs(rc, logic, pwm, 0);
    };
    assert(cmd(2, 1, 1500, 10) == Result::NoSession);
    assert(cmd(1, 1, 1500, 10) == Result::Invalid);
    assert(cmd(1, 1, 1500, 0) == Result::Accepted); cycle();
    assert(pwm.value_us(0) == 1500 && pwm.value_us(1) == 1000 && pwm.value_us(3) == 1500);
    assert(cmd(2, 1, 2000, -10) == Result::Invalid);
    assert(cmd(2, 1, 2000, 21) == Result::Invalid);
    assert(cmd(2, 1, 2001, 10) == Result::Invalid);
    assert(cmd(2, 1, 1700.5f, 10) == Result::Invalid);
    assert(cmd(2, 1, 1700, 10, 43) == Result::Busy);
    assert(cmd(2, 1, 1700, 10, 42, 254) == Result::Busy);
    assert(cmd(1, 2, 1500, 0) == Result::Busy);
    assert(cmd(2, 1, 2000, 20) == Result::Accepted); cycle();
    assert(pwm.value_us(0) == 2000 && pwm.value_us(1) == 1200 && pwm.value_us(3) == 1500);
    VTelemetryCore::Sample sample;
    assert(VTelemetryCore::read_propulsion_source(285, sample, logic) && sample.number == 4);
    assert(VTelemetryCore::read_propulsion_source(286, sample, logic) && sample.number == 0);
    assert(VTelemetryCore::read_propulsion_source(296, sample, logic) && sample.number == 0);
    assert(cmd(3, 1, 1500, 0) == Result::Accepted); cycle();
    assert(pwm.value_us(1) == 1000 && pwm.value_us(3) == 1500);
    assert(VTelemetryCore::read_propulsion_source(285, sample, logic) && sample.number == 1);
    assert(cmd(1, 2, 1500, 0, 43) == Result::Accepted); cycle();
    assert(cmd(2, 2, 1000, -20, 43) == Result::Accepted); cycle();
    assert(pwm.value_us(2) == 1000 && pwm.value_us(3) == 1400 && pwm.value_us(1) == 1000);
    assert(cmd(2, 2, 1000, 20, 43) == Result::Invalid);
    assert(cmd(2, 2, 1500, 0, 43) == Result::Accepted); cycle();
    assert(cmd(2, 2, 2000, 20, 43) == Result::Accepted); cycle();
    assert(pwm.value_us(2) == 2000 && pwm.value_us(3) == 1600);
    now_ms += Routing::calibration_timeout_ms; cycle();
    assert(pwm.value_us(1) == 1000 && pwm.value_us(3) == 1500);
    assert(cmd(2, 2, 1500, 20, 43) == Result::NoSession);
    assert(cmd(1, 2, 1500, 0, 44) == Result::Accepted); cycle();
    assert(cmd(2, 2, 2000, 20, 44) == Result::Accepted); cycle();
    rejected_pwm = 2; cycle(); rejected_pwm = -1;
    assert(VTelemetryCore::read_propulsion_source(296, sample, logic) &&
           (uint16_t(sample.number) & Components::AzipodControl::OutputRejected));
    assert(pwm.value_us(3) == 1500);
    assert(cmd(2, 2, 1500, 20, 44) == Result::NoSession);
    assert(cmd(1, 2, 1500, 0, 45) == Result::Accepted); cycle();
    assert(cmd(2, 2, 2000, 20, 45) == Result::Accepted); cycle();
    routing.clear_route(2, &pwm); cycle();
    assert(pwm.value_us(3) == 1500);
    assert(cmd(1, 2, 1500, 0, 46) == Result::Unavailable);
    Logic vsp; AP_Param::setup_object_defaults(&vsp, Logic::var_info); vsp.initialize_main_propulsion();
    const float begin[] = {1, 1, 1500, 0, 42, 0, 0};
    assert(routing.azipod_calibration(now_ms, 255, 1, 0, begin, vsp, pwm) == Result::Unavailable);
}

static void vsp_allocation_test(bool bow)
{
    Logic logic; AP_Param::setup_object_defaults(&logic, Logic::var_info);
    logic.initialize_main_propulsion(); logic.vsp1_params.vmode.set(1); logic.vsp2_params.vmode.set(1);
    logic.vsp1_params.lim.set(100); logic.vsp2_params.lim.set(100);
    logic.thruster_bow_params.enable.set(1); logic.thruster_bow_params.heading_enable.set(1);
    logic.thruster_bow_params.dst.set(300);
    Routing routing; RoutingParameters params; PwmOut pwm; RcInput rc;
    route(params,0,1,101); route(params,1,2,102); route(params,2,1,111); route(params,3,2,112);
    route(params,4,101,9); route(params,5,102,10); route(params,6,111,11); route(params,7,112,12);
    route(params,8,3,103); route(params,9,103,13);
    if (bow) { route(params,10,5,121); route(params,11,121,7); }
    assert(params.sync(routing,pwm,true));
    rc_values[0]=rc_values[1]=rc_values[4]=1500; rc_values[2]=1700;
    auto prepare=[&](){now_ms+=10;rc.update();routing.prepare(rc,logic,pwm,{});};
    prepare(); assert(!logic.vsp_pair_ready()); prepare(); assert(logic.vsp_pair_ready());
    assert(routing.vsp_routes_ready(pwm,bow));
    assert(routing.vsp_routes_ready(pwm,true)==bow);
    auto* allocator=logic.vsp_allocator();
    VspAllocator::Config c; c.geometry.layout=bow?VesselThrustAllocator::Layout::AFT:VesselThrustAllocator::Layout::VWT;
    c.geometry.vsp1={bow?-2.f:1.5f,-1,1}; c.geometry.vsp2={bow?-2.f:1.5f,1,1};
    c.max_surge=c.max_sway=c.max_yaw=1; c.slew1=c.slew2=2;
    c.bow=bow; c.geometry.bow_x=3; c.geometry.bow_force=.7f; c.bow_slew=2;
    c.revision1=logic.vsp_revision(0);c.revision2=logic.vsp_revision(1);c.bow_revision=logic.thruster(0).revision();
    VspAllocator::Input in; in.mode=2;in.selected=in.pair_ready=in.allowed=in.centered=true;in.bow_ready=bow;in.dt=.01f;in.demand={0,.8f,0};
    for(int n=0;n<100;n++) {
        prepare(); assert(allocator->update(c,in)); assert(logic.apply_vsp_allocation(allocator->command()));
        assert(routing.publish_outputs(rc,logic,pwm,0,true,bow)==4);logic.confirm_vsp_allocation(true);
        assert(logic.vsp_authority(0)==18 && logic.vsp_authority(1)==18);
        assert(pwm.value_us(12)==1700);
    }
    assert(pwm.value_us(9)>1500 && pwm.value_us(11)>1500);
    assert(bow ? pwm.value_us(6)>1500 : pwm.value_us(6)==0);
    if(!bow) assert(pwm.value_us(8)<1500 && pwm.value_us(10)>1500);
    prepare(); assert(allocator->update(c,in)); assert(logic.apply_vsp_allocation(allocator->command()));
    rejected_pwm=10; assert(routing.publish_outputs(rc,logic,pwm,0,true,bow)==0); rejected_pwm=-1;
    assert(!allocator->active() && allocator->rearm() && (allocator->status()&VspAllocator::OutputRejected));
    assert(logic.vsp_authority(0)==8 && logic.vsp_authority(1)==8);
    assert(pwm.value_us(8)==1500 && pwm.value_us(9)==1500 && pwm.value_us(10)==0 && pwm.value_us(11)==1500);
    if(bow) assert(pwm.value_us(6)==1500);
    assert(pwm.value_us(12)==1700);
    prepare(); unavailable_pwm=9; assert(!routing.vsp_routes_ready(pwm,bow)); unavailable_pwm=-1;
    rc_values[0]=2000;
}

static void vsp_rpm_governor_test()
{
    Logic logic;
    AP_Param::setup_object_defaults(&logic, Logic::var_info);
    logic.initialize_main_propulsion();
    logic.vsp2_params.vmode.set(1);
    assert(logic.vrpm_enable.get() == 0 && logic.vrpm_deadband.get() == 2);
    logic.vrpm_low.set(100);
    logic.vrpm_high.set(300);

    Routing routing;
    RoutingParameters params;
    PwmOut pwm;
    RcInput rc;
    route(params, 0, 3, 103);
    route(params, 1, 4, 113);
    route(params, 2, 103, 11);
    route(params, 3, 113, 12);
    assert(params.sync(routing, pwm, true));

    auto check = [&](uint16_t pot1, uint16_t pot2, float rpm1, float rpm2, uint16_t out1, uint16_t out2) {
        now_ms += 100;
        rc_values[2] = pot1;
        rc_values[3] = pot2;
        rc.update();
        routing.prepare(rc, logic, pwm, {{rpm1, true}, {rpm2, true}});
        routing.publish_outputs(rc, logic, pwm, 0);
        assert(pwm.value_us(10) == out1);
        assert(pwm.value_us(11) == out2);
    };
    using Governor = VspRpmGovernor;
    const uint16_t valid_inputs = Governor::ControlValid | Governor::ConfigValid |
                                  Governor::ThrottleValid | Governor::FeedbackValid;
    const uint16_t engaged = valid_inputs | Governor::Enabled | Governor::Requested | Governor::Active;

    check(1480, 1800, 60, 400, 1480, 1800);
    assert(logic.vsp_rpm_status(0) == valid_inputs);
    assert(logic.vsp_rpm_status(1) == (valid_inputs | Governor::Requested));
    logic.vrpm_enable.set(1);
    check(1480, 1000, 60, 400, 1000, 1000);
    assert(logic.vsp_rpm_status(0) == (valid_inputs | Governor::Enabled));
    check(1481, 2000, 50, 200, 1000, 1000);
    check(1481, 2000, 50, 200, 1001, 1003); // low and high targets, slow and fast rates
    assert(logic.vsp_rpm_status(0) == engaged && logic.vsp_rpm_status(1) == engaged);
    check(1500, 1750, 98, 202, 1001, 1003); // inclusive deadband, midpoint of curve
    assert(logic.vsp_rpm_status(0) == (engaged | Governor::InDeadband));
    check(1750, 1750, 125, 150, 1003, 1004); // blend at 75 RPM and slow rate at 50 RPM
    logic.vrpm_fast.set(40);
    logic.vrpm_slow.set(20);
    check(1750, 1750, 150, 100, 1005, 1008); // slew parameters take effect immediately
    check(1750, 1750, 222, 200, 1004, 1008); // taper and reduce throttle above target
    check(1750, 1750, 0, 200, 1000, 1008); // feedback loss stops only VSP1
    assert(logic.vsp_rpm_status(0) == ((engaged & ~(Governor::FeedbackValid | Governor::Active)) | Governor::Inhibited));
    assert(logic.vsp_rpm_status(1) == (engaged | Governor::InDeadband));
    check(1750, 1750, 100, 200, 1000, 1008); // feedback recovery needs a low pot
    check(1480, 1480, 0, 0, 1000, 1000);
    check(1481, 1480, 0, 0, 1200, 1000); // immediate startup pulse
    check(2000, 1480, 0, 0, 1200, 1000); // pot changes the target, not the startup pulse
    assert(logic.vsp_rpm_status(0) == ((engaged & ~(Governor::FeedbackValid | Governor::Active)) | Governor::Starting));
    check(1500, 1480, 50, 0, 1200, 1000); // closed-loop handoff holds the startup pulse
    check(1500, 1480, 50, 0, 1202, 1000); // feedback now controls the approach

    logic.vrpm_start_us.set(1250);
    logic.vrpm_start_timeout.set(0.3f);
    check(1480, 1480, 0, 0, 1000, 1000);
    check(1500, 1480, 0, 0, 1250, 1000);
    check(1500, 1480, 0, 0, 1250, 1000);
    check(1500, 1480, 0, 0, 1250, 1000);
    check(1500, 1480, 0, 0, 1000, 1000); // timeout measured from engagement
    assert((logic.vsp_rpm_status(0) & (Governor::Inhibited | Governor::Starting)) == Governor::Inhibited);
    check(1500, 1480, 100, 0, 1000, 1000); // late feedback cannot restart the drive
    check(1480, 1480, 0, 0, 1000, 1000);
    check(1500, 1480, 0, 0, 1250, 1000);

    logic.vrpm_high.set(0);
    check(2000, 2000, 100, 300, 1000, 1000);
    assert(logic.vsp_rpm_status(1) == (engaged & ~(Governor::ConfigValid | Governor::Active)));
    logic.vrpm_high.set(300);
    routing.clear_route(0, &pwm);
    check(2000, 1480, 100, 300, 1000, 1000); // missing pot still emits a stop pulse
    logic.vrpm_enable.set(0);
    check(2000, 1480, 100, 300, 0, 1480); // passthrough keeps original routing semantics

    rc_values[2] = 1700;
    rc_values[3] = rc_values[5] = 1500;
}

int main()
{
    storage_test();
    vsp_rpm_governor_test();
    vsp_allocation_test(false); vsp_allocation_test(true);
    calibration_test();
    boot_test(0); boot_test(1); boot_test(-1); boot_test(2);
    boot_test(1, 0); boot_test(1, 2); boot_test(1, -1);
    std::printf("PASS main propulsion (profile %d): VspBackend=%zu AziBackend=%zu shared=%zu Logic=%zu\n",
                VEKTOR2_SEQ_LARGE, sizeof(VspBackend), sizeof(AzipodBackend),
                sizeof(MainPropulsionStorage<VspBackend,AzipodBackend>), sizeof(Logic));
}
