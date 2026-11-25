#include "Logic.h"
#include <AP_HAL/AP_HAL.h>

#include "VSP1.h"
#include "VSP2.h"
#include "ThrusterBow.h"
#include "ThrusterStern.h"

namespace Vektor2 {

static void vsp1_adapter(const uint16_t* inputs,
                         uint16_t* outputs,
                         const void* parameter_store,
                         RollStabilization* stabilization)
{
    const auto& store = *static_cast<const VSP1ParameterStore*>(parameter_store);
    const Components::VSP1Params params = store.values();
    Components::vsp1_loop(inputs, outputs, params, stabilization);
}

static void vsp2_adapter(const uint16_t* inputs,
                         uint16_t* outputs,
                         const void* parameter_store,
                         RollStabilization* stabilization)
{
    const auto& store = *static_cast<const VSP2ParameterStore*>(parameter_store);
    const Components::VSP2Params params = store.values();
    Components::vsp2_loop(inputs, outputs, params, stabilization);
}

static const ComponentDefinition component_definitions[] = {
    {
        ComponentId::VSP1,
        "VSP1",
        "VSP1",
        3,
        3,
        vsp1_adapter,
    },
    {
        ComponentId::VSP2,
        "VSP2",
        "VSP2",
        3,
        3,
        vsp2_adapter,
    },
    {
        ComponentId::ThrusterBow,
        "ThrusterBow",
        "THR_BOW",
        1,
        1,
        nullptr, // stateful independent actuator is run by Logic::run
    },
    {
        ComponentId::ThrusterStern,
        "ThrusterStern",
        "THR_STN",
        1,
        1,
        nullptr, // stateful independent actuator is run by Logic::run
    },
    { ComponentId::AZI1, "AZI1", "AZI1", Components::AZI1::input_count, Components::AZI1::output_count, nullptr },
    { ComponentId::AZI2, "AZI2", "AZI2", Components::AZI2::input_count, Components::AZI2::output_count, nullptr },
};

static_assert(sizeof(component_definitions) / sizeof(component_definitions[0]) ==
                  Logic::component_count,
              "Every ComponentId must have a definition");

const AP_Param::GroupInfo Logic::var_info[] = {
    // These subgroup prefixes intentionally match the component definition
    // prefixes, producing VSP1_LIM / VSP1_THR_ANG and matching VSP2_*.
    AP_SUBGROUPINFO(vsp1_params, "VSP1_", 1, Logic, VSP1ParameterStore),
    AP_SUBGROUPINFO(vsp2_params, "VSP2_", 2, Logic, VSP2ParameterStore),
    AP_SUBGROUPINFO(thruster_bow_params, "THR_BOW_", 5, Logic, ThrusterBowParameterStore),
    AP_SUBGROUPINFO(thruster_stern_params, "THR_STN_", 6, Logic, ThrusterSternParameterStore),

    // @Param: VSP_CONF
    // @DisplayName: VSP configuration
    // @Description: Vessel layout. VSP1 is port and VSP2 starboard, side by side. AFT places the pair aft; VWT places it toward the center/front. Does not change individual rod mapping.
    // @Values: 0:Unspecified,1:AFT,2:VWT
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("VSP_CONF", 3, Logic, vsp_conf, 0),

    // @Param: VSP_TEL_HZ
    // @DisplayName: VSP named-value telemetry rate
    // @Description: Rate for each VSP named-value telemetry key. Zero disables it.
    // @Units: Hz
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("VSP_TEL_HZ", 4, Logic, vsp_tel_hz, 5),
    // @Param: THR_TEL_HZ
    // @DisplayName: Thruster PWM telemetry rate
    // @Description: Rate for each thruster output named float. Zero disables it.
    // @Units: Hz
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("THR_TEL_HZ", 7, Logic, thr_tel_hz, 0),

    // @Group: VRS_
    // @Path: RollStabilizationParams.cpp
    AP_SUBGROUPINFO(vrs_params, "VRS_", 8, Logic, RollStabilizationParameters),
    // @Group: VFN
    // @Path: FunctionParameters.cpp
    AP_SUBGROUPINFO(vfn_params, "VFN", 10, Logic, FunctionParameters),
    // Append only: indices 1..10 retain their original meanings.
    // @Param: MAIN_PROP_TYPE
    // @DisplayName: Main propulsion type
    // @Description: Select exactly one main propulsion family at boot. Tunnel thrusters are independent. Invalid values inhibit both main families.
    // @Values: 0:VSP,1:Azipod
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("MAIN_PROP_TYPE", 11, Logic, main_prop_type, 0),
    // @Param: AZI_CONF
    // @DisplayName: Azipod vessel layout
    // @Description: Select installed drives at boot. AFT installs AZI1 port and AZI2 starboard. Tunnel thrusters are independent. Unspecified or unsupported layouts inhibit all azipods.
    // @Values: 0:Unspecified,1:AFT
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("AZI_CONF", 12, Logic, azi_conf, 0),
    // @Group: AZI1_
    // @Path: ComponentParameters.cpp
    AP_SUBGROUPINFO(azi1_params, "AZI1_", 13, Logic, AzipodParameterStore),
    // @Group: AZI2_
    // @Path: ComponentParameters.cpp
    AP_SUBGROUPINFO(azi2_params, "AZI2_", 14, Logic, AzipodParameterStore),
    // @Param: AZI_TEL_HZ
    // @DisplayName: Azipod named-value telemetry rate
    // @Description: Rate for each installed azipod MAVLink named float. Zero disables it.
    // @Units: Hz
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("AZI_TEL_HZ", 15, Logic, azi_tel_hz, 5),
    // @Param: VRPM_ENABLE
    // @DisplayName: VSP RPM governor enable
    // @Description: Each VSP uses its routed RPM_IN in both modes. In governor mode, 1480 us and below commands motor off at 1000 us. At 1500 us the target is RPM_LOW and at 2000 us it is RPM_HIGH.
    // @Values: 0:Passthrough,1:Governor
    // @User: Standard
    AP_GROUPINFO("VRPM_ENABLE", 16, Logic, vrpm_enable, 0),

    // Index 17 previously held VRPM_CH. The governor now uses the RPM_IN route.

    // @Param: VRPM_RPM_LOW
    // @DisplayName: VSP RPM governor low target
    // @Description: Shaft RPM target at 1500 us. Set a positive value within the operating range of both shafts. Zero leaves the target range unconfigured.
    // @Units: RPM
    // @Range: 0 60000
    // @User: Standard
    AP_GROUPINFO("VRPM_RPM_LOW", 18, Logic, vrpm_low, 0),

    // @Param: VRPM_RPM_HIGH
    // @DisplayName: VSP RPM governor high target
    // @Description: Shaft RPM target at 2000 us. Set a value at least as high as RPM_LOW.
    // @Units: RPM
    // @Range: 0 60000
    // @User: Standard
    AP_GROUPINFO("VRPM_RPM_HIGH", 19, Logic, vrpm_high, 0),

    // @Param: VRPM_DB
    // @DisplayName: VSP RPM governor deadband
    // @Description: RPM tolerance above and below the target. Each governor holds its output pulse within this tolerance.
    // @Units: RPM
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("VRPM_DB", 20, Logic, vrpm_deadband, 2),

    // @Param: VRPM_SLEW_FAST
    // @DisplayName: VSP RPM governor fast slew rate
    // @Description: Throttle pulse change in microseconds per second at RPM errors twice NEAR_RPM or larger. Must be at least SLEW_SLOW.
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("VRPM_SLEW_FAST", 21, Logic, vrpm_fast, 30),

    // @Param: VRPM_SLEW_SLOW
    // @DisplayName: VSP RPM governor slow slew rate
    // @Description: Throttle pulse change in microseconds per second at NEAR_RPM error. The rate decreases linearly to zero at the edge of the deadband.
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("VRPM_SLEW_SLOW", 22, Logic, vrpm_slow, 10),

    // @Param: VRPM_NEAR_RPM
    // @DisplayName: VSP RPM governor slow approach distance
    // @Description: Absolute RPM error at which the slow slew rate applies. Must be greater than DB. The rate increases linearly to SLEW_FAST at twice this error.
    // @Units: RPM
    // @Range: 1 60000
    // @User: Standard
    AP_GROUPINFO("VRPM_NEAR_RPM", 23, Logic, vrpm_near, 50),

    // @Param: VRPM_START_MAX
    // @DisplayName: VSP RPM governor startup pulse
    // @Description: Fixed PWM command sent immediately when the pot passes 1480 us and shaft RPM is unavailable. Held until the first valid reading or START_TMO expires.
    // @Units: us
    // @Range: 1001 1500
    // @User: Standard
    AP_GROUPINFO("VRPM_START_MAX", 24, Logic, vrpm_start_us, 1200),

    // @Param: VRPM_START_TMO
    // @DisplayName: VSP RPM governor startup timeout
    // @Description: Seconds to hold the startup pulse while waiting for shaft RPM. At timeout the drive sends 1000 us until the pot is lowered to 1480 us or below.
    // @Units: s
    // @Range: 0.1 60
    // @User: Standard
    AP_GROUPINFO("VRPM_START_TMO", 25, Logic, vrpm_start_timeout, 10),
    AP_GROUPEND
};

void Logic::initialize_main_propulsion()
{
    if (_main.initialized()) { return; }
    _main.initialize(main_prop_type.get());
    if (auto* azi = _main.azi()) {
        azi->layout = azi_conf.get() == 1 ? AzipodLayout::AFT : AzipodLayout::Unspecified;
        azi->port.configure(azi1_params.values());
        azi->starboard.configure(azi2_params.values());
    }
}

void Logic::begin_cycle(const VspRpmGovernor::Feedback (&shaft_rpm)[2])
{
    if (auto* v = _main.vsp()) {
        v->shaft_rpm[0] = shaft_rpm[0];
        v->shaft_rpm[1] = shaft_rpm[1];
        v->authority[0] = v->authority[1] = 0;
        v->allocated_rods = false;
        // AHRS has already updated. Snapshot once, before either VSP runs.
        const bool vector_owned = v->allocator.active() || v->allocator.rearm() || vsp1_params.vmode.get() != 0 || vsp2_params.vmode.get() != 0 ||
            v->vectors[0].blocks_assistance() || v->vectors[1].blocks_assistance();
        v->roll_enabled = !vector_owned && vrs_params.enabled() && vfn.permits(Functions::Type::Roll);
        v->vrs.update(vrs_params, vsp1_params.roll, vsp2_params.roll, v->roll_enabled);
    }
    for (uint8_t component = 0; component < runtime_slot_count; component++) {
        for (uint8_t port = 0; port < max_inputs; port++) {
            _runtime[component].inputs[port] = 0;
            _runtime[component].sources_valid[port] = false;
        }
        for (uint8_t port = 0; port < max_outputs; port++) {
            _runtime[component].outputs[port] = 0;
        }
    }
}

const void* Logic::parameter_store(ComponentId id) const
{
    switch (id) {
    case ComponentId::VSP1:
        return &vsp1_params;
    case ComponentId::VSP2:
        return &vsp2_params;
    case ComponentId::ThrusterBow:
        return &thruster_bow_params;
    case ComponentId::ThrusterStern:
        return &thruster_stern_params;
    case ComponentId::AZI1:
        return &azi1_params;
    case ComponentId::AZI2:
        return &azi2_params;
    case ComponentId::Count:
        break;
    }
    return nullptr;
}

void Logic::run(ComponentId id)
{
    if (!component_selected(id)) {
        return;
    }

    const uint8_t index = runtime_slot(id);
    if (is_tunnel_thruster(id)) {
        const uint8_t n = index - uint8_t(ComponentId::ThrusterBow);
        const auto settings = n == 0 ? thruster_bow_params.control_values() : thruster_stern_params.control_values();
        _thrusters[n].update(AP_HAL::millis(), settings,
            vfn.permits(n == 0 ? Functions::Type::Bow : Functions::Type::Stern),
            (_thruster_routes & (1U << n)) != 0, _runtime[index].sources_valid[0], _runtime[index].inputs[0]);
        _runtime[index].outputs[0] = _thrusters[n].output();
        return;
    }
    if (is_vsp(id)) {
        finish_vsp(index, prepare_vsp(index));
    } else if (is_azipod(id)) {
        auto& runtime = _runtime[index];
        auto& azi = *_main.azi();
        azi.drive(index).run(AP_HAL::millis(), _thruster_route_revision, runtime.inputs, runtime.sources_valid,
                             (_azi_routes & (1U << index)) != 0,
                             _azi_rc_valid, runtime.outputs);
    }
}

void Logic::calibrate_azipod(ComponentId id, bool selected, uint16_t steering, int8_t motor_percent)
{
    if (!is_azipod(id) || !component_selected(id)) { return; }
    const auto slot = runtime_slot(id);
    _main.azi()->drive(slot).calibrate(selected, steering, motor_percent,
                                     (_azi_routes & (1U << slot)) != 0, _runtime[slot].outputs);
}

bool Logic::prepare_vsp(uint8_t drive)
{
    auto& runtime = _runtime[drive];
    const auto settings = drive == 0 ? vsp1_params.vector_settings() : vsp2_params.vector_settings();
    return _main.vsp()->vectors[drive].prepare(AP_HAL::millis(), _thruster_route_revision, settings,
                                  runtime.inputs[0], runtime.inputs[1],
                                  runtime.sources_valid[0] && runtime.sources_valid[1]);
}

void Logic::finish_vsp(uint8_t drive, bool vector_owned)
{
    auto& runtime = _runtime[drive];
    auto& vsp = *_main.vsp();
    if (vector_owned) {
        vsp.vectors[drive].apply(runtime.outputs[0], runtime.outputs[1]);
        vsp.authority[drive] = vsp.vectors[drive].state() == VspVectorControl::State::Active ? Manual : NeutralGuard;
    } else {
        const auto& def = component_definitions[drive];
        def.loop(runtime.inputs, runtime.outputs, parameter_store(ComponentId(drive)), vsp.vrs.controller());
        record_vsp_authority(drive);
    }

    const VspRpmGovernor::Settings settings = {vrpm_enable.get() == 1, vrpm_low.get(), vrpm_high.get(),
        vrpm_deadband.get(), vrpm_fast.get(), vrpm_slow.get(), vrpm_near.get(),
        vrpm_start_us.get(), vrpm_start_timeout.get()};
    runtime.outputs[2] = vsp.rpm_governors[drive].update(AP_HAL::millis(), settings,
        runtime.inputs[2], runtime.sources_valid[2], vsp.shaft_rpm[drive]);
}

bool Logic::vsp_pair_ready() const
{
    const auto* v=_main.vsp();
    return v && vsp1_params.vmode.get()==1 && vsp2_params.vmode.get()==1 &&
        v->vectors[0].heading_ready() && v->vectors[1].heading_ready();
}
bool Logic::vsp_centered() const
{
    const auto* v=_main.vsp();
    return v && v->vectors[0].centered() && v->vectors[1].centered();
}
VspMapper::Vector Logic::vsp_pilot() const
{
    const auto* v=_main.vsp();
    const auto a=v->vectors[0].requested(), b=v->vectors[1].requested();
    return {(a.forward+b.forward)*0.5f,(a.starboard+b.starboard)*0.5f};
}
bool Logic::apply_vsp_allocation(const VspAllocator::Result& result)
{
    if (!vsp_pair_ready() || !result.valid) return false;
    const auto a=VspMapper::map(vsp1_params.mapper_config(),result.vsp1);
    const auto b=VspMapper::map(vsp2_params.mapper_config(),result.vsp2);
    if (!a.valid || !b.valid) return false;
    if (result.bow_allocated) {
        if (!_thrusters[0].heading_available() || _thrusters[0].reasons()) return false;
        // Bow heading_sign is also its lateral polarity: + means starboard.
        apply_thruster_heading(0,result.bow);
    }
    _runtime[0].outputs[0]=a.x_us; _runtime[0].outputs[1]=a.y_us;
    _runtime[1].outputs[0]=b.x_us; _runtime[1].outputs[1]=b.y_us;
    _main.vsp()->allocated_rods=true;
    return true;
}
void Logic::neutral_vsp_allocation(bool bow)
{
    auto* v=_main.vsp(); if (!v) return;
    const VspMapper::Config c[]={vsp1_params.mapper_config(),vsp2_params.mapper_config()};
    for (uint8_t n=0;n<2;n++) {
        _runtime[n].outputs[0]=c[n].x_center>=1000 && c[n].x_center<=2000 ? c[n].x_center : 1500;
        _runtime[n].outputs[1]=c[n].y_center>=1000 && c[n].y_center<=2000 ? c[n].y_center : 1500;
        v->authority[n]=NeutralGuard;
        v->vectors[n].reset_command();
    }
    v->allocated_rods=true;
    if (bow) {
        _thrusters[0].force_neutral();
        _runtime[uint8_t(ComponentId::ThrusterBow)].outputs[0]=_thrusters[0].output();
    }
}
void Logic::confirm_vsp_allocation(bool dp)
{
    auto* v=_main.vsp(); v->allocator.confirm();
    const auto& command = v->allocator.command();
    v->vectors[0].follow_allocation(command.vsp1);
    v->vectors[1].follow_allocation(command.vsp2);
    if (command.bow_allocated) {
        _thrusters[0].follow_heading_output();
    }
    v->authority[0]=v->authority[1]=dp ? uint8_t(HeadingAssist|DPAssist) : uint8_t(Manual|HeadingAssist);
}

void Logic::set_thruster_routes(uint8_t mask, uint32_t revision)
{
    if (revision != _thruster_route_revision) {
        for (auto& thruster : _thrusters) {
            thruster.reset();
        }
        _thruster_route_revision = revision;
    }
    _thruster_routes = mask;
}

void Logic::apply_thruster_heading(uint8_t n, float command)
{
    _thrusters[n].apply_heading(command, true);
    _runtime[uint8_t(ComponentId::ThrusterBow) + n].outputs[0] = _thrusters[n].output();
}

void Logic::release_thruster_heading(uint8_t n)
{
    _thrusters[n].release_heading();
    _runtime[uint8_t(ComponentId::ThrusterBow) + n].outputs[0] = _thrusters[n].output();
}

void Logic::record_vsp_authority(uint8_t drive)
{
    if (drive >= 2) {
        return;
    }
    const ComponentId id = ComponentId(drive);
    uint8_t flags = input_valid(id, 0) && input_valid(id, 1) ? Manual : 0;
    // VRS is additive and intermittent: report only this drive's applied,
    // quantized correction, not the global enable/request or the other drive.
    const auto* roll = _main.vsp()->vrs.controller();
    if (roll != nullptr && roll->status(drive).command > 1.0e-6f) {
        flags |= RollAssist;
    }
    _main.vsp()->authority[drive] = flags;
}

bool Logic::output_valid(ComponentId id) const
{
    if (!component_selected(id)) {
        return false;
    }
    if (is_azipod(id)) { return output_valid(id, 0) && output_valid(id, 1); }
    for (uint8_t port = 0; port < definition(id).input_count; port++) {
        if (!_runtime[runtime_slot(id)].sources_valid[port]) {
            return false;
        }
    }
    return true;
}

bool Logic::output_valid(ComponentId id, uint8_t port) const
{
    if (!component_selected(id) || port >= definition(id).output_count) {
        return false;
    }
    const auto& runtime = _runtime[runtime_slot(id)];
    if (is_azipod(id)) { return runtime.outputs[port] >= 1000 && runtime.outputs[port] <= 2000; }
    if (id == ComponentId::VSP1 || id == ComponentId::VSP2) {
        // Governor mode supplies a stop pulse when the RPM_IN source is lost.
        return port == 2 ? (vrpm_enable.get() == 1 || runtime.sources_valid[2]) :
               (_main.vsp()->allocated_rods || (runtime.sources_valid[0] && runtime.sources_valid[1]));
    }
    return runtime.sources_valid[0];
}

bool Logic::input_valid(ComponentId id, uint8_t port) const
{
    return component_selected(id) && port < definition(id).input_count &&
        _runtime[runtime_slot(id)].sources_valid[port];
}

bool Logic::set_input(ComponentId id, uint8_t port, uint16_t value_us, bool source_valid)
{
    if (!component_selected(id)) {
        return false;
    }

    const uint8_t index = runtime_slot(id);
    if (port >= definition(id).input_count) {
        return false;
    }

    _runtime[index].inputs[port] = value_us;
    _runtime[index].sources_valid[port] = source_valid;
    return true;
}

uint16_t Logic::input(ComponentId id, uint8_t port) const
{
    if (!component_selected(id)) {
        return 0;
    }

    const uint8_t index = runtime_slot(id);
    if (port >= definition(id).input_count) {
        return 0;
    }
    return _runtime[index].inputs[port];
}

uint16_t Logic::output(ComponentId id, uint8_t port) const
{
    if (!component_selected(id)) {
        return 0;
    }

    const uint8_t index = runtime_slot(id);
    if (port >= definition(id).output_count) {
        return 0;
    }
    return _runtime[index].outputs[port];
}

const ComponentDefinition& Logic::definition(ComponentId id)
{
    // Callers validate IDs before reaching here. Falling back to VSP1 keeps
    // this accessor reference-based and allocation-free.
    if (!valid(id)) {
        return component_definitions[0];
    }
    return component_definitions[uint8_t(id)];
}

} // namespace Vektor2
