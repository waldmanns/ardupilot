#include "Logic.h"

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
    // @Group: VHDG_
    // @Path: HeadingHoldParams.cpp
    AP_SUBGROUPINFO(vhdg_params, "VHDG_", 9, Logic, HeadingHoldParameters),
    // @Group: VFN
    // @Path: FunctionParameters.cpp
    AP_SUBGROUPINFO(vfn_params, "VFN", 10, Logic, FunctionParameters),
    AP_GROUPEND
};

void Logic::begin_cycle()
{
    _vsp_authority[0] = _vsp_authority[1] = 0;
    // AHRS has already updated. Snapshot once so neither component execution
    // order nor call count can advance the shared roll controller twice.
    // Legacy paired rod corrections cannot share ownership with vector control.
    // Disable their shared state until both drives return to legacy ownership.
    const bool vector_owned = vsp1_params.vmode.get() != 0 || vsp2_params.vmode.get() != 0 ||
        _vectors[0].blocks_assistance() || _vectors[1].blocks_assistance();
    _roll_enabled = !vector_owned && vrs_params.enabled() && vfn.permits(Functions::Type::Roll);
    _heading_enabled = vhdg_params.enabled() && vfn.permits(Functions::Type::Heading);
    vrs.update(vrs_params, vsp1_params.roll, vsp2_params.roll, _roll_enabled);
    for (uint8_t component = 0; component < component_count; component++) {
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
    case ComponentId::Count:
        break;
    }
    return nullptr;
}

void Logic::run(ComponentId id)
{
    if (!valid(id)) {
        return;
    }

    const uint8_t index = uint8_t(id);
    if (id == ComponentId::ThrusterBow || id == ComponentId::ThrusterStern) {
        const uint8_t n = index - uint8_t(ComponentId::ThrusterBow);
        const auto settings = n == 0 ? thruster_bow_params.control_values() : thruster_stern_params.control_values();
        _thrusters[n].update(AP_HAL::millis(), settings,
            vfn.permits(n == 0 ? Functions::Type::Bow : Functions::Type::Stern),
            (_thruster_routes & (1U << n)) != 0, _runtime[index].sources_valid[0], _runtime[index].inputs[0]);
        _runtime[index].outputs[0] = _thrusters[n].output();
        return;
    }
    finish_vsp(index, prepare_vsp(index));
}

bool Logic::prepare_vsp(uint8_t drive)
{
    auto& runtime = _runtime[drive];
    const auto settings = drive == 0 ? vsp1_params.vector_settings() : vsp2_params.vector_settings();
    return _vectors[drive].prepare(AP_HAL::millis(), _thruster_route_revision, settings,
                                  runtime.inputs[0], runtime.inputs[1],
                                  runtime.sources_valid[0] && runtime.sources_valid[1]);
}

void Logic::finish_vsp(uint8_t drive, bool vector_owned, const VspMapper::Vector* heading)
{
    auto& runtime = _runtime[drive];
    if (vector_owned) {
        const bool assisted = _vectors[drive].apply(runtime.outputs[0], runtime.outputs[1], heading);
        runtime.outputs[2] = runtime.inputs[2]; // RPM remains independent.
        _vsp_authority[drive] = _vectors[drive].state() == VspVectorControl::State::Active ? Manual : NeutralGuard;
        if (assisted) {
            _vsp_authority[drive] |= AutoHeading;
        }
        return;
    }
    const auto id = ComponentId(drive);
    const auto& def = component_definitions[drive];
    def.loop(runtime.inputs, runtime.outputs, parameter_store(id), vrs.controller());
    record_vsp_authority(drive);
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

HeadingActuation Logic::heading_auxiliary() const
{
    HeadingActuation result;
    for (const auto& thruster : _thrusters) {
        result.manual |= thruster.manual_override();
        result.revision += thruster.revision();
        if (thruster.heading_available()) {
            result.available = true;
            result.slew = fminf(result.slew, thruster.heading_slew());
        }
    }
    return result;
}

void Logic::run_heading_group(bool controls_valid, uint32_t route_revision)
{
    run(ComponentId::ThrusterBow);
    run(ComponentId::ThrusterStern);
    if (vhdg_params.values().vsp_enabled) {
        run_vsp_pair(controls_valid, route_revision);
    } else {
        vhdg.update(vhdg_params, heading_auxiliary(), controls_valid, true, route_revision,
                    _heading_enabled);
    }
    const auto& heading = vhdg.controller();
    for (uint8_t n = 0; n < 2; n++) {
        _thrusters[n].apply_heading(heading.command(), heading.state() == HeadingHold::State::Hold);
        _runtime[uint8_t(ComponentId::ThrusterBow) + n].outputs[0] = _thrusters[n].output();
    }
}

void Logic::run_vsp_pair(bool controls_valid, uint32_t route_revision)
{
    const bool managed[2] {prepare_vsp(0), prepare_vsp(1)};
    const VspVectorControl::Settings settings[2] {vsp1_params.vector_settings(), vsp2_params.vector_settings()};
    const auto auxiliary = heading_auxiliary();
    HeadingActuation actuation;
    actuation.available = true;
    actuation.manual = auxiliary.manual;
    actuation.slew = auxiliary.available ? auxiliary.slew : 100;
    actuation.revision = auxiliary.revision;
    const auto heading_settings = vhdg_params.values();
    for (uint8_t n = 0; n < 2; n++) {
        // The actuator's preparation stage is the sole mapping/settings validator.
        actuation.configuration_valid &= settings[n].mode == 1 &&
            _vectors[n].state() != VspVectorControl::State::Invalid;
        actuation.available &= _vectors[n].heading_ready();
        actuation.slew = fminf(actuation.slew, settings[n].slew);
        actuation.revision += _vectors[n].revision();
        // Both neutral definitions must agree before automatic authority begins.
        for (uint8_t axis = 0; axis < 2; axis++) {
            const int32_t offset = int32_t(_runtime[n].inputs[axis]) - 1500;
            if (input_valid(ComponentId(n), axis) &&
                (offset < -heading_settings.stick_deadband || offset > heading_settings.stick_deadband ||
                 offset < -settings[n].deadband_us || offset > settings[n].deadband_us)) {
                actuation.manual = true;
            }
        }
    }
    vhdg.update(vhdg_params, actuation, controls_valid, true, route_revision, _heading_enabled);
    const auto& heading = vhdg.controller();
    const bool holding = heading.state() == HeadingHold::State::Hold;
    // One signed yaw effort, positive toward starboard. The mapper owns all
    // rotor/installation/rod geometry; no rod-Y correction path remains.
    const VspMapper::Vector commands[2] {{heading.command(), 0}, {-heading.command(), 0}};
    for (uint8_t n = 0; n < 2; n++) {
        finish_vsp(n, managed[n], holding ? &commands[n] : nullptr);
    }
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
    const auto* roll = vrs.controller();
    if (roll != nullptr && roll->status(drive).command > 1.0e-6f) {
        flags |= RollAssist;
    }
    _vsp_authority[drive] = flags;
}

void Logic::update_heading_bypass(bool controls_valid, bool routes_valid, uint32_t route_revision)
{
    vhdg.update(vhdg_params, HeadingActuation{}, controls_valid, routes_valid, route_revision, _heading_enabled);
}

bool Logic::output_valid(ComponentId id) const
{
    if (!valid(id)) {
        return false;
    }
    for (uint8_t port = 0; port < definition(id).input_count; port++) {
        if (!_runtime[uint8_t(id)].sources_valid[port]) {
            return false;
        }
    }
    return true;
}

bool Logic::input_valid(ComponentId id, uint8_t port) const
{
    return valid(id) && port < definition(id).input_count &&
        _runtime[uint8_t(id)].sources_valid[port];
}

bool Logic::set_input(ComponentId id, uint8_t port, uint16_t value_us, bool source_valid)
{
    if (!valid(id)) {
        return false;
    }

    const uint8_t index = uint8_t(id);
    if (port >= component_definitions[index].input_count) {
        return false;
    }

    _runtime[index].inputs[port] = value_us;
    _runtime[index].sources_valid[port] = source_valid;
    return true;
}

uint16_t Logic::input(ComponentId id, uint8_t port) const
{
    if (!valid(id)) {
        return 0;
    }

    const uint8_t index = uint8_t(id);
    if (port >= component_definitions[index].input_count) {
        return 0;
    }
    return _runtime[index].inputs[port];
}

uint16_t Logic::output(ComponentId id, uint8_t port) const
{
    if (!valid(id)) {
        return 0;
    }

    const uint8_t index = uint8_t(id);
    if (port >= component_definitions[index].output_count) {
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
