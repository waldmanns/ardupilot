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
    // @Description: VSP configuration value.
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
    // AHRS has already updated. Snapshot once so neither component execution
    // order nor call count can advance the shared roll controller twice.
    _roll_enabled = vrs_params.enabled() && vfn.permits(Functions::Type::Roll);
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
    const ComponentDefinition& def = component_definitions[index];
    const void* params = parameter_store(id);
    if (params == nullptr) {
        return;
    }

    def.loop(_runtime[index].inputs, _runtime[index].outputs, params, vrs.controller());
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

HeadingAuxiliary Logic::heading_auxiliary() const
{
    HeadingAuxiliary result;
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
        const HeadingDrive unused[2] {};
        vhdg.update(vhdg_params, unused, controls_valid, true, route_revision,
                    _heading_enabled, heading_auxiliary());
    }
    const auto& heading = vhdg.controller();
    for (uint8_t n = 0; n < 2; n++) {
        _thrusters[n].apply_heading(heading.command(), heading.state() == HeadingHold::State::Hold);
        _runtime[uint8_t(ComponentId::ThrusterBow) + n].outputs[0] = _thrusters[n].output();
    }
}

void Logic::run_vsp_pair(bool controls_valid, uint32_t route_revision)
{
    const auto first = vsp1_params.values();
    const auto second = vsp2_params.values();
    // Produce the original geometry ONCE, without VRS. Both routed rod
    // snapshots are present before the shared heading decision is made.
    auto& a = _runtime[uint8_t(ComponentId::VSP1)];
    auto& b = _runtime[uint8_t(ComponentId::VSP2)];
    Components::vsp1_loop(a.inputs, a.outputs, first);
    Components::vsp2_loop(b.inputs, b.outputs, second);
    HeadingDrive drives[2];
    drives[0].limits = setupLimiter(first.lim, first.x_c, first.y_c);
    drives[1].limits = setupLimiter(second.lim, second.x_c, second.y_c);
    for (uint8_t n = 0; n < 2; n++) {
        const auto& runtime = _runtime[n];
        for (uint8_t port = 0; port < 3; port++) {
            drives[n].inputs[port] = runtime.inputs[port];
        }
        drives[n].baseline = {runtime.outputs[0], runtime.outputs[1]};
        drives[n].sources_valid = output_valid(ComponentId(n));
    }
    vhdg.update(vhdg_params, drives, controls_valid, true, route_revision, _heading_enabled, heading_auxiliary());
    const auto& heading = vhdg.controller();
    for (uint8_t n = 0; n < 2; n++) {
        auto& runtime = _runtime[n];
        const int32_t correction = heading.correction_us(n);
        Coordinates result = drives[n].baseline;
        result.y_coordinate += correction;
        if (auto* roll = vrs.controller()) {
            const bool valid_inputs = drives[n].sources_valid &&
                runtime.inputs[0] >= VSP_PWM_MIN && runtime.inputs[0] <= VSP_PWM_MAX &&
                runtime.inputs[1] >= VSP_PWM_MIN && runtime.inputs[1] <= VSP_PWM_MAX &&
                runtime.inputs[2] >= VSP_PWM_MIN && runtime.inputs[2] <= VSP_PWM_MAX;
            // VRS gets the travel left by heading hold. An opposing Y ray
            // yields instead of undoing an applied heading correction.
            result = roll->apply(n, result, drives[n].limits, valid_inputs,
                                 correction > 0 ? 1 : (correction < 0 ? -1 : 0));
        }
        runtime.outputs[0] = uint16_t(result.x_coordinate);
        runtime.outputs[1] = uint16_t(result.y_coordinate);
        // RPM is still the original component passthrough.
    }
}

void Logic::update_heading_bypass(bool controls_valid, bool routes_valid, uint32_t route_revision)
{
    const HeadingDrive unused[2] {};
    vhdg.update(vhdg_params, unused, controls_valid, routes_valid, route_revision, _heading_enabled);
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
