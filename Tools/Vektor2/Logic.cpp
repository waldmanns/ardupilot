#include "Logic.h"

#include "VSP1.h"
#include "VSP2.h"
#include "ThrusterBow.h"
#include "ThrusterStern.h"

namespace Vektor2 {

static void vsp1_adapter(const uint16_t* inputs,
                         uint16_t* outputs,
                         const void* parameter_store)
{
    const auto& store = *static_cast<const VSP1ParameterStore*>(parameter_store);
    const Components::VSP1Params params = store.values();
    Components::vsp1_loop(inputs, outputs, params);
}

static void vsp2_adapter(const uint16_t* inputs,
                         uint16_t* outputs,
                         const void* parameter_store)
{
    const auto& store = *static_cast<const VSP2ParameterStore*>(parameter_store);
    const Components::VSP2Params params = store.values();
    Components::vsp2_loop(inputs, outputs, params);
}

static void thruster_bow_adapter(const uint16_t* inputs,
                                 uint16_t* outputs,
                                 const void* parameter_store)
{
    const auto& store = *static_cast<const ThrusterBowParameterStore*>(parameter_store);
    const ThrusterBowParams params = store.values();
    thruster_bow_loop(inputs, outputs, &params);
}

static void thruster_stern_adapter(const uint16_t* inputs,
                                   uint16_t* outputs,
                                   const void* parameter_store)
{
    const auto& store = *static_cast<const ThrusterSternParameterStore*>(parameter_store);
    const ThrusterSternParams params = store.values();
    thruster_stern_loop(inputs, outputs, &params);
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
        "THRBOW",
        1,
        1,
        thruster_bow_adapter,
    },
    {
        ComponentId::ThrusterStern,
        "ThrusterStern",
        "THRSTN",
        1,
        1,
        thruster_stern_adapter,
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
    AP_SUBGROUPINFO(thruster_bow_params, "THRBOW_", 5, Logic, ThrusterBowParameterStore),
    AP_SUBGROUPINFO(thruster_stern_params, "THRSTN_", 6, Logic, ThrusterSternParameterStore),

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
    AP_GROUPEND
};

void Logic::begin_cycle()
{
    for (uint8_t component = 0; component < component_count; component++) {
        for (uint8_t port = 0; port < max_inputs; port++) {
            _runtime[component].inputs[port] = 0;
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
    const ComponentDefinition& def = component_definitions[index];
    const void* params = parameter_store(id);
    if (params == nullptr) {
        return;
    }

    def.loop(_runtime[index].inputs, _runtime[index].outputs, params);
}

bool Logic::set_input(ComponentId id, uint8_t port, uint16_t value_us)
{
    if (!valid(id)) {
        return false;
    }

    const uint8_t index = uint8_t(id);
    if (port >= component_definitions[index].input_count) {
        return false;
    }

    _runtime[index].inputs[port] = value_us;
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
