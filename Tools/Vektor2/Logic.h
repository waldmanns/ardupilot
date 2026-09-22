#pragma once

#include "ComponentParameters.h"

#include <AP_Param/AP_Param.h>
#include <stdint.h>

namespace Vektor2 {

enum class ComponentId : uint8_t {
    VSP1 = 0,
    VSP2 = 1,
    Count
};

using ComponentLoopFn = void (*)(const uint16_t* inputs,
                                 uint16_t* outputs,
                                 const void* parameter_store);

struct ComponentDefinition {
    ComponentId id;
    const char* name;
    const char* prefix;
    uint8_t input_count;
    uint8_t output_count;
    ComponentLoopFn loop;
};

class Logic {
public:
    static constexpr uint8_t component_count = uint8_t(ComponentId::Count);
    static constexpr uint8_t max_inputs = 2;
    static constexpr uint8_t max_outputs = 2;

    static const AP_Param::GroupInfo var_info[];

    void begin_cycle();
    void run(ComponentId id);

    bool set_input(ComponentId id, uint8_t port, uint16_t value_us);
    uint16_t input(ComponentId id, uint8_t port) const;
    uint16_t output(ComponentId id, uint8_t port) const;

    static const ComponentDefinition& definition(ComponentId id);
    static bool valid(ComponentId id)
    {
        return uint8_t(id) < component_count;
    }

    // Persistent parameters belong to the framework, not to VSP1.cpp/VSP2.cpp.
    // The AP_Param prefixes are provided by Logic::var_info.
    VSP1ParameterStore vsp1_params;
    VSP2ParameterStore vsp2_params;

private:
    struct Runtime {
        uint16_t inputs[max_inputs] {};
        uint16_t outputs[max_outputs] {};
    };

    const void* parameter_store(ComponentId id) const;

    Runtime _runtime[component_count] {};
};

} // namespace Vektor2
