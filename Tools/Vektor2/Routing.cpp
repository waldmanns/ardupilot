#include "Routing.h"

namespace Vektor2 {

bool Routing::same_consumer(const ConsumerEndpoint& a, const ConsumerEndpoint& b) const
{
    if (a.kind != b.kind) {
        return false;
    }

    if (a.kind == ConsumerKind::PwmOutput) {
        return a.port == b.port;
    }

    return a.instance == b.instance && a.port == b.port;
}

bool Routing::consumer_claimed_by_other(uint8_t index,
                                        const ConsumerEndpoint& consumer) const
{
    for (uint8_t i = 0; i < capacity; i++) {
        if (i == index) {
            continue;
        }

        const Route& existing = _routes[i];
        if (existing.enabled && same_consumer(existing.consumer, consumer)) {
            return true;
        }
    }
    return false;
}

bool Routing::validate_endpoint_pair(const Route& route) const
{
    switch (route.source.kind) {
    case SourceKind::RcInput:
        if (route.source.port >= RcInput::max_channels) {
            return false;
        }
        break;

    case SourceKind::ComponentOutput: {
        const ComponentId id = ComponentId(route.source.instance);
        if (!Logic::valid(id) ||
            route.source.port >= Logic::definition(id).output_count) {
            return false;
        }
        break;
    }
    case SourceKind::SequencerOutput:
        if (route.source.port >= Sequencer::lane_count) {
            return false;
        }
        break;
    }

    switch (route.consumer.kind) {
    case ConsumerKind::PwmOutput:
        if (route.consumer.port >= max_pwm_channels) {
            return false;
        }
        break;

    case ConsumerKind::ComponentInput: {
        const ComponentId id = ComponentId(route.consumer.instance);
        if (!Logic::valid(id) ||
            route.consumer.port >= Logic::definition(id).input_count) {
            return false;
        }
        break;
    }

    case ConsumerKind::SequencerTrigger:
        if (route.consumer.port > Sequencer::shared_slot) {
            return false;
        }
        break;

    case ConsumerKind::FunctionInput:
        if (route.consumer.port >= Functions::count) {
            return false;
        }
        break;
    }

    return true;
}

bool Routing::rebuild_execution_order()
{
    bool edge[Logic::component_count][Logic::component_count] {};
    uint8_t indegree[Logic::component_count] {};

    for (uint8_t i = 0; i < capacity; i++) {
        const Route& route = _routes[i];
        if (!route.enabled ||
            route.source.kind != SourceKind::ComponentOutput ||
            route.consumer.kind != ConsumerKind::ComponentInput) {
            continue;
        }

        const uint8_t from = route.source.instance;
        const uint8_t to = route.consumer.instance;

        if (from >= Logic::component_count || to >= Logic::component_count) {
            return false;
        }

        if (!edge[from][to]) {
            edge[from][to] = true;
            indegree[to]++;
        }
    }

    ComponentId order[Logic::component_count] {};
    bool emitted[Logic::component_count] {};
    uint8_t count = 0;

    while (count < Logic::component_count) {
        int8_t next = -1;
        for (uint8_t i = 0; i < Logic::component_count; i++) {
            if (!emitted[i] && indegree[i] == 0) {
                next = int8_t(i);
                break;
            }
        }

        if (next < 0) {
            return false; // component dependency cycle
        }

        const uint8_t node = uint8_t(next);
        emitted[node] = true;
        order[count++] = ComponentId(node);

        for (uint8_t to = 0; to < Logic::component_count; to++) {
            if (edge[node][to] && indegree[to] > 0) {
                indegree[to]--;
            }
        }
    }

    for (uint8_t i = 0; i < Logic::component_count; i++) {
        _execution_order[i] = order[i];
    }

    _revision++;
    return true;
}

bool Routing::set_route(uint8_t index, const Route& route)
{
    if (index >= capacity ||
        route.input_min_us >= route.input_max_us ||
        route.output_min_us >= route.output_max_us ||
        !validate_endpoint_pair(route)) {
        return false;
    }

    if (route.enabled && consumer_claimed_by_other(index, route.consumer)) {
        return false;
    }

    const Route previous = _routes[index];
    _routes[index] = route;

    if (!rebuild_execution_order()) {
        _routes[index] = previous;
        rebuild_execution_order();
        return false;
    }

    return true;
}

void Routing::clear_route(uint8_t index, PwmOut* pwm)
{
    if (index >= capacity) {
        return;
    }

    if (pwm != nullptr && _routes[index].enabled &&
        _routes[index].consumer.kind == ConsumerKind::PwmOutput) {
        const uint8_t channel = _routes[index].consumer.port;
        const uint32_t mask = 1UL << channel;
        pwm->disable(channel);
        _driven_output_mask &= ~mask;
    }

    _routes[index] = Route{};
    rebuild_execution_order();
}

void Routing::clear_all(PwmOut* pwm)
{
    _function_error_mask = 0;
    if (pwm != nullptr) {
        disable_mask(_driven_output_mask, *pwm);
        _driven_output_mask = 0;
    }

    for (uint8_t i = 0; i < capacity; i++) {
        _routes[i] = Route{};
    }
    rebuild_execution_order();
}

const Route* Routing::route(uint8_t index) const
{
    return index < capacity ? &_routes[index] : nullptr;
}

uint8_t Routing::configure_passthrough(uint8_t count, uint16_t min_us, uint16_t max_us)
{
    if (min_us >= max_us) {
        return 0;
    }

    uint8_t configured = 0;
    const uint8_t limit = count > capacity ? capacity : count;

    for (uint8_t i = 0; i < limit; i++) {
        if (i >= RcInput::max_channels || i >= max_pwm_channels) {
            break;
        }

        Route route;
        route.source = SourceEndpoint::rc(i);
        route.consumer = ConsumerEndpoint::pwm(i);
        route.input_min_us = min_us;
        route.input_max_us = max_us;
        route.output_min_us = min_us;
        route.output_max_us = max_us;
        route.enabled = true;

        if (!set_route(i, route)) {
            break;
        }
        configured++;
    }

    return configured;
}

uint16_t Routing::map_us(uint16_t value,
                         uint16_t in_min,
                         uint16_t in_max,
                         uint16_t out_min,
                         uint16_t out_max,
                         bool reversed)
{
    if (value < in_min) {
        value = in_min;
    } else if (value > in_max) {
        value = in_max;
    }

    const uint32_t input_span = uint32_t(in_max) - in_min;
    const uint32_t output_span = uint32_t(out_max) - out_min;
    const uint32_t input_offset = uint32_t(value) - in_min;
    const uint32_t scaled = (input_offset * output_span + input_span / 2U) / input_span;

    if (reversed) {
        return uint16_t(uint32_t(out_max) - scaled);
    }
    return uint16_t(uint32_t(out_min) + scaled);
}

bool Routing::sample_source(const SourceEndpoint& source,
                            const RcInput& rc,
                            const Logic& logic,
                            uint16_t& value_us, bool* source_valid) const
{
    if (source_valid != nullptr) {
        *source_valid = false;
    }
    switch (source.kind) {
    case SourceKind::RcInput:
        if (!rc.valid() || source.port >= rc.channel_count()) {
            return false;
        }
        value_us = rc.channel_us(source.port);
        if (source_valid != nullptr) {
            *source_valid = true;
        }
        return value_us != 0;

    case SourceKind::ComponentOutput: {
        const ComponentId id = ComponentId(source.instance);
        if (!Logic::valid(id)) {
            return false;
        }
        value_us = logic.output(id, source.port);
        const bool valid_output = logic.output_valid(id, source.port);
        if (source_valid != nullptr) {
            *source_valid = valid_output;
        }
        return valid_output && value_us != 0;
    }

    case SourceKind::SequencerOutput:
        if (_sequencer == nullptr) {
            return false;
        }
        const auto& output = _sequencer->status(source.port);
        value_us = output.pulse;
        if (source_valid != nullptr) {
            *source_valid = output.valid;
        }
        return output.valid;
    }

    return false;
}

void Routing::disable_mask(uint32_t mask, PwmOut& pwm) const
{
    for (uint8_t channel = 0; channel < max_pwm_channels; channel++) {
        if ((mask & (1UL << channel)) != 0) {
            pwm.disable(channel);
        }
    }
}

void Routing::fill_inputs(ComponentId component, const RcInput& rc, Logic& logic) const
{
    for (uint8_t i = 0; i < capacity; i++) {
        const Route& route = _routes[i];
        if (!route.enabled ||
            route.consumer.kind != ConsumerKind::ComponentInput ||
            route.consumer.instance != uint8_t(component)) {
            continue;
        }
        uint16_t source_us = 0;
        bool producer_valid = false;
        if (!sample_source(route.source, rc, logic, source_us, &producer_valid)) {
            continue;
        }
        const uint16_t routed_us = map_us(source_us,
                                          route.input_min_us,
                                          route.input_max_us,
                                          route.output_min_us,
                                          route.output_max_us,
                                          route.reversed);
        const bool source_valid = source_us >= VSP_PWM_MIN && source_us <= VSP_PWM_MAX && producer_valid;
        logic.set_input(component, route.consumer.port, routed_us, source_valid);
    }
}

void Routing::update(const RcInput& rc, Logic& logic, PwmOut& pwm)
{
    const Route* function_routes[Functions::count] {};
    prepare_functions(logic, function_routes);
    uint8_t thruster_routes = 0;
    for (const auto& route : _routes) {
        if (route.enabled && route.consumer.kind == ConsumerKind::ComponentInput &&
            route.consumer.instance >= uint8_t(ComponentId::ThrusterBow)) {
            thruster_routes |= 1U << (route.consumer.instance - uint8_t(ComponentId::ThrusterBow));
        }
    }
    logic.set_thruster_routes(thruster_routes, _revision);
    logic.begin_cycle();
    for (const auto component : _execution_order) {
        fill_inputs(component, rc, logic);
        logic.run(component);
    }

    // All producers have run exactly once. Requests from these samples take
    // effect at the next begin_cycle(), including feedback from VSP outputs.
    sample_functions(rc, logic, function_routes);
    sample_sequencer(rc, logic);

    uint32_t next_output_mask = 0;

    // Final consumer stage: direct RC or component output can drive PWM.
    for (uint8_t i = 0; i < capacity; i++) {
        const Route& route = _routes[i];
        if (!route.enabled || route.consumer.kind != ConsumerKind::PwmOutput) {
            continue;
        }

        const uint8_t channel = route.consumer.port;
        const uint32_t mask = 1UL << channel;
        uint16_t source_us = 0;

        bool source_valid = false;
        if (!sample_source(route.source, rc, logic, source_us, &source_valid) || !source_valid) {
            // HoldLast is restricted to direct RC routes. Never latch stale
            // component-derived propulsion commands on invalid input.
            if (route.source.kind == SourceKind::RcInput &&
                _rc_loss_policy == RcLossPolicy::HoldLast &&
                (_driven_output_mask & mask) != 0) {
                next_output_mask |= mask;
            }
            continue;
        }

        const uint16_t output_us = map_us(source_us,
                                          route.input_min_us,
                                          route.input_max_us,
                                          route.output_min_us,
                                          route.output_max_us,
                                          route.reversed);
        if (pwm.write_us(channel, output_us)) {
            next_output_mask |= mask;
        }
    }

    const uint32_t stale_output_mask = _driven_output_mask & ~next_output_mask;
    disable_mask(stale_output_mask, pwm);
    _driven_output_mask = next_output_mask;
}

static bool same_route(const Route& a, const Route& b)
{
    return a.enabled == b.enabled && a.consumer.kind == b.consumer.kind &&
           a.consumer.instance == b.consumer.instance && a.consumer.port == b.consumer.port &&
           a.source.kind == b.source.kind &&
           a.source.instance == b.source.instance && a.source.port == b.source.port &&
           a.input_min_us == b.input_min_us && a.input_max_us == b.input_max_us &&
           a.output_min_us == b.output_min_us && a.output_max_us == b.output_max_us &&
           a.reversed == b.reversed;
}

void Routing::sample_sequencer(const RcInput& rc, const Logic& logic) const
{
    if (_sequencer == nullptr) {
        return;
    }
    for (uint8_t target = 0; target <= Sequencer::shared_slot; target++) {
        _sequencer->sample(target, {});
    }
    for (const auto& route : _routes) {
        const uint8_t target = route.consumer.port;
        if (!route.enabled || route.consumer.kind != ConsumerKind::SequencerTrigger ||
            (_sequencer_errors & (1UL << target))) {
            continue;
        }
        Sequencer::Input input;
        bool valid = false;
        input.valid = sample_source(route.source, rc, logic, input.pulse, &valid) && valid;
        _sequencer->sample(target, input);
    }
}

void Routing::reconcile_sequencer(const Route (&previous)[capacity], uint32_t previous_errors)
{
    if (_sequencer == nullptr) {
        return;
    }
    uint32_t triggers = _sequencer_errors ^ previous_errors;
    uint32_t outputs = 0;
    uint8_t components = 0;
    const auto changes = [&](const Route (&old)[capacity], const Route (&current)[capacity]) {
        for (const auto& route : old) {
            if (!route.enabled) {
                continue;
            }
            bool present = false;
            for (const auto& candidate : current) {
                if (same_route(route, candidate)) {
                    present = true;
                    break;
                }
            }
            if (present) {
                continue;
            }
            if (route.source.kind == SourceKind::SequencerOutput) {
                outputs |= 1UL << route.source.port;
            }
            if (route.consumer.kind == ConsumerKind::SequencerTrigger) {
                triggers |= 1UL << route.consumer.port;
            } else if (route.consumer.kind == ConsumerKind::ComponentInput) {
                components |= 1U << route.consumer.instance;
            }
        }
    };
    changes(previous, _routes);
    changes(_routes, previous);

    // A rewire of a component feeding a trigger changes that trigger's
    // effective binding. Walk the existing (acyclic) component graph.
    for (uint8_t pass = 0; pass < Logic::component_count && components != 0; pass++) {
        for (const auto& route : _routes) {
            if (!route.enabled || route.source.kind != SourceKind::ComponentOutput ||
                !(components & (1U << route.source.instance))) {
                continue;
            }
            if (route.consumer.kind == ConsumerKind::SequencerTrigger) {
                triggers |= 1UL << route.consumer.port;
            } else if (route.consumer.kind == ConsumerKind::ComponentInput) {
                components |= 1U << route.consumer.instance;
            }
        }
    }
    _sequencer->routes_changed(triggers, outputs);
}

void Routing::prepare_functions(Logic& logic, const Route* (&routes)[Functions::count])
{
    for (const auto& route : _routes) {
        if (route.enabled && route.consumer.kind == ConsumerKind::FunctionInput) {
            routes[route.consumer.port] = &route;
        }
    }
    FunctionSettings settings[Functions::count];
    FunctionBinding bindings[Functions::count];
    logic.vfn_params.values(settings);
    for (uint8_t n = 0; n < Functions::count; n++) {
        const Route current = routes[n] == nullptr ? Route{} : *routes[n];
        if (!same_route(current, _function_routes[n])) {
            _function_routes[n] = current;
            _function_revisions[n]++;
        }
        bindings[n].revision = _function_revisions[n];
        bindings[n].routed = routes[n] != nullptr;
        bindings[n].error = (_function_error_mask & (1U << n)) != 0;
    }
    logic.vfn.prepare(AP_HAL::millis(), settings, bindings);
}

void Routing::sample_functions(const RcInput& rc, Logic& logic,
                              const Route* const (&routes)[Functions::count]) const
{
    FunctionInput inputs[Functions::count];
    for (uint8_t n = 0; n < Functions::count; n++) {
        if (routes[n] == nullptr) {
            continue;
        }
        const Route& route = *routes[n];
        uint16_t source_us = 0;
        bool producer_valid = false;
        inputs[n].valid = sample_source(route.source, rc, logic, source_us, &producer_valid) &&
            producer_valid && source_us >= Functions::minimum_us && source_us <= Functions::maximum_us;
        inputs[n].pulse_us = map_us(source_us, route.input_min_us, route.input_max_us,
                                    route.output_min_us, route.output_max_us, route.reversed);
    }
    logic.vfn.sample(inputs);
}

} // namespace Vektor2
