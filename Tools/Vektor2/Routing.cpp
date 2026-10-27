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

    // Also compile an order with the VSP pair collapsed to one node. A path
    // from either VSP into the other makes paired control impossible: preserve
    // ordinary routing, but inhibit heading hold rather than apply it twice.
    static_assert(uint8_t(ComponentId::VSP1) == 0 && uint8_t(ComponentId::VSP2) == 1,
                  "Paired routing requires the first two components to be VSPs");
    constexpr uint8_t groups = Logic::component_count - 1;
    bool group_edge[groups][groups] {};
    uint8_t group_indegree[groups] {};
    bool group_emitted[groups] {};
    _heading_routes_valid = true;
    for (uint8_t from = 0; from < Logic::component_count; from++) {
        for (uint8_t to = 0; to < Logic::component_count; to++) {
            if (!edge[from][to]) {
                continue;
            }
            const uint8_t a = from < 2 ? 0 : from - 1;
            const uint8_t b = to < 2 ? 0 : to - 1;
            if (a == b) {
                _heading_routes_valid = false;
            } else if (!group_edge[a][b]) {
                group_edge[a][b] = true;
                group_indegree[b]++;
            }
        }
    }
    uint8_t paired_count = 0;
    for (uint8_t step = 0; step < groups && _heading_routes_valid; step++) {
        int8_t next = -1;
        for (uint8_t n = 0; n < groups; n++) {
            if (!group_emitted[n] && group_indegree[n] == 0) {
                next = int8_t(n);
                break;
            }
        }
        if (next < 0) {
            _heading_routes_valid = false;
            break;
        }
        group_emitted[next] = true;
        if (next == 0) {
            _paired_execution_order[paired_count++] = ComponentId::VSP1;
            _paired_execution_order[paired_count++] = ComponentId::VSP2;
        } else {
            _paired_execution_order[paired_count++] = ComponentId(next + 1);
        }
        for (uint8_t n = 0; n < groups; n++) {
            if (group_edge[next][n]) {
                group_indegree[n]--;
            }
        }
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
        if (source_valid != nullptr) {
            *source_valid = logic.output_valid(id);
        }
        return value_us != 0;
    }
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
    bool thruster_dependencies = false;
    for (const auto& route : _routes) {
        if (!route.enabled || route.consumer.kind != ConsumerKind::ComponentInput) {
            continue;
        }
        if (route.consumer.instance >= uint8_t(ComponentId::ThrusterBow)) {
            thruster_routes |= 1U << (route.consumer.instance - uint8_t(ComponentId::ThrusterBow));
        }
        thruster_dependencies |= route.source.kind == SourceKind::ComponentOutput &&
            (route.source.instance >= uint8_t(ComponentId::ThrusterBow) ||
             route.consumer.instance >= uint8_t(ComponentId::ThrusterBow));
    }
    logic.set_thruster_routes(thruster_routes, _revision);
    logic.begin_cycle();
    // Independent thruster assistance needs one coherent input snapshot. Do
    // not add a feedback path through component chaining; ordinary manual
    // routing remains available, but heading is inhibited for that topology.
    const bool independent = logic.heading_enabled() &&
        (thruster_routes != 0 || !logic.vhdg_params.values().vsp_enabled);
    const bool use_vsps = logic.vhdg_params.values().vsp_enabled != 0;
    if (independent && !thruster_dependencies && (!use_vsps || _heading_routes_valid)) {
        for (uint8_t n = 0; n < Logic::component_count; n++) {
            fill_inputs(ComponentId(n), rc, logic);
        }
        logic.run_heading_group(rc.valid(), _revision);
        if (!use_vsps) {
            // VSPs retain their own routing/VRS and cannot inhibit thruster yaw.
            for (const auto component : _execution_order) {
                if (uint8_t(component) < uint8_t(ComponentId::ThrusterBow)) {
                    fill_inputs(component, rc, logic);
                    logic.run(component);
                }
            }
        }
    } else if (independent) {
        logic.update_heading_bypass(rc.valid(), false, _revision);
        for (const auto component : _execution_order) {
            fill_inputs(component, rc, logic);
            logic.run(component);
        }
    } else {
        const bool paired = logic.heading_enabled() && _heading_routes_valid;
        if (!paired) {
            logic.update_heading_bypass(rc.valid(), _heading_routes_valid, _revision);
        }
        for (uint8_t order_index = 0; order_index < Logic::component_count; order_index++) {
            const ComponentId component = paired ? _paired_execution_order[order_index] :
                                                  _execution_order[order_index];
            if (paired && component == ComponentId::VSP1) {
                fill_inputs(ComponentId::VSP1, rc, logic);
                fill_inputs(ComponentId::VSP2, rc, logic);
                logic.run_vsp_pair(rc.valid(), _revision);
                order_index++; // the paired order always places VSP2 immediately next
                continue;
            }

            fill_inputs(component, rc, logic);
            logic.run(component);
        }
    }

    // All producers have run exactly once. Requests from these samples take
    // effect at the next begin_cycle(), including feedback from VSP outputs.
    sample_functions(rc, logic, function_routes);

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

        if (!sample_source(route.source, rc, logic, source_us)) {
            if (_rc_loss_policy == RcLossPolicy::HoldLast &&
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
    return a.enabled == b.enabled && a.source.kind == b.source.kind &&
           a.source.instance == b.source.instance && a.source.port == b.source.port &&
           a.input_min_us == b.input_min_us && a.input_max_us == b.input_max_us &&
           a.output_min_us == b.output_min_us && a.output_max_us == b.output_max_us &&
           a.reversed == b.reversed;
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
