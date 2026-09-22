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
                            uint16_t& value_us) const
{
    switch (source.kind) {
    case SourceKind::RcInput:
        if (!rc.valid() || source.port >= rc.channel_count()) {
            return false;
        }
        value_us = rc.channel_us(source.port);
        return value_us != 0;

    case SourceKind::ComponentOutput: {
        const ComponentId id = ComponentId(source.instance);
        if (!Logic::valid(id)) {
            return false;
        }
        value_us = logic.output(id, source.port);
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

void Routing::update(const RcInput& rc, Logic& logic, PwmOut& pwm)
{
    logic.begin_cycle();

    // Fill and execute components in dependency order. This allows chains such
    // as RC -> VSP1 -> VSP2 -> PWM while rejecting cycles at route-set time.
    for (uint8_t order_index = 0; order_index < Logic::component_count; order_index++) {
        const ComponentId component = _execution_order[order_index];

        for (uint8_t i = 0; i < capacity; i++) {
            const Route& route = _routes[i];
            if (!route.enabled ||
                route.consumer.kind != ConsumerKind::ComponentInput ||
                route.consumer.instance != uint8_t(component)) {
                continue;
            }

            uint16_t source_us = 0;
            if (!sample_source(route.source, rc, logic, source_us)) {
                continue;
            }

            const uint16_t routed_us = map_us(source_us,
                                              route.input_min_us,
                                              route.input_max_us,
                                              route.output_min_us,
                                              route.output_max_us,
                                              route.reversed);
            logic.set_input(component, route.consumer.port, routed_us);
        }

        logic.run(component);
    }

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

} // namespace Vektor2
