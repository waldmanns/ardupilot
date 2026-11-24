#include "Routing.h"
#include <AP_HAL/AP_HAL.h>
#include <math.h>

namespace Vektor2 {

static uint8_t thruster_output_mask(const Route& route)
{
    if (route.source.kind != SourceKind::ComponentOutput ||
        !is_tunnel_thruster(ComponentId(route.source.instance))) {
        return 0;
    }
    return 1U << (route.source.instance - uint8_t(ComponentId::ThrusterBow));
}

// Bit per physical automatic rod, plus optional bow. RPM is independent.
static uint8_t vsp_output_bit(const Route& route, bool bow)
{
    if (route.source.kind != SourceKind::ComponentOutput) return 0;
    const auto id=ComponentId(route.source.instance);
    if (is_vsp(id) && route.source.port<2) return 1U << (2*route.source.instance+route.source.port);
    return bow && id==ComponentId::ThrusterBow ? 16 : 0;
}
bool Routing::vsp_routes_ready(const PwmOut& pwm, bool bow) const
{
    uint8_t outputs=0, inputs=0;
    for (const auto& r:_routes) {
        if (!r.enabled) continue;
        const uint8_t bit=vsp_output_bit(r,bow);
        if (bit) {
            // Post-prepare automatic replacement is safe only for terminal
            // PWM consumers. No sequencer/function/component feedback paths.
            if (r.consumer.kind!=ConsumerKind::PwmOutput || (outputs&bit) ||
                !pwm.heading_output_ready(r.consumer.port)) return false;
            outputs|=bit;
        }
        if (r.consumer.kind==ConsumerKind::ComponentInput) {
            const auto id=ComponentId(r.consumer.instance);
            uint8_t ib=0;
            if (is_vsp(id) && r.consumer.port<2) ib=1U<<(2*r.consumer.instance+r.consumer.port);
            if (bow && id==ComponentId::ThrusterBow) ib=16;
            if (ib) {
                if (r.source.kind!=SourceKind::RcInput || (inputs&ib)) return false;
                inputs|=ib;
            }
        }
    }
    const uint8_t required=bow?31:15;
    return outputs==required && inputs==required;
}

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

    // AZ2 actuator commands are not generic demand signals. Keep their
    // logical neutral and endpoints intact; calibrate in PWM channel params.
    if (route.source.kind == SourceKind::ComponentOutput &&
        is_azipod(ComponentId(route.source.instance)) &&
        (route.consumer.kind != ConsumerKind::PwmOutput || route.reversed ||
         route.input_min_us != 1000 || route.input_max_us != 2000 ||
         route.output_min_us != 1000 || route.output_max_us != 2000)) {
        return false;
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
        // VSP routes use the mapped endpoints for overtravel on a valid source.
        const bool source_valid = producer_valid &&
            (is_vsp(component) || (source_us >= VSP_PWM_MIN && source_us <= VSP_PWM_MAX));
        logic.set_input(component, route.consumer.port, routed_us, source_valid);
    }
}

uint8_t Routing::azipod_ready_mask(const Logic& logic, const PwmOut& pwm) const
{
    uint8_t count[2][2] {};
    uint8_t bad = 0;
    for (const auto& route : _routes) {
        const auto id = ComponentId(route.source.instance);
        if (!route.enabled || route.consumer.kind != ConsumerKind::PwmOutput ||
            route.source.kind != SourceKind::ComponentOutput || !is_azipod(id) ||
            !logic.component_selected(id)) { continue; }
        const uint8_t n = uint8_t(id) - uint8_t(ComponentId::AZI1);
        ++count[n][route.source.port];
        if (!pwm.heading_output_ready(route.consumer.port)) { bad |= 1U << n; }
    }
    uint8_t ready = 0;
    for (uint8_t n = 0; n < 2; ++n) {
        if (!(bad & (1U << n)) && count[n][0] == 1 && count[n][1] == 1) { ready |= 1U << n; }
    }
    return ready;
}

Routing::CalibrationResult Routing::azipod_calibration(uint32_t now, uint8_t system,
    uint8_t component, uint8_t link, const float (&f)[7], const Logic& logic, const PwmOut& pwm)
{
    using Result = CalibrationResult;
    // USER_3: operation, pod, logical steering PWM, signed motor %, token, 0, 0.
    for (float value : f) {
        if (!isfinite(value) || fabsf(value - truncf(value)) > 0.0f) { return Result::Invalid; }
    }
    if (!system || !component || f[0] < 1 || f[0] > 3 || f[1] < 1 || f[1] > 2 ||
        f[2] < 1000 || f[2] > 2000 || f[3] < -20 || f[3] > 20 ||
        f[4] < 1 || f[4] > 65535 || fabsf(f[5]) > 0.0f || fabsf(f[6]) > 0.0f) { return Result::Invalid; }
    const uint8_t operation = uint8_t(f[0]);
    const int8_t motor_percent = int8_t(f[3]);
    if (operation != 2 && motor_percent != 0) { return Result::Invalid; }
    if (_calibration.token && (now - _calibration.last_ms >= calibration_timeout_ms ||
        _calibration.revision != _revision)) { _calibration.token = 0; }
    const bool owner = _calibration.system == system && _calibration.component == component &&
        _calibration.link == link && _calibration.token == uint16_t(f[4]) && _calibration.pod == uint8_t(f[1]);
    if (_calibration.token && !owner) { return Result::Busy; }
    if (operation == 3) { _calibration.token = 0; return Result::Accepted; }
    if (operation == 2 && !_calibration.token) { return Result::NoSession; }
    const auto id = ComponentId(uint8_t(ComponentId::AZI1) + uint8_t(f[1]) - 1);
    const auto* drive = logic.azipod_controller(id);
    if (!drive || !drive->calibration_valid() || !(azipod_ready_mask(logic, pwm) & (1U << (uint8_t(f[1]) - 1)))) {
        _calibration.token = 0; return Result::Unavailable;
    }
    if (motor_percent < 0 && !drive->reversible()) { return Result::Invalid; }
    // Changing direction requires a zero motor command first. No steering solver or slew in calibration.
    if (_calibration.token && _calibration.motor * motor_percent < 0) { return Result::Invalid; }
    _calibration.last_ms = now;
    _calibration.revision = _revision;
    _calibration.token = uint16_t(f[4]);
    _calibration.steering = uint16_t(f[2]);
    _calibration.motor = motor_percent;
    _calibration.system = system;
    _calibration.component = component;
    _calibration.link = link;
    _calibration.pod = uint8_t(f[1]);
    return Result::Accepted;
}

void Routing::prepare(const RcInput& rc, Logic& logic, const PwmOut& pwm,
                      const VspRpmGovernor::Feedback (&shaft_rpm)[2])
{
    const Route* function_routes[Functions::count] {};
    prepare_functions(logic, function_routes);
    uint8_t thruster_routes = 0;
    uint8_t thruster_outputs = 0;
    uint8_t unavailable_outputs = 0;
    for (const auto& route : _routes) {
        if (!route.enabled) {
            continue;
        }
        if (route.consumer.kind == ConsumerKind::ComponentInput &&
            is_tunnel_thruster(ComponentId(route.consumer.instance))) {
            thruster_routes |= 1U << (route.consumer.instance - uint8_t(ComponentId::ThrusterBow));
        }
        if (route.consumer.kind == ConsumerKind::PwmOutput) {
            const uint8_t thruster = thruster_output_mask(route);
            thruster_outputs |= thruster;
            if (thruster && !pwm.heading_output_ready(route.consumer.port)) {
                unavailable_outputs |= thruster;
            }
        }
    }
    const uint8_t azi_ready = azipod_ready_mask(logic, pwm);
    if (_calibration.token && (AP_HAL::millis() - _calibration.last_ms >= calibration_timeout_ms ||
        _calibration.revision != _revision || !(azi_ready & (1U << (_calibration.pod - 1))))) {
        _calibration.token = 0;
    }
    logic.set_azipod_routes(azi_ready, rc.valid());
    logic.set_thruster_routes(thruster_routes, _revision);
    logic.begin_cycle(shaft_rpm);
    for (const auto component : _execution_order) {
        if (!logic.component_selected(component)) {
            continue;
        }
        fill_inputs(component, rc, logic);
        if (_calibration.token && is_azipod(component)) {
            const bool selected = uint8_t(component) - uint8_t(ComponentId::AZI1) + 1 == _calibration.pod;
            logic.calibrate_azipod(component, selected, _calibration.steering, _calibration.motor);
        } else {
            logic.run(component);
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!(thruster_outputs & (1U << i))) {
            logic.set_thruster_output_reasons(i, ThrusterControl::OutputRouteMissing);
        } else if (unavailable_outputs & (1U << i)) {
            logic.set_thruster_output_reasons(i, ThrusterControl::OutputRejected);
        }
    }

    // All producers have run exactly once. Requests from these samples take
    // effect at the next begin_cycle(), including feedback from VSP outputs.
    sample_functions(rc, logic, function_routes);
    sample_sequencer(rc, logic);
}

bool Routing::write_routed(const Route& route, PwmOut& pwm, uint16_t source_us)
{
    return pwm.write_us(route.consumer.port,
                        map_us(source_us, route.input_min_us, route.input_max_us,
                               route.output_min_us, route.output_max_us, route.reversed));
}

uint8_t Routing::publish_outputs(const RcInput& rc, Logic& logic, PwmOut& pwm, uint8_t heading_mask, bool vsp_auto, bool vsp_bow, bool vsp_guard)
{
    uint32_t next_output_mask = 0;
    uint8_t heading_written = 0;
    uint8_t heading_failed = 0;
    uint8_t vsp_written=0, vsp_failed=0;
    if (heading_mask || vsp_auto || vsp_guard) {
        pwm.begin_batch();
    }

    // AZI: write steering before motor, regardless of route slot order.
    // If steering is missing/rejected, only a motor stop may be published.
    for (uint8_t n = 0; n < 2; ++n) {
        const auto id = ComponentId(uint8_t(ComponentId::AZI1) + n);
        if (!logic.component_selected(id)) { continue; }
        bool steering_written = false;
        for (uint8_t port = 0; port < 2; ++port) {
            for (const auto& route : _routes) {
                if (!route.enabled || route.consumer.kind != ConsumerKind::PwmOutput ||
                    route.source.kind != SourceKind::ComponentOutput ||
                    route.source.instance != uint8_t(id) || route.source.port != port ||
                    !logic.output_valid(id, port)) { continue; }
                const auto channel = route.consumer.port;
                if (!pwm.heading_output_ready(channel)) {
                    logic.azipod_output_failed(id);
                    if (_calibration.token && _calibration.pod == n + 1) { _calibration.token = 0; }
                    continue;
                }
                const uint16_t command = port == 1 && !steering_written ?
                    logic.azipod_motor_stop(id) : logic.output(id, port);
                if (write_routed(route, pwm, command)) {
                    next_output_mask |= 1UL << channel;
                    if (port == 0) { steering_written = true; }
                } else {
                    logic.azipod_output_failed(id);
                    if (_calibration.token && _calibration.pod == n + 1) { _calibration.token = 0; }
                }
            }
        }
    }

    // Final consumer stage: direct RC or component output can drive PWM.
    for (uint8_t i = 0; i < capacity; i++) {
        const Route& route = _routes[i];
        if (!route.enabled || route.consumer.kind != ConsumerKind::PwmOutput) {
            continue;
        }

        if (route.source.kind == SourceKind::ComponentOutput &&
            is_azipod(ComponentId(route.source.instance))) { continue; }
        const uint8_t channel = route.consumer.port;
        const uint32_t mask = 1UL << channel;
        const uint8_t heading = heading_mask & thruster_output_mask(route);
        const uint8_t vsp_bit = (vsp_auto || vsp_guard) ? vsp_output_bit(route,vsp_bow) : 0;
        uint16_t source_us = 0;

        bool source_valid = false;
        if (vsp_guard && vsp_bit) {
            source_us=logic.output(ComponentId(route.source.instance),route.source.port);
            source_valid=true;
        } else if (!sample_source(route.source, rc, logic, source_us, &source_valid) || !source_valid) {
            heading_failed |= heading;
            vsp_failed |= vsp_bit;
            // HoldLast is restricted to direct RC routes. Never latch stale
            // component-derived propulsion commands on invalid input.
            if (route.source.kind == SourceKind::RcInput &&
                _rc_loss_policy == RcLossPolicy::HoldLast &&
                (_driven_output_mask & mask) != 0) {
                next_output_mask |= mask;
            }
            continue;
        }

        if (((!heading && !vsp_bit) || pwm.heading_output_ready(channel)) && write_routed(route, pwm, source_us)) {
            next_output_mask |= mask;
            heading_written |= heading;
            vsp_written |= vsp_bit;
        } else {
            heading_failed |= heading;
            vsp_failed |= vsp_bit;
        }
    }

    if (vsp_auto) vsp_failed |= (vsp_bow?31:15) & ~vsp_written;
    if (vsp_auto && vsp_failed) {
        logic.vsp_allocator()->reject();
        logic.neutral_vsp_allocation(vsp_bow);
        for (const auto& r:_routes) {
            if (!r.enabled || r.consumer.kind!=ConsumerKind::PwmOutput || !vsp_output_bit(r,vsp_bow)) continue;
            const auto mask=1UL<<r.consumer.port;
            if (pwm.heading_output_ready(r.consumer.port) && write_routed(r,pwm,logic.output(ComponentId(r.source.instance),r.source.port))) {
                next_output_mask|=mask;
            } else { next_output_mask&=~mask; pwm.disable(r.consumer.port); }
        }
    }
    heading_failed |= heading_mask & ~heading_written;
    if (heading_failed) {
        // Roll back every automatic command before the HAL publishes the batch.
        for (uint8_t i = 0; i < 2; i++) {
            if (heading_mask & (1U << i)) {
                logic.release_thruster_heading(i);
            }
            if (heading_failed & (1U << i)) {
                logic.set_thruster_output_reasons(i, ThrusterControl::OutputRejected);
            }
        }
        for (const auto& route : _routes) {
            if (route.enabled && route.consumer.kind == ConsumerKind::PwmOutput &&
                (heading_mask & thruster_output_mask(route))) {
                const uint32_t mask = 1UL << route.consumer.port;
                const auto source = ComponentId(route.source.instance);
                if (pwm.heading_output_ready(route.consumer.port) &&
                    write_routed(route, pwm, logic.output(source, 0))) {
                    next_output_mask |= mask;
                } else {
                    next_output_mask &= ~mask;
                    pwm.disable(route.consumer.port);
                }
            }
        }
    }
    const uint32_t stale_output_mask = _driven_output_mask & ~next_output_mask;
    disable_mask(stale_output_mask, pwm);
    _driven_output_mask = next_output_mask;
    if (heading_mask || vsp_auto || vsp_guard) {
        pwm.end_batch();
    }
    return vsp_auto ? (vsp_failed ? 0 : 4) : (heading_failed ? 0 : heading_mask);
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
