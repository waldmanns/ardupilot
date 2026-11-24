#include "RouteParameters.h"

#include "Logic.h"
#include "PwmOut.h"
#include "Routing.h"

namespace Vektor2 {

const AP_Param::GroupInfo RouteSlotParameters::var_info[] = {
    // @Param: SRC
    // @DisplayName: Route source
    // @Description: 0 disabled; 1..18 RC1..RC18; 101/102 VSP1 outputs; 103 VSP1 RPM_OUT; 111/112 VSP2 outputs; 113 VSP2 RPM_OUT; 121 ThrusterBow output; 131 ThrusterStern output; 141/142 AZI1 azimuth/motor; 151/152 AZI2 azimuth/motor (AZ2 direct PWM outputs).
    // @User: Standard
    AP_GROUPINFO("SRC", 1, RouteSlotParameters, source, 0),

    // @Param: DST
    // @DisplayName: Route destination
    // @Description: 0 disabled; 1..32 PWM1..PWM32; 101/102 VSP1 inputs; 103 VSP1 RPM_IN; 111/112 VSP2 inputs; 113 VSP2 RPM_IN; 121 ThrusterBow input; 131 ThrusterStern input; 141/142 AZI1 X/Y; 151/152 AZI2 X/Y; 201..210 VFN1..VFN10 inputs.
    // @User: Standard
    AP_GROUPINFO("DST", 2, RouteSlotParameters, destination, 0),

    AP_GROUPEND
};

RoutingParameters::RoutingParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define ROUTE_SLOT(n, idx) AP_SUBGROUPINFO(slots[(n)-1], #n "_", idx, RoutingParameters, RouteSlotParameters)
const AP_Param::GroupInfo RoutingParameters::var_info[] = {
    ROUTE_SLOT(1, 1),
    ROUTE_SLOT(2, 2),
    ROUTE_SLOT(3, 3),
    ROUTE_SLOT(4, 4),
    ROUTE_SLOT(5, 5),
    ROUTE_SLOT(6, 6),
    ROUTE_SLOT(7, 7),
    ROUTE_SLOT(8, 8),
    ROUTE_SLOT(9, 9),
    ROUTE_SLOT(10, 10),
    ROUTE_SLOT(11, 11),
    ROUTE_SLOT(12, 12),
    ROUTE_SLOT(13, 13),
    ROUTE_SLOT(14, 14),
    ROUTE_SLOT(15, 15),
    ROUTE_SLOT(16, 16),
    ROUTE_SLOT(17, 17),
    ROUTE_SLOT(18, 18),
    ROUTE_SLOT(19, 19),
    ROUTE_SLOT(20, 20),
    ROUTE_SLOT(21, 21),
    ROUTE_SLOT(22, 22),
    ROUTE_SLOT(23, 23),
    ROUTE_SLOT(24, 24),
    ROUTE_SLOT(25, 25),
    ROUTE_SLOT(26, 26),
    ROUTE_SLOT(27, 27),
    ROUTE_SLOT(28, 28),
    ROUTE_SLOT(29, 29),
    ROUTE_SLOT(30, 30),
    ROUTE_SLOT(31, 31),
    ROUTE_SLOT(32, 32),
    AP_GROUPEND
};
#undef ROUTE_SLOT

static bool decode_source(int16_t code, SourceEndpoint& endpoint)
{
    if (code >= 301 && code < 301 + Sequencer::lane_count) {
        endpoint = {SourceKind::SequencerOutput, 0, uint8_t(code - 301)};
        return true;
    }

    if (code >= 1 && code <= RcInput::max_channels) {
        endpoint = SourceEndpoint::rc(uint8_t(code - 1));
        return true;
    }

    if (code >= 101 && code < 101 + Logic::component_count * 10) {
        const int16_t compact = code - 101;
        const uint8_t component = uint8_t(compact / 10);
        const uint8_t port = uint8_t(compact % 10);
        const ComponentId id = ComponentId(component);
        if (Logic::valid(id) && port < Logic::definition(id).output_count) {
            endpoint = SourceEndpoint::component(id, port);
            return true;
        }
    }

    return false;
}

static bool decode_destination(int16_t code, ConsumerEndpoint& endpoint)
{
    if (code == 417 || (code >= 401 && code < 401 + Sequencer::lane_count)) {
        endpoint = {ConsumerKind::SequencerTrigger, 0,
                    code == 417 ? Sequencer::shared_slot : uint8_t(code - 401)};
        return true;
    }

    if (code >= 1 && code <= max_pwm_channels) {
        endpoint = ConsumerEndpoint::pwm(uint8_t(code - 1));
        return true;
    }

    if (code >= 201 && code < 201 + Functions::count) {
        endpoint = ConsumerEndpoint::function(uint8_t(code - 201));
        return true;
    }

    if (code >= 101 && code < 101 + Logic::component_count * 10) {
        const int16_t compact = code - 101;
        const uint8_t component = uint8_t(compact / 10);
        const uint8_t port = uint8_t(compact % 10);
        const ComponentId id = ComponentId(component);
        if (Logic::valid(id) && port < Logic::definition(id).input_count) {
            endpoint = ConsumerEndpoint::component(id, port);
            return true;
        }
    }

    return false;
}

bool RoutingParameters::sync(Routing& routing, PwmOut& pwm, bool force)
{
    bool changed = force || !_have_snapshot;

    int16_t current_source[max_routes] {};
    int16_t current_destination[max_routes] {};

    for (uint8_t i = 0; i < max_routes; i++) {
        current_source[i] = slots[i].source.get();
        current_destination[i] = slots[i].destination.get();
        if (!changed &&
            (current_source[i] != _last_source[i] ||
             current_destination[i] != _last_destination[i])) {
            changed = true;
        }
    }

    if (!changed) {
        return _last_apply_valid;
    }

    // Configuration changes are infrequent. Rebuild the tiny table from the
    // parameter snapshot so validation remains deterministic and old PWM
    // consumers are explicitly released.
    Route previous[max_routes];
    for (uint8_t i = 0; i < max_routes; i++) {
        const auto* current = routing.route(i);
        if (current != nullptr) {
            previous[i] = *current;
        }
    }
    const uint32_t previous_errors = routing.sequencer_errors();
    routing.clear_all(&pwm);

    bool all_valid = true;
    uint16_t function_errors = 0;
    uint32_t sequencer_errors = 0;
    for (uint8_t i = 0; i < max_routes; i++) {
        _last_source[i] = current_source[i];
        _last_destination[i] = current_destination[i];

        if (current_source[i] == 0 || current_destination[i] == 0) {
            continue;
        }

        Route route;
        const bool destination_valid = decode_destination(current_destination[i], route.consumer);
        const bool trigger = destination_valid && route.consumer.kind == ConsumerKind::SequencerTrigger;
        if (!decode_source(current_source[i], route.source) || !destination_valid) {
            all_valid = false;
            if (trigger) {
                sequencer_errors |= 1UL << route.consumer.port;
            }
            if (current_destination[i] >= 201 && current_destination[i] < 201 + Functions::count) {
                function_errors |= 1U << (current_destination[i] - 201);
            }
            continue;
        }

        // Standard parameter routes are direct microsecond routes. The Route
        // type keeps mapping fields available for future local/programmatic
        // uses, but the baseline MAVLink configuration does not expose them.
        const bool function = route.consumer.kind == ConsumerKind::FunctionInput;
        route.input_min_us = function ? Functions::minimum_us : 1000;
        route.input_max_us = function ? Functions::maximum_us : 2000;
        route.output_min_us = route.input_min_us;
        route.output_max_us = route.input_max_us;
        route.reversed = false;
        route.enabled = true;

        if (!routing.set_route(i, route)) {
            all_valid = false;
            if (trigger) {
                sequencer_errors |= 1UL << route.consumer.port;
            }
            if (function) {
                function_errors |= 1U << route.consumer.port;
            }
        }
    }

    routing.set_function_errors(function_errors);
    routing.set_sequencer_errors(sequencer_errors);
    routing.reconcile_sequencer(previous, previous_errors);
    _have_snapshot = true;
    _last_apply_valid = all_valid;
    return all_valid;
}

} // namespace Vektor2
