#include "RouteParameters.h"

#include "Logic.h"
#include "PwmOut.h"
#include "Routing.h"

namespace Vektor2 {

const AP_Param::GroupInfo RouteSlotParameters::var_info[] = {
    // @Param: SRC
    // @DisplayName: Route source
    // @Description: 0 disabled; 1..18 RC1..RC18; 101/102 VSP1 outputs; 111/112 VSP2 outputs.
    // @User: Standard
    AP_GROUPINFO("SRC", 1, RouteSlotParameters, source, 0),

    // @Param: DST
    // @DisplayName: Route destination
    // @Description: 0 disabled; 1..32 PWM1..PWM32; 101/102 VSP1 inputs; 111/112 VSP2 inputs.
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
    if (code >= 1 && code <= RcInput::max_channels) {
        endpoint = SourceEndpoint::rc(uint8_t(code - 1));
        return true;
    }

    if (code >= 101) {
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
    if (code >= 1 && code <= max_pwm_channels) {
        endpoint = ConsumerEndpoint::pwm(uint8_t(code - 1));
        return true;
    }

    if (code >= 101) {
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
    routing.clear_all(&pwm);

    bool all_valid = true;
    for (uint8_t i = 0; i < max_routes; i++) {
        _last_source[i] = current_source[i];
        _last_destination[i] = current_destination[i];

        if (current_source[i] == 0 || current_destination[i] == 0) {
            continue;
        }

        Route route;
        if (!decode_source(current_source[i], route.source) ||
            !decode_destination(current_destination[i], route.consumer)) {
            all_valid = false;
            continue;
        }

        // Standard parameter routes are direct microsecond routes. The Route
        // type keeps mapping fields available for future local/programmatic
        // uses, but the baseline MAVLink configuration does not expose them.
        route.input_min_us = 1000;
        route.input_max_us = 2000;
        route.output_min_us = 1000;
        route.output_max_us = 2000;
        route.reversed = false;
        route.enabled = true;

        if (!routing.set_route(i, route)) {
            all_valid = false;
        }
    }

    _have_snapshot = true;
    _last_apply_valid = all_valid;
    return all_valid;
}

} // namespace Vektor2
