#pragma once

#include "Config.h"
#include "Logic.h"
#include "PwmOut.h"
#include "RcInput.h"

#include <stdint.h>

namespace Vektor2 {

enum class SourceKind : uint8_t {
    RcInput = 0,
    ComponentOutput = 1,
};

struct SourceEndpoint {
    SourceKind kind = SourceKind::RcInput;
    uint8_t instance = 0; // ComponentId when kind == ComponentOutput
    uint8_t port = 0;     // RC channel or component output port

    SourceEndpoint() = default;
    constexpr SourceEndpoint(SourceKind endpoint_kind, uint8_t endpoint_instance, uint8_t endpoint_port) :
        kind(endpoint_kind),
        instance(endpoint_instance),
        port(endpoint_port)
    {}

    static SourceEndpoint rc(uint8_t channel)
    {
        return SourceEndpoint(SourceKind::RcInput, 0, channel);
    }

    static SourceEndpoint component(ComponentId id, uint8_t output_port)
    {
        return SourceEndpoint(SourceKind::ComponentOutput, uint8_t(id), output_port);
    }
};

enum class ConsumerKind : uint8_t {
    ComponentInput = 0,
    PwmOutput = 1,
    FunctionInput = 2,
};

struct ConsumerEndpoint {
    ConsumerKind kind = ConsumerKind::PwmOutput;
    uint8_t instance = 0; // ComponentId when kind == ComponentInput
    uint8_t port = 0;     // component input port or PWM channel

    ConsumerEndpoint() = default;
    constexpr ConsumerEndpoint(ConsumerKind endpoint_kind, uint8_t endpoint_instance, uint8_t endpoint_port) :
        kind(endpoint_kind),
        instance(endpoint_instance),
        port(endpoint_port)
    {}

    static ConsumerEndpoint component(ComponentId id, uint8_t input_port)
    {
        return ConsumerEndpoint(ConsumerKind::ComponentInput, uint8_t(id), input_port);
    }

    static ConsumerEndpoint pwm(uint8_t channel)
    {
        return ConsumerEndpoint(ConsumerKind::PwmOutput, 0, channel);
    }

    static ConsumerEndpoint function(uint8_t slot)
    {
        return ConsumerEndpoint(ConsumerKind::FunctionInput, 0, slot);
    }
};

// All routed values stay in microseconds. A source may fan out to any number
// of consumers. A consumer may have at most one enabled producer.
struct Route {
    SourceEndpoint source {};
    ConsumerEndpoint consumer {};

    uint16_t input_min_us = 1000;
    uint16_t input_max_us = 2000;
    uint16_t output_min_us = 1000;
    uint16_t output_max_us = 2000;

    bool reversed = false;
    bool enabled = false;
};

enum class RcLossPolicy : uint8_t {
    HoldLast = 0,
    DisableRoutedOutputs = 1,
};

class Routing {
public:
    static constexpr uint8_t capacity = max_routes;

    bool set_route(uint8_t index, const Route& route);
    void clear_route(uint8_t index, PwmOut* pwm = nullptr);
    void clear_all(PwmOut* pwm = nullptr);

    const Route* route(uint8_t index) const;

    void set_rc_loss_policy(RcLossPolicy policy) { _rc_loss_policy = policy; }
    RcLossPolicy rc_loss_policy() const { return _rc_loss_policy; }

    // Convenience for bench bring-up. Configures RCn -> PWMn.
    uint8_t configure_passthrough(uint8_t count,
                                  uint16_t min_us = 1000,
                                  uint16_t max_us = 2000);

    void update(const RcInput& rc, Logic& logic, PwmOut& pwm);

    // Invalid/duplicate parameter routes must inhibit the entire function
    // consumer, not leave the first accepted producer active.
    void set_function_errors(uint16_t mask) { _function_error_mask = mask; }

private:
    static uint16_t map_us(uint16_t value,
                           uint16_t in_min,
                           uint16_t in_max,
                           uint16_t out_min,
                           uint16_t out_max,
                           bool reversed);

    bool validate_endpoint_pair(const Route& route) const;
    bool consumer_claimed_by_other(uint8_t index, const ConsumerEndpoint& consumer) const;
    bool same_consumer(const ConsumerEndpoint& a, const ConsumerEndpoint& b) const;
    bool rebuild_execution_order();

    bool sample_source(const SourceEndpoint& source,
                       const RcInput& rc,
                       const Logic& logic,
                       uint16_t& value_us, bool* source_valid = nullptr) const;
    void fill_inputs(ComponentId component, const RcInput& rc, Logic& logic) const;

    void disable_mask(uint32_t mask, PwmOut& pwm) const;
    void prepare_functions(Logic& logic, const Route* (&routes)[Functions::count]);
    void sample_functions(const RcInput& rc, Logic& logic,
                          const Route* const (&routes)[Functions::count]) const;

    Route _routes[capacity] {};
    RcLossPolicy _rc_loss_policy = RcLossPolicy::HoldLast;
    uint32_t _driven_output_mask = 0;
    uint32_t _revision = 0;
    bool _heading_routes_valid = true;
    uint16_t _function_error_mask = 0;
    Route _function_routes[Functions::count] {};
    uint32_t _function_revisions[Functions::count] {};

    ComponentId _execution_order[Logic::component_count] {
        ComponentId::VSP1,
        ComponentId::VSP2,
        ComponentId::ThrusterBow,
        ComponentId::ThrusterStern,
    };
    ComponentId _paired_execution_order[Logic::component_count] {
        ComponentId::VSP1, ComponentId::VSP2,
        ComponentId::ThrusterBow, ComponentId::ThrusterStern,
    };
};

} // namespace Vektor2
