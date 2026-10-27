#include "Functions.h"

namespace Vektor2 {

static bool changed(const FunctionSettings& a, const FunctionSettings& b)
{
    return a.enable != b.enable || a.type != b.type || a.mode != b.mode ||
           a.low != b.low || a.high != b.high;
}

static bool valid(const FunctionSettings& s)
{
    return (s.enable == 0 || s.enable == 1) && s.type >= 0 &&
           s.type <= int8_t(Functions::Type::Stern) &&
           (s.mode == int8_t(Functions::Mode::Level) || s.mode == int8_t(Functions::Mode::Toggle)) &&
           s.low >= Functions::minimum_us && s.low <= s.high && s.high <= Functions::maximum_us;
}

void Functions::prepare(uint32_t now_ms, const FunctionSettings (&settings)[count],
                        const FunctionBinding (&bindings)[count])
{
    _timing_invalid = _have_time && now_ms - _last_ms > 100;
    _last_ms = now_ms;
    _have_time = true;
    for (uint8_t n = 0; n < count; n++) {
        auto& slot = _slots[n];
        const auto& binding = bindings[n];
        if (changed(slot.settings, settings[n]) || slot.binding.revision != binding.revision ||
            slot.binding.routed != binding.routed || slot.binding.error != binding.error ||
            _timing_invalid) {
            slot.status = Status{};
        }
        slot.settings = settings[n];
        slot.binding = binding;
        auto& status = slot.status;
        status.errors = valid(slot.settings) ? 0 : Configuration;
        status.flags = (status.flags & ~(Enabled | Routed)) |
                       (slot.settings.enable == 1 ? Enabled : 0) |
                       (binding.routed ? Routed : 0);
        if (slot.settings.type == 0 || slot.settings.enable != 1) {
            // Disabled slots cannot retain a toggle latch or arm an edge.
            status.flags &= Enabled | Routed;
            status.pulse_us = 0;
            continue;
        }
        if (!binding.routed) {
            status.errors |= Unrouted;
        }
        if (binding.error) {
            status.errors |= Routing;
        }
        if (_timing_invalid) {
            status.errors |= Timing;
        }
        if (status.errors) {
            status.flags &= Enabled | Routed;
            status.pulse_us = 0;
        }
    }
}

void Functions::sample(const FunctionInput (&inputs)[count])
{
    for (uint8_t n = 0; n < count; n++) {
        auto& slot = _slots[n];
        auto& status = slot.status;
        const auto& settings = slot.settings;
        const auto& input = inputs[n];
        status.pulse_us = input.pulse_us;
        status.flags &= ~(Valid | Matched);
        const bool input_valid = slot.binding.routed && !slot.binding.error && input.valid &&
                                 input.pulse_us >= minimum_us && input.pulse_us <= maximum_us;
        if (input_valid) {
            status.flags |= Valid;
            if (input.pulse_us >= settings.low && input.pulse_us <= settings.high) {
                status.flags |= Matched;
            }
        }
        if (settings.enable != 1 || settings.type == 0 || status.errors) {
            continue;
        }
        if (!input_valid) {
            status.flags &= Enabled | Routed;
            status.errors |= Input;
            continue;
        }
        const bool matched = status.flags & Matched;
        if (settings.mode == int8_t(Mode::Level)) {
            status.flags = (status.flags & ~Requested) | (matched ? Requested : 0);
        } else if (!matched) {
            status.flags |= Armed;
        } else if (status.flags & Armed) {
            status.flags ^= Requested;
            status.flags &= ~Armed;
        }
    }
}

bool Functions::permits(Type type) const
{
    bool assigned = false;
    for (const auto& slot : _slots) {
        if (slot.settings.type != int8_t(type)) {
            continue;
        }
        assigned = true;
        if (slot.status.flags & Requested) {
            return true;
        }
    }
    return !assigned;
}

bool Functions::configured() const
{
    for (const auto& slot : _slots) {
        if (slot.settings.type != 0 || slot.settings.enable != 0) {
            return true;
        }
    }
    return false;
}

} // namespace Vektor2
