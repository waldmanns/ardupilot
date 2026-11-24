#include "Functions.h"

namespace Vektor2 {

static bool changed(const FunctionSettings& a, const FunctionSettings& b)
{
    return a.enable != b.enable || a.type != b.type || a.mode != b.mode ||
           a.low != b.low || a.high != b.high;
}

static bool valid(const FunctionSettings& s)
{
    const bool nav_mode = s.type == int8_t(Functions::Type::NavMode);
    // Do not silently map old, stored type 5/6 parameter values to NavMode.
    // They must raise Configuration and fail to Free until explicitly changed.
    return (s.enable == 0 || s.enable == 1) && s.type >= 0 &&
           (s.type <= int8_t(Functions::Type::Stern) || nav_mode) &&
           (s.mode == int8_t(Functions::Mode::Level) ||
            (!nav_mode && s.mode == int8_t(Functions::Mode::Toggle))) &&
           s.low >= Functions::minimum_us && s.low <= s.high &&
           s.high <= Functions::maximum_us &&
           // NavMode must have three non-empty stable regions plus two
           // +/-50us no-change bands around LOW and HIGH. No 2-way mode.
           (!nav_mode ||
            (s.low > Functions::minimum_us + Functions::nav_transition_guard_us &&
             s.high <= Functions::maximum_us - Functions::nav_transition_guard_us &&
             s.high - s.low > 2 * Functions::nav_transition_guard_us));
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
        const bool slot_changed = changed(slot.settings, settings[n]) ||
                                  slot.binding.revision != binding.revision ||
                                  slot.binding.routed != binding.routed ||
                                  slot.binding.error != binding.error;
        if (slot_changed || _timing_invalid) {
            slot.status = Status{};
            // Never carry a navigation mode through a switch/route edit or
            // an interrupted control-loop interval.
            if (slot.settings.type == int8_t(Type::NavMode) ||
                settings[n].type == int8_t(Type::NavMode) || _timing_invalid) {
                _nav_selected_mode = 0;
            }
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
            // NavMode is finalized in navigation_selection(), which applies
            // the three-way guard bands and updates Requested/Matched to
            // reflect the ACTUAL held mode, not the pulse's raw threshold.
            // Ordinary VFN actions keep their inclusive LOW..HIGH interval.
            const bool nav_mode = settings.type == int8_t(Type::NavMode);
            if (!nav_mode && input.pulse_us >= settings.low &&
                input.pulse_us <= settings.high) {
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

Functions::NavigationSelection Functions::navigation_selection(bool rc_valid)
{
    NavigationSelection selection;
    int8_t next_mode = _nav_selected_mode;
    for (const auto& slot : _slots) {
        const auto type = slot.settings.type;
        const bool nav_mode = type == int8_t(Type::NavMode);
        const bool retired = type == int8_t(Type::RetiredNavHeading) ||
                             type == int8_t(Type::RetiredNavDP);
        if (!nav_mode && !retired) {
            continue;
        }
        // Exactly one NavMode action owns one three-position RC switch.
        // Retired type 5/6 or a second selector must fail to Free.
        if (selection.assigned) {
            selection.conflict = true;
        }
        selection.assigned = true;
        if (retired || slot.settings.enable != 1 || slot.status.errors != 0 ||
            (slot.status.flags & (Enabled | Routed | Valid)) !=
            (Enabled | Routed | Valid)) {
            selection.valid = false;
            continue;
        }
        const int16_t pulse = slot.status.pulse_us;
        const int16_t low = slot.settings.low;
        const int16_t high = slot.settings.high;
        // Three stable intervals, two +/-50us gaps where NO MODE CHANGE is
        // made while the operator moves the switch. Do not treat an in-gap
        // reading as an extra RC position or a new DP target capture.
        if (pulse < low - nav_transition_guard_us) {
            next_mode = 0; // switch LOW: Free
            selection.free_selected = true;
        } else if (pulse >= low + nav_transition_guard_us &&
                   pulse < high - nav_transition_guard_us) {
            next_mode = 1; // switch MIDDLE: Heading Hold
        } else if (pulse >= high + nav_transition_guard_us) {
            next_mode = 2; // switch HIGH: Full DP
        }
    }
    if (!selection.assigned || !rc_valid || !selection.valid || selection.conflict) {
        // A guard band is NOT a failsafe. Loss of RC or invalid selection
        // always clears the held mode immediately to Free.
        _nav_selected_mode = 0;
        selection.free_selected = false;
    } else {
        _nav_selected_mode = next_mode;
    }
    selection.mode = _nav_selected_mode;

    // The VFN status bit means the HELD request is non-Free; diagnostics
    // must not claim Heading/DP before the pulse leaves the no-change band.
    for (auto& slot : _slots) {
        if (slot.settings.type != int8_t(Type::NavMode)) {
            continue;
        }
        slot.status.flags &= ~(Matched | Requested);
        if (selection.valid && !selection.conflict && rc_valid &&
            _nav_selected_mode != 0) {
            slot.status.flags |= Matched | Requested;
        }
    }
    return selection;
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
