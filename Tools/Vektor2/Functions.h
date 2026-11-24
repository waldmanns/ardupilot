#pragma once

#include <stdint.h>

namespace Vektor2 {

struct FunctionSettings {
    int8_t enable = 0;
    int8_t type = 0;
    int8_t mode = 0;
    int16_t low = 1700;
    int16_t high = 2000;
};

struct FunctionBinding {
    uint32_t revision = 0;
    bool routed = false;
    bool error = false;
};

struct FunctionInput {
    uint16_t pulse_us = 0;
    bool valid = false;
};

// Pure, fixed-size trigger evaluator. prepare() runs before the components;
// sample() runs after them. Its requests are used by the NEXT control cycle.
class Functions {
public:
    static constexpr uint8_t count = 10;
    static constexpr uint16_t minimum_us = 800;
    static constexpr uint16_t maximum_us = 2200;
    // NavMode-only transition band: +/- 50us at each configured boundary.
    // Keep the current mode in this band instead of changing on PWM jitter.
    static constexpr uint16_t nav_transition_guard_us = 50;
    // 5 and 6 were M3.8 navigation actions. Retire, never reinterpret:
    // stored slots with those types must fail off until explicitly migrated.
    enum class Type : uint8_t {
        None = 0, Roll = 1, Heading = 2, Bow = 3, Stern = 4,
        RetiredNavHeading = 5, RetiredNavDP = 6, NavMode = 7
    };
    enum class Mode : uint8_t { Level = 0, Toggle = 1 };
    enum Flag : uint8_t {
        Enabled = 1U << 0, Routed = 1U << 1, Valid = 1U << 2,
        Matched = 1U << 3, Requested = 1U << 4, Armed = 1U << 5,
    };
    enum Error : uint8_t {
        Configuration = 1U << 0, Unrouted = 1U << 1, Routing = 1U << 2,
        Input = 1U << 3, Timing = 1U << 4,
    };
    struct Status {
        uint16_t pulse_us = 0;
        uint8_t flags = 0;
        uint8_t errors = 0;
    };

    void prepare(uint32_t now_ms, const FunctionSettings (&settings)[count],
                 const FunctionBinding (&bindings)[count]);
    void sample(const FunctionInput (&inputs)[count]);
    // An assigned type remains managed even if its slots are disabled/broken.
    // Multiple slots of a type combine with OR. Unassigned types pass through.
    bool permits(Type type) const;
    // NavMode (TYPE=7) REQUIRES A THREE-POSITION RC SWITCH (not a two-way
    // Free/DP switch): low=Free, middle=Heading Hold, high=Full DP.
    // LOW and HIGH are the two PWM transition centers, NOT action ranges.
    // With the 50us guard on each side, e.g. LOW=1300/HIGH=1700:
    //   800..1249  Free        1250..1349  no change (switch transition)
    //   1350..1649 Heading     1650..1749  no change (switch transition)
    //   1750..2200 DP
    // "No change" keeps the last stable mode, initially Free. Missing/invalid
    // RC, faulty routing, duplicate selectors or config changes reset to Free.
    // A PWM signal cannot prove the physical switch has three positions:
    // check the three stable RC pulse values during board commissioning.
    struct NavigationSelection {
        int8_t mode = 0; // 0 Free, 1 Heading, 2 DP
        bool assigned = false;
        bool valid = true;
        bool conflict = false;
        bool free_selected = false; // confirmed switch position outside the guard band
    };
    NavigationSelection navigation_selection(bool rc_valid);
    bool configured() const;
    const Status& status(uint8_t slot) const { return _slots[slot].status; }

private:
    struct Slot {
        FunctionSettings settings;
        FunctionBinding binding;
        Status status;
    };
    Slot _slots[count] {};
    uint32_t _last_ms = 0;
    bool _have_time = false;
    bool _timing_invalid = false;
    int8_t _nav_selected_mode = 0; // retained only across valid switch transitions
};

} // namespace Vektor2
