#pragma once

#include <stdint.h>

namespace Vektor2 {

class VspRpmGovernor {
public:
    enum Status : uint16_t {
        Enabled = 1U << 0,
        ControlValid = 1U << 1,
        Requested = 1U << 2,
        FeedbackValid = 1U << 3,
        ThrottleValid = 1U << 4,
        Active = 1U << 5,
        InDeadband = 1U << 6,
        AtThrottleLimit = 1U << 7,
        ConfigValid = 1U << 8,
        Starting = 1U << 9,
        Inhibited = 1U << 10,
    };

    struct Settings {
        bool enabled;
        float low_rpm;
        float high_rpm;
        float deadband_rpm;
        float fast_us_per_s;
        float slow_us_per_s;
        float near_rpm;
        int16_t start_us;
        float start_timeout_s;
    };

    struct Feedback {
        float rpm;
        bool available;
    };

    uint16_t update(uint32_t now_ms, const Settings& settings, uint16_t input_us,
                    bool input_routed, const Feedback& shaft);
    uint16_t status() const { return _status; }

private:
    enum class State : uint8_t { Stopped, Acquiring, Regulating, WaitForOff };

    float _throttle_us = 0;
    uint32_t _last_ms = 0;
    uint32_t _start_ms = 0;
    uint16_t _status = 0;
    State _state = State::Stopped;
};

} // namespace Vektor2
