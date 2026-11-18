#pragma once

#include <stdint.h>

namespace Vektor2 {

// Pure heading controller. Actuator readiness and limits are supplied by its owner.
struct HeadingSettings {
    bool enabled = false;
    int8_t vsp_enabled = 1;
    int16_t stick_deadband = 30; // input us either side of 1500, X AND Y
    int16_t idle_ms = 1500;
    float p = 0;
    float i = 0;
    float d = 0;
    float imax = 0.1f;          // integral contribution, normalized command units
    float heading_deadband = 1; // degrees; does not suppress rate damping
    float maximum = 0.2f;       // normalized command units
    float slew = 0.5f;          // normalized command units / second
    float filter_hz = 2;
};

struct HeadingActuation {
    bool available = false;
    bool manual = false;
    bool configuration_valid = true;
    float slew = 100;          // common normalized command units/second
    float maximum = 1;
    uint32_t revision = 0;
};

struct HeadingSample {
    float heading_deg = 0;
    float rate_dps = 0;         // earth-frame vertical angular rate
    bool compass_valid = false;
    bool attitude_valid = false;
    bool control_valid = false;
    bool routes_valid = true;
    bool yaw_reset = false;
    bool routes_changed = false;
};

class HeadingHold {
public:
    enum class State : uint8_t { Disabled, Inhibited, Manual, Waiting, Hold };
    enum Inhibit : uint16_t {
        Compass = 1U << 0,
        Attitude = 1U << 1,
        Input = 1U << 2,
        Configuration = 1U << 3,
        Timing = 1U << 4,
        Routing = 1U << 5,
        YawReset = 1U << 6,
        Stick = 1U << 7,
        Delay = 1U << 8,
        Travel = 1U << 9,
        Slew = 1U << 10,
    };

    void update(uint32_t now_ms, const HeadingSettings& settings,
                const HeadingSample& sample, const HeadingActuation& actuation);
    bool enabled() const { return _settings.enabled; }
    State state() const { return _state; }
    uint16_t inhibited() const { return _inhibited; }
    const char* inhibit_reason() const;
    float heading() const { return _heading; }
    float target() const { return _target; }
    float error() const { return _error; }
    float rate() const { return _rate; }
    float demand() const { return _demand; }
    float command() const { return _command; }
    float integral() const { return _integral; }
    float travel_scale() const { return _travel_scale; }
    uint16_t wait_ms() const { return _wait_ms; }

private:
    static bool valid(const HeadingSettings& settings);
    bool changed(const HeadingSettings& settings) const;
    void release();
    float constrain(float request, float dt);

    HeadingSettings _settings;
    HeadingActuation _actuation;
    State _state = State::Disabled;
    uint16_t _inhibited = 0;
    uint16_t _wait_ms = 0;
    uint32_t _last_ms = 0;
    uint32_t _neutral_ms = 0;
    bool _filter_ready = false;
    float _heading = 0;
    float _target = 0;
    float _error = 0;
    float _rate = 0;
    float _demand = 0;
    float _integral = 0;
    float _command = 0;        // common signed normalized command before PWM quantization
    float _travel_scale = 1;
};
} // namespace Vektor2
