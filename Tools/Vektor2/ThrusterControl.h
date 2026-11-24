#pragma once
#include <stdint.h>

namespace Vektor2 {
struct ThrusterSettings {
    int8_t enable = 0;
    int16_t mid = 1500;
    int16_t deadband = 30;
    float maximum = 1;
    float slew = 2; // fraction of symmetric travel per second
    int8_t heading_enable = 0;
    int8_t heading_sign = 1;
    float heading_maximum = 0.3f;
    int16_t distance_cm = 0;
};

// Independent actuator: no VSP, AHRS or parameter-library dependencies.
class ThrusterControl {
public:
    enum class State : uint8_t { Disabled, NeutralWait, Manual, Heading, InputLost, Configuration };
    enum Reason : uint16_t {
        MasterDisabled = 1U << 0,
        FunctionBlocked = 1U << 1,
        HeadingDisabled = 1U << 2,
        InputRouteMissing = 1U << 3,
        InputInvalid = 1U << 4,
        WaitNeutral = 1U << 5,
        ManualOverride = 1U << 6,
        ConfigInvalid = 1U << 7,
        OutputRouteMissing = 1U << 8,
        OutputRejected = 1U << 9,
        GeometryInvalid = 1U << 10,
    };
    void update(uint32_t now_ms, const ThrusterSettings& settings, bool permitted,
                bool routed, bool valid, uint16_t input);
    void reset() { _have_time = _armed = _input_valid = _enabled = _heading_handover = false; _manual = 0; _revision++; }
    void apply_heading(float command, bool holding);
    void release_heading();
    void follow_heading_output() { _manual = int(_output) - _settings.mid; _heading_handover = true; }
    float heading_equivalent() const;
    void force_neutral() { _output = _settings.mid >= 1000 && _settings.mid <= 2000 ? _settings.mid : 1500; _manual=0; _armed=_heading_handover=false; _state=State::NeutralWait; }
    uint16_t output() const { return _output; }
    State state() const { return _state; }
    bool enabled() const { return _enabled; }
    bool heading_selected() const { return _settings.heading_enable == 1; }
    float lever_arm_m() const { return _settings.distance_cm * 0.01f; }
    uint16_t reasons() const { return _reasons; }
    void set_output_reasons(uint16_t reasons) { _reasons |= reasons; }
    bool manual_override() const { return _enabled && _input_valid && !_neutral; }
    bool heading_available() const { return _armed && _input_valid && _neutral && _settings.heading_enable == 1 && _settings.heading_maximum > 0 && _settings.mid > 1000 && _settings.mid < 2000; }
    float heading_slew() const { return _settings.slew / _settings.heading_maximum; }
    uint32_t revision() const { return _revision; }
private:
    ThrusterSettings _settings;
    State _state = State::Disabled;
    uint32_t _last_ms = 0;
    uint32_t _revision = 0;
    bool _have_time = false;
    bool _enabled = false;
    bool _armed = false;
    bool _input_valid = false;
    bool _neutral = false;
    bool _routed = false;
    bool _was_available = false;
    bool _heading_handover = false;
    float _manual = 0;
    uint16_t _output = 1500;
    uint16_t _reasons = 0;
};
}
