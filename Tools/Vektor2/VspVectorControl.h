#pragma once

#include "VspMapper.h"

namespace Vektor2 {

// Stateful manual-vector owner. No sensor, routing or parameter dependencies.
class VspVectorControl {
public:
    struct Settings {
        int8_t mode;
        int16_t deadband_us;
        float slew;
        VspMapper::Config mapper;
    };
    enum class State : uint8_t { Legacy = 0, Waiting = 1, Active = 2, InputLost = 3, Invalid = 4 };
    // True means the caller must use these rod outputs and skip legacy logic.
    // RPM is deliberately outside this owner.
    bool update(uint32_t now_ms, uint32_t route_revision, const Settings& settings,
                uint16_t forward_us, uint16_t starboard_us, bool inputs_valid,
                uint16_t& x_us, uint16_t& y_us);
    // Prepare exactly once, select the owner, then map exactly once in apply().
    bool prepare(uint32_t now_ms, uint32_t route_revision, const Settings& settings,
                 uint16_t forward_us, uint16_t starboard_us, bool inputs_valid);
    bool apply(uint16_t& x_us, uint16_t& y_us, const VspMapper::Vector* heading = nullptr);
    bool heading_ready() const { return _state == State::Active && !_neutral_cycle; }
    bool centered() const { return _centered; }
    uint32_t revision() const { return _generation; }
    bool blocks_assistance() const { return _state != State::Legacy; }
    State state() const { return _state; }

private:
    Settings _settings {};
    VspMapper::Vector _command {0, 0};
    State _state = State::Legacy;
    uint32_t _last_ms = 0;
    uint32_t _revision = 0;
    bool _initialized = false;
    bool _ready = false;
    bool _centered = false;
    bool _neutral_cycle = true;
    bool _heading_last = false;
    float _dt = 0;
    VspMapper::Vector _requested {0, 0};
    uint32_t _generation = 0;
};

} // namespace Vektor2
