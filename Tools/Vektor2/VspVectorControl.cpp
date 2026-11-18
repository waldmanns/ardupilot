#include "VspVectorControl.h"

#include <math.h>
#include <stdlib.h>

namespace Vektor2 {
namespace {
bool same(const VspVectorControl::Settings& a, const VspVectorControl::Settings& b)
{
    return a.mode == b.mode && a.deadband_us == b.deadband_us &&
        isfinite(a.slew) && isfinite(b.slew) && fabsf(a.slew - b.slew) < 1.0e-6f &&
        a.mapper.rotation == b.mapper.rotation &&
        isfinite(a.mapper.installation_deg) && isfinite(b.mapper.installation_deg) &&
        fabsf(a.mapper.installation_deg - b.mapper.installation_deg) < 1.0e-6f &&
        a.mapper.x_center == b.mapper.x_center && a.mapper.y_center == b.mapper.y_center &&
        a.mapper.radius == b.mapper.radius;
}
uint16_t neutral(int16_t center)
{
    return center >= 1000 && center <= 2000 ? uint16_t(center) : 1500;
}
float axis(uint16_t pulse, int16_t deadband)
{
    const int offset = int(pulse) - 1500;
    return copysignf(fmaxf(0, abs(offset) - deadband) / (500 - deadband), offset);
}
}

bool VspVectorControl::update(uint32_t now_ms, uint32_t revision, const Settings& s,
                             uint16_t forward_us, uint16_t starboard_us, bool inputs_valid,
                             uint16_t& x_us, uint16_t& y_us)
{
    if (!prepare(now_ms, revision, s, forward_us, starboard_us, inputs_valid)) {
        return false;
    }
    apply(x_us, y_us);
    return true;
}

bool VspVectorControl::prepare(uint32_t now_ms, uint32_t revision, const Settings& s,
                              uint16_t forward_us, uint16_t starboard_us, bool inputs_valid)
{
    const uint32_t elapsed = now_ms - _last_ms;
    const bool changed = !_initialized || !same(s, _settings) || revision != _revision;
    _initialized = true;
    _settings = s;
    _revision = revision;
    _last_ms = now_ms;
    _dt = elapsed <= 100 ? elapsed * 0.001f : 0;
    _neutral_cycle = true;
    _centered = false;
    if (s.mode == 0 && _state == State::Legacy) {
        return false;
    }
    if (changed || elapsed > 100) {
        _ready = false;
        _command = {0, 0};
        _heading_last = false;
        _generation++;
    }
    if ((s.mode != 0 && s.mode != 1) || s.deadband_us < 0 || s.deadband_us > 250 ||
        !isfinite(s.slew) || s.slew <= 0 || s.slew > 100 || !VspMapper::valid_config(s.mapper)) {
        _ready = false;
        _command = {0, 0};
        _state = State::Invalid;
        return true;
    }
    if (!inputs_valid || forward_us < 1000 || forward_us > 2000 ||
        starboard_us < 1000 || starboard_us > 2000) {
        if (_ready) {
            _generation++;
        }
        _ready = false;
        _command = {0, 0};
        _state = State::InputLost;
        return true;
    }
    _centered = abs(int(forward_us)-1500) <= s.deadband_us &&
                abs(int(starboard_us)-1500) <= s.deadband_us;
    if (!_ready) {
        _state = State::Waiting;
        if (_centered) {
            _ready = true;
            _state = s.mode == 0 ? State::Legacy : State::Active;
        }
        return true; // one full neutral cycle, even when becoming ready
    }
    _neutral_cycle = false;
    _state = State::Active;
    _requested = {axis(forward_us, s.deadband_us), axis(starboard_us, s.deadband_us)};
    const float magnitude = hypotf(_requested.forward, _requested.starboard);
    if (magnitude > 1) {
        _requested.forward /= magnitude;
        _requested.starboard /= magnitude;
    }
    return true;
}

bool VspVectorControl::apply(uint16_t& x_us, uint16_t& y_us, const VspMapper::Vector* heading)
{
    x_us = neutral(_settings.mapper.x_center);
    y_us = neutral(_settings.mapper.y_center);
    if (_neutral_cycle || _state != State::Active) {
        _heading_last = false;
        return false;
    }
    const bool assisted = heading != nullptr && _centered;
    if (assisted) {
        // Heading has already been pair-limited and slewed. No second filter.
        _command = *heading;
    } else {
        // Release automatic contribution immediately before slewing manual demand.
        if (_heading_last || _centered) {
            _command = {0, 0};
        }
        if (!_centered) {
            const float df = _requested.forward - _command.forward;
            const float ds = _requested.starboard - _command.starboard;
            const float distance = hypotf(df, ds);
            const float step = _settings.slew * _dt;
            const float scale = distance > step ? step / distance : 1;
            _command.forward += df * scale;
            _command.starboard += ds * scale;
        }
    }
    _heading_last = assisted;
    const auto result = VspMapper::map(_settings.mapper, _command);
    if (result.valid) {
        x_us = result.x_us;
        y_us = result.y_us;
        return assisted;
    }
    _ready = false;
    _command = {0, 0};
    _state = State::Invalid;
    return false;
}
} // namespace Vektor2
