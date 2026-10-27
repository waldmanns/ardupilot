#include "ThrusterControl.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace Vektor2 {
namespace {
bool valid(const ThrusterSettings& s)
{
    return (s.enable == 0 || s.enable == 1) && s.mid >= 1000 && s.mid <= 2000 &&
        s.deadband >= 0 && s.deadband <= 250 && isfinite(s.maximum) && s.maximum >= 0 && s.maximum <= 1 &&
        isfinite(s.slew) && s.slew > 0 && s.slew <= 100 &&
        (s.heading_enable == 0 || s.heading_enable == 1) &&
        (s.heading_sign == -1 || s.heading_sign == 1) &&
        isfinite(s.heading_maximum) && s.heading_maximum >= 0 && s.heading_maximum <= s.maximum;
}
bool different(float a, float b) { return memcmp(&a, &b, sizeof(a)) != 0; }
bool changed(const ThrusterSettings& a, const ThrusterSettings& b)
{
    return a.enable != b.enable || a.mid != b.mid || a.deadband != b.deadband ||
        different(a.maximum, b.maximum) || different(a.slew, b.slew) || a.heading_enable != b.heading_enable ||
        a.heading_sign != b.heading_sign || different(a.heading_maximum, b.heading_maximum);
}
float span(const ThrusterSettings& s) { return fminf(s.mid - 1000, 2000 - s.mid); }
}

void ThrusterControl::update(uint32_t now_ms, const ThrusterSettings& settings, bool permitted,
                             bool routed, bool source_valid, uint16_t input)
{
    const uint32_t elapsed = now_ms - _last_ms;
    const bool gap = _have_time && elapsed > 100;
    const bool enabled = settings.enable == 1 && permitted && valid(settings);
    const bool reset = !_have_time || gap || changed(settings, _settings) || enabled != _enabled || routed != _routed;
    if (reset) {
        _armed = false;
        _manual = 0;
        _revision++;
    }
    const float dt = !reset ? elapsed * 0.001f : 0;
    _have_time = true;
    _last_ms = now_ms;
    _settings = settings;
    _enabled = enabled;
    _routed = routed;
    // Invalid neutral cannot be sent as an arbitrary out-of-range command.
    _output = settings.mid >= 1000 && settings.mid <= 2000 ? settings.mid : 1500;
    _input_valid = routed && source_valid && input >= 1000 && input <= 2000 && !gap;
    _neutral = _input_valid && abs(int(input) - settings.mid) <= settings.deadband;
    if (!valid(settings)) {
        _state = State::Configuration;
        _armed = false;
        _enabled = false;
    } else if (!enabled) {
        _state = State::Disabled;
        _armed = false;
    } else if (!_input_valid) {
        _state = State::InputLost;
        _armed = false;
    } else if (!_armed && !_neutral) {
        _state = State::NeutralWait;
    } else {
        _armed = true;
        _state = State::Manual;
        const float travel = span(settings);
        const float target = _neutral ? 0 : fmaxf(-travel * settings.maximum,
                                                  fminf(travel * settings.maximum, int(input) - settings.mid));
        const float step = settings.slew * travel * dt;
        _manual += fmaxf(-step, fminf(step, target - _manual));
        // Neutral is also the baseline for heading capture; no manual tail remains.
        if (_neutral) {
            _manual = 0;
        }
        _output = uint16_t(settings.mid + lroundf(_manual));
    }
    if (!_armed) {
        _manual = 0;
    }
    const bool available = heading_available();
    if (available != _was_available) {
        _revision++;
        _was_available = available;
    }
}

void ThrusterControl::apply_heading(float command, bool holding)
{
    if (!heading_available() || !holding || !isfinite(command)) {
        return;
    }
    const float correction = fmaxf(-1, fminf(1, command)) * _settings.heading_sign *
        _settings.heading_maximum * span(_settings);
    _output = uint16_t(_settings.mid + lroundf(correction));
    _state = State::Heading;
}
}
