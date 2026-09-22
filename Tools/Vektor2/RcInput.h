#pragma once

#include "Config.h"

#include <AP_HAL/AP_HAL.h>
#include <stdint.h>

namespace Vektor2 {

class RcInput {
public:
    static constexpr uint8_t max_channels = 18;

    void init();
    void update();

    bool fresh() const { return _fresh; }
    bool failsafe() const { return _failsafe; }
    bool valid(uint16_t timeout_ms = rc_input_timeout_ms) const
    {
        if (_failsafe || _last_update_us == 0) {
            return false;
        }
        const uint64_t age_us = AP_HAL::micros64() - _last_update_us;
        return age_us <= uint64_t(timeout_ms) * 1000ULL;
    }
    uint8_t channel_count() const { return _channel_count; }
    uint16_t channel_us(uint8_t channel) const
    {
        return channel < _channel_count ? _channels[channel] : 0;
    }
    uint64_t last_update_us() const { return _last_update_us; }

    const char* protocol_name() const;
    bool take_protocol_changed()
    {
        const bool changed = _protocol_changed;
        _protocol_changed = false;
        return changed;
    }

private:
    uint16_t _channels[max_channels] {};
    uint8_t _channel_count = 0;
    uint64_t _last_update_us = 0;
    bool _fresh = false;
    bool _failsafe = false;
    uint8_t _protocol_code = 0xFF;
    bool _protocol_changed = false;
};

} // namespace Vektor2
