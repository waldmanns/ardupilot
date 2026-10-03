#include "RcInput.h"

#include <AP_RCProtocol/AP_RCProtocol.h>

// Use the same parameter table and RC override policy as ArduPilot vehicles.
#define RC_CHANNELS_SUBCLASS Vektor2::RcInput
#define RC_CHANNEL_SUBCLASS RC_Channel
#include <RC_Channel/RC_Channels_VarInfo.h>

extern const AP_HAL::HAL &hal;

namespace Vektor2 {

void RcInput::update()
{
    _fresh = false;

#if AP_RCPROTOCOL_ENABLED
    // ChibiOS polls the serial RC decoder in its dedicated RC input thread.
    // Calling update() here would read the same UART from a second thread and
    // can split receiver frames before either reader has a complete packet.
    _failsafe = AP::RC().failsafe_active() && !option_is_enabled(Option::IGNORE_FAILSAFE);

#else
    _failsafe = false;
#endif

    // RC_Channels owns receiver reads and MAVLink overrides, including the
    // standard RC_OPTIONS and RC_OVERRIDE_TIME policies.
    _fresh = read_input();
    if (_fresh) {
        _last_update_us = AP_HAL::micros64();
    }

    _using_overrides = false;
    _channel_count = 0;
    const bool receiver_valid = !option_is_enabled(Option::IGNORE_RECEIVER) &&
        !_failsafe && _last_update_us != 0 &&
        AP_HAL::micros64() - _last_update_us <= uint64_t(rc_input_timeout_ms) * 1000ULL;
    const uint8_t receiver_count = receiver_valid ? hal.rcin->num_channels() : 0;
    for (uint8_t i = 0; i < max_channels; i++) {
        RC_Channel* c = channel(i);
        const bool overridden = c != nullptr && c->has_override() &&
            !option_is_enabled(Option::IGNORE_OVERRIDES);
        _channels[i] = 0;
        if (c != nullptr && (overridden || i < receiver_count)) {
            // Re-evaluate expiry even when no new packet arrived this cycle.
            c->update();
            _channels[i] = c->get_radio_in();
            _channel_count = i + 1;
        } else if (c == nullptr && i < receiver_count) {
            // Preserve direct receiver channels beyond RC_Channels' capacity.
            _channels[i] = hal.rcin->read(i);
            _channel_count = i + 1;
        }
        _using_overrides |= overridden;
    }
    if (_using_overrides) {
        _failsafe = false;
    }
}

const char* RcInput::protocol_name() const
{
    if (_using_overrides) {
        return "MAVLink";
    }
#if AP_RCPROTOCOL_ENABLED
    return AP::RC().detected_protocol_name();
#else
    return nullptr;
#endif
}

} // namespace Vektor2
