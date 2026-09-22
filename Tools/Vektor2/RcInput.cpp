#include "RcInput.h"

#include <AP_RCProtocol/AP_RCProtocol.h>

extern const AP_HAL::HAL &hal;

namespace Vektor2 {

void RcInput::init()
{
#if AP_RCPROTOCOL_ENABLED
    // Initialise the protocol decoder before AP_SerialManager scans ports.
    // SERIALx_PROTOCOL=23 will then be attached to this decoder by
    // AP_SerialManager::init().
    AP::RC().init();
#endif
}

void RcInput::update()
{
    _fresh = false;

#if AP_RCPROTOCOL_ENABLED
    AP::RC().update();
    _failsafe = AP::RC().failsafe_active();

    const auto detected = AP::RC().protocol_detected();
    const uint8_t detected_code = uint8_t(detected);
    if (detected_code != _protocol_code) {
        _protocol_code = detected_code;
        if (AP::RC().detected_protocol_name() != nullptr) {
            _protocol_changed = true;
        }
    }
#else
    _failsafe = false;
#endif

    if (hal.rcin == nullptr || !hal.rcin->new_input()) {
        return;
    }

    const uint8_t available = hal.rcin->num_channels();
    _channel_count = available > max_channels ? max_channels : available;
    if (_channel_count != 0) {
        hal.rcin->read(_channels, _channel_count);
    }
    _last_update_us = AP_HAL::micros64();
    _fresh = true;
}

const char* RcInput::protocol_name() const
{
#if AP_RCPROTOCOL_ENABLED
    return AP::RC().detected_protocol_name();
#else
    return nullptr;
#endif
}

} // namespace Vektor2
