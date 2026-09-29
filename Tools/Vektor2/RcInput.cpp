#include "RcInput.h"

#include <AP_RCProtocol/AP_RCProtocol.h>

extern const AP_HAL::HAL &hal;

namespace Vektor2 {

void RcInput::update()
{
    _fresh = false;

#if AP_RCPROTOCOL_ENABLED
    // ChibiOS polls the serial RC decoder in its dedicated RC input thread.
    // Calling update() here would read the same UART from a second thread and
    // can split receiver frames before either reader has a complete packet.
    _failsafe = AP::RC().failsafe_active();

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
