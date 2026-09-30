#pragma once

#include <stdint.h>

namespace Vektor2 {

class Logic;

// Owns the rate limit for component telemetry. Component files choose which
// named values to publish; MAVLink transport remains outside their loops.
class VspTelemetry {
public:
    void update(uint32_t now_ms, int16_t rate_hz, const Logic& logic);
    void send_float(const char* name, float value) const;

private:
    uint32_t _last_send_ms = 0;
    bool _sent_once = false;
};

} // namespace Vektor2
