#pragma once

#include "Functions.h"
#include "NavigationSupervisor.h"
#include "NavigationDP.h"
#include "NavigationHeading.h"
#include <stdint.h>

namespace Vektor2 {

class Logic;

// Owns the rate limit for component telemetry. Component files choose which
// named values to publish; MAVLink transport remains outside their loops.
class NamedFloatTelemetry {
public:
    void send_float(const char* name, float value) const;
    void send_string(const char* name, const char* value) const;

protected:
    bool ready(uint32_t now_ms, int16_t rate_hz);
    void reset() { _sent_once = false; }

private:
    uint32_t _last_send_ms = 0;
    bool _sent_once = false;
};

class VspTelemetry : public NamedFloatTelemetry {
public:
    void update(uint32_t now_ms, int16_t rate_hz, const Logic& logic);
};

class ThrusterTelemetry : public NamedFloatTelemetry {
public:
    void update(uint32_t now_ms, int16_t rate_hz, const Logic& logic);
};

class NavigationTelemetry : public NamedFloatTelemetry {
public:
    void update(uint32_t now_ms, const NavigationSupervisor::Status& status);
private:
    uint32_t _last_sent_ms = 0;
    bool _sent = false;
    NavigationSupervisor::Status _previous;
};

// Independent DP/heading diagnostic stream, capped by DP_TEL_HZ (0..50).
// NAV_STATE / NAV_RBITS retain their own state-event cadence.
class DPTelemetry : public NamedFloatTelemetry {
public:
    void update(uint32_t now_ms, int16_t rate_hz, const NavigationDP::Output& dp,
                const NavigationHeading::Output& heading);
};

class FunctionTelemetry : public NamedFloatTelemetry {
public:
    void update(uint32_t now_ms, const Logic& logic);

private:
    bool _was_configured = false;
    uint32_t _snapshot_ms = 0;
    Functions::Status _status[Functions::count] {};
    uint32_t _pending[8] {};
    bool _roll_enabled = false;
};

} // namespace Vektor2
