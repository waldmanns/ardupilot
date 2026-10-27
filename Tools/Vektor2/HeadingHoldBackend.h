#pragma once

#include "HeadingHoldParams.h"

namespace Vektor2 {
class HeadingTelemetry;

class HeadingHoldBackend {
public:
    // Disabled calls do no sensor/clock reads and publish no heading telemetry.
    void update(const HeadingHoldParameters& params, const HeadingDrive (&drives)[2],
                bool controls_valid, bool routes_valid, uint32_t route_revision,
                bool permitted = true, const HeadingAuxiliary& auxiliary = HeadingAuxiliary{});
    const HeadingHold& controller() const { return _controller; }
    void send_telemetry(const HeadingTelemetry& telemetry) const;

private:
    HeadingHold _controller;
    uint32_t _yaw_reset_ms = 0;
    uint32_t _route_revision = 0;
};
} // namespace Vektor2
