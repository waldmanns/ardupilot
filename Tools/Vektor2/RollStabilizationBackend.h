#pragma once

#include "RollStabilizationParams.h"

namespace Vektor2 {
class VspTelemetry;

// Small ArduPilot adapter: sensor access and telemetry live here, not in the
// independently testable controller or in the two VSP algorithms.
class RollStabilizationBackend {
public:
    void update(const RollStabilizationParameters& params,
                const RollDriveParameters& first, const RollDriveParameters& second);
    RollStabilization* controller() { return _controller.enabled() ? &_controller : nullptr; }
    void send_telemetry(const VspTelemetry& telemetry) const;

private:
    RollStabilization _controller;
};
} // namespace Vektor2
