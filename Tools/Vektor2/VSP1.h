#pragma once

#include <stdint.h>

namespace Vektor2 { class VspTelemetry; class RollStabilization; }

namespace Vektor2::Components {

struct VSP1Params {
    int16_t lim;
    int16_t x_c;
    int16_t y_c;
    uint16_t dir;
    int16_t thr_ang;
};

// VSP1 logic only. Inputs and outputs are pulse-width values in microseconds.
// Persistent storage/routing/MAVLink remain outside this file.
void vsp1_loop(const uint16_t* inputs,
               uint16_t* outputs,
               const VSP1Params& params,
               RollStabilization* stabilization = nullptr);

void vsp1_emit_telemetry(const uint16_t* outputs, const VspTelemetry& telemetry);

} // namespace Vektor2::Components
