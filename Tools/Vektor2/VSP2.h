#pragma once

#include <stdint.h>

namespace Vektor2::Components {

struct VSP2Params {
    int16_t lim;
    int16_t x_c;
    int16_t y_c;
};

// VSP2 logic only. Inputs and outputs are pulse-width values in microseconds.
// Persistent storage/routing/MAVLink remain outside this file.
void vsp2_loop(const uint16_t* inputs,
               uint16_t* outputs,
               const VSP2Params& params);

} // namespace Vektor2::Components
