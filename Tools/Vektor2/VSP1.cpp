#include "VSP1.h"

namespace Vektor2::Components {

void vsp1_loop(const uint16_t* inputs,
               uint16_t* outputs,
               const VSP1Params& params)
{
    // Starter behavior remains transparent pass-through. The typed parameter
    // block is already available here for the actual VSP1 algorithm.
    (void)params;
    outputs[0] = inputs[0];
    outputs[1] = inputs[1];
    outputs[2] = inputs[2]; // RPM_OUT <- RPM_IN
}

} // namespace Vektor2::Components
