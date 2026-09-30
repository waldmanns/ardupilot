#include "VSP2.h"

namespace Vektor2::Components {

void vsp2_loop(const uint16_t* inputs,
               uint16_t* outputs,
               const VSP2Params& params)
{
    // Starter behavior remains transparent pass-through. The typed parameter
    // block is already available here for the actual VSP2 algorithm.
    (void)params;
    outputs[0] = inputs[0];
    outputs[1] = inputs[1];
    outputs[2] = inputs[2]; // RPM_OUT <- RPM_IN
}

} // namespace Vektor2::Components
