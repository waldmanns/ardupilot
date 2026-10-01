#include "ThrusterBow.h"

void thruster_bow_loop(const uint16_t* inputs,
                       uint16_t* outputs,
                       const ThrusterBowParams* params)
{
    // The parameters are reserved for the bow thruster's control logic.
    (void)params;
    outputs[0] = inputs[0];
}
