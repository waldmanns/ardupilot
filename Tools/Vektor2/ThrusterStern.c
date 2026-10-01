#include "ThrusterStern.h"

void thruster_stern_loop(const uint16_t* inputs,
                       uint16_t* outputs,
                       const ThrusterSternParams* params)
{
    // The parameters are reserved for the stern thruster's control logic.
    (void)params;
    outputs[0] = inputs[0];
}
