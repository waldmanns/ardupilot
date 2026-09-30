#include "LibVSP.h"
#include "VSP1.h"

namespace Vektor2::Components {

static uint16_t pulse_width(int32_t value)
{
    if (value < 0) {
        return 0;
    }
    if (value > UINT16_MAX) {
        return UINT16_MAX;
    }
    return uint16_t(value);
}

void vsp1_loop(const uint16_t* inputs,
               uint16_t* outputs,
               const VSP1Params& params)
{
    const Limiter limits = setupLimiter(params.lim, params.x_c, params.y_c);
    const Coordinates input { inputs[0], inputs[1] };
    const Coordinates result = mapCoordinates(input, limits, params.dir);

    outputs[0] = pulse_width(result.x_coordinate);
    outputs[1] = pulse_width(result.y_coordinate);
    outputs[2] = inputs[2]; // RPM_OUT <- RPM_IN
}

} // namespace Vektor2::Components
