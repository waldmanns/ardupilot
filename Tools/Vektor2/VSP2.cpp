#include "LibVSP.h"
#include "Telemetry.h"
#include "VSP2.h"

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

void vsp2_loop(const uint16_t* inputs,
               uint16_t* outputs,
               const VSP2Params& params)
{
    const Limiter limits = setupLimiter(params.lim, params.x_c, params.y_c);
    const Coordinates input { inputs[0], inputs[1] };
    const Coordinates result = mapCoordinates(input, limits, params.dir);

    outputs[0] = pulse_width(result.x_coordinate);
    outputs[1] = pulse_width(result.y_coordinate);
    outputs[2] = inputs[2]; // RPM_OUT <- RPM_IN
}

void vsp2_emit_telemetry(const uint16_t* outputs, const VspTelemetry& telemetry)
{
    telemetry.send_float("VSP2_XA", float(outputs[0]));
    telemetry.send_float("VSP2_YA", float(outputs[1]));
}

} // namespace Vektor2::Components
