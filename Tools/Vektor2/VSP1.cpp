#include "LibVSP.h"
#include "RollStabilization.h"
#include "Telemetry.h"
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
               const VSP1Params& params,
               RollStabilization* stabilization)
{
    const Limiter limits = setupLimiter(params.lim, params.x_c, params.y_c);
    const Coordinates input { vsp_common_map(inputs[0], VSP_PWM_MIN, VSP_PWM_MAX, params.x_c-params.lim, params.x_c+params.lim), vsp_common_map(inputs[1], VSP_PWM_MIN, VSP_PWM_MAX, params.y_c-params.lim, params.y_c+params.lim) };
    Coordinates result = mapCoordinates(input, limits, params.thr_ang);

    // VRS is strictly AFTER the existing limiter/angle correction. Disabled
    // routing supplies nullptr, so the original result reaches the outputs
    // unchanged, without additional clipping, rounding, or a ramp-down tail.
    if (stabilization != nullptr) {
        const bool valid_inputs = inputs[0] >= VSP_PWM_MIN && inputs[0] <= VSP_PWM_MAX &&
                                  inputs[1] >= VSP_PWM_MIN && inputs[1] <= VSP_PWM_MAX &&
                                  inputs[2] >= VSP_PWM_MIN && inputs[2] <= VSP_PWM_MAX;
        result = stabilization->apply(0, result, limits, valid_inputs);
    }

    outputs[0] = pulse_width(result.x_coordinate);
    outputs[1] = pulse_width(result.y_coordinate);
    outputs[2] = inputs[2]; // RPM_OUT <- RPM_IN, including while VRS is active
}

void vsp1_emit_telemetry(const uint16_t* outputs, const VspTelemetry& telemetry)
{
    telemetry.send_float("VSP1_XA", float(outputs[0]));
    telemetry.send_float("VSP1_YA", float(outputs[1]));
}

} // namespace Vektor2::Components
