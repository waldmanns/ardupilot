#include "Vektor_Vsp.h"

namespace Vektor {

void VspComponent::reset()
{
    _input_x = { 0.0F, 0, SignalQuality::INVALID };
    _input_y = { 0.0F, 0, SignalQuality::INVALID };
    _servo_a = { 0.0F, 0, SignalQuality::INVALID };
    _servo_b = { 0.0F, 0, SignalQuality::INVALID };
}

void VspComponent::set_inputs(const SignalSample<float> &x,
                              const SignalSample<float> &y)
{
    _input_x = x;
    _input_y = y;
}

void VspComponent::update(uint64_t now_us)
{
    (void)now_us;

    // Intentionally empty. Implement the VSP/VRS control law here.
    // Inputs are _input_x/_input_y; publish normalized logical commands via
    // _servo_a/_servo_b with the appropriate timestamp and quality.
}

} // namespace Vektor
