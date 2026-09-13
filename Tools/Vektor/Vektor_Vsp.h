#pragma once

#include "Vektor_Signal.h"

#include <stdint.h>

namespace Vektor {

// Board-independent logical core for one Voith Schneider Propeller instance.
// Physical PWM/CAN calibration remains the responsibility of routed endpoints.
class VspComponent {
public:
    void reset();
    void set_inputs(const SignalSample<float> &x,
                    const SignalSample<float> &y);
    void update(uint64_t now_us);

    const SignalSample<float> &input_x() const { return _input_x; }
    const SignalSample<float> &input_y() const { return _input_y; }
    const SignalSample<float> &servo_a() const { return _servo_a; }
    const SignalSample<float> &servo_b() const { return _servo_b; }

private:
    SignalSample<float> _input_x { 0.0F, 0, SignalQuality::INVALID };
    SignalSample<float> _input_y { 0.0F, 0, SignalQuality::INVALID };
    SignalSample<float> _servo_a { 0.0F, 0, SignalQuality::INVALID };
    SignalSample<float> _servo_b { 0.0F, 0, SignalQuality::INVALID };
};

} // namespace Vektor
