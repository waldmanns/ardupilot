#pragma once

#include "Vektor_Signal.h"

#include <math.h>
#include <stdint.h>

namespace Vektor {

struct AttitudeValue {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    float body_rate_x_rad_s;
    float body_rate_y_rad_s;
    float body_rate_z_rad_s;
};

// Transport-neutral snapshot of the onboard attitude estimator.  Production
// code feeds this from AP_AHRS; tests and other estimators can feed the same
// serial-facing state without coupling the protocol to AP_AHRS.
class AttitudeSource {
public:
    void reset()
    {
        _value = {};
        _timestamp_us = 0;
        _quality = SignalQuality::INVALID;
    }

    void ingest(const AttitudeValue &value,
                uint64_t timestamp_us,
                bool healthy)
    {
        _value = value;
        _timestamp_us = timestamp_us;
        _quality = healthy && finite(value) ? SignalQuality::VALID :
                                               SignalQuality::INVALID;
    }

    const AttitudeValue &value() const { return _value; }
    uint64_t timestamp_us() const { return _timestamp_us; }
    SignalQuality quality() const { return _quality; }

private:
    static bool finite(const AttitudeValue &value)
    {
        return isfinite(value.roll_deg) &&
               isfinite(value.pitch_deg) &&
               isfinite(value.yaw_deg) &&
               isfinite(value.quaternion_w) &&
               isfinite(value.quaternion_x) &&
               isfinite(value.quaternion_y) &&
               isfinite(value.quaternion_z) &&
               isfinite(value.body_rate_x_rad_s) &&
               isfinite(value.body_rate_y_rad_s) &&
               isfinite(value.body_rate_z_rad_s);
    }

    AttitudeValue _value {};
    uint64_t _timestamp_us = 0;
    SignalQuality _quality = SignalQuality::INVALID;
};

} // namespace Vektor
