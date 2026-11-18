#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

namespace Vektor2 {

// Board-defined ADC measurement points, reported in volts over MAVLink.
class MeasurePoints {
public:
    MeasurePoints();
    void init();
    void update(uint32_t now_ms);

    static const AP_Param::GroupInfo var_info[];

private:
    static constexpr uint8_t point_count = 4;
    AP_Int16 _tel_hz;
    AP_HAL::AnalogSource* _sources[point_count] {};
    float _values[point_count] {};
    uint8_t _pending[8] {};
    uint32_t _snapshot_ms = 0;
    bool _sent_once = false;
};

} // namespace Vektor2
