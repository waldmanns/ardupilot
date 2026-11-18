#include "MeasurePoints.h"
#include <GCS_MAVLink/GCS.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

GCS ground_station;
unsigned space[3] {};
std::vector<mavlink_named_value_float_t> sent[3];

class Source : public AP_HAL::AnalogSource {
public:
    float voltage = 0;
    unsigned reads = 0;
    float voltage_average() override { reads++; return voltage; }
};

class Analog : public AP_HAL::AnalogIn {
public:
    Source sources[19];
    std::vector<int16_t> pins;
    AP_HAL::AnalogSource* channel(int16_t pin) override {
        pins.push_back(pin);
        return pin == 18 ? nullptr : &sources[pin];
    }
} analog;

const AP_HAL::HAL hardware {&analog};
const AP_HAL::HAL& hal = hardware;

static void rate(Vektor2::MeasurePoints& points, int16_t hz)
{
    AP_Param::set(&points, Vektor2::MeasurePoints::var_info[0], hz);
}

int main()
{
    Vektor2::MeasurePoints points;
    points.init();
    space[0] = space[1] = space[2] = 100;
    points.update(100);
    assert(sent[0].empty()); // default is disabled
#ifdef HAL_VEKTOR2_MEASURE_POINTS
    assert((analog.pins == std::vector<int16_t> {4, 8, 18, 0}));
    analog.sources[4].voltage = 1.0f;
    analog.sources[8].voltage = 5.0f / 6.6f;
    analog.sources[0].voltage = 12.0f / 11.0f;
    ground_station.links[1].private_link = true;
    ground_station.links[2].is_high_latency_link = true;
    rate(points, 10);
    space[0] = 1;
    points.update(100);
    assert(sent[0].size() == 1);
    assert(strcmp(sent[0][0].name, "MP_V_VCC") == 0);
    assert(fabsf(sent[0][0].value - 6.6f) < 0.0001f);
    assert(sent[1].empty() && sent[2].empty());
    // Busy links resume the same snapshot without reading ADCs again.
    space[0] = 3;
    points.update(110);
    assert(sent[0].size() == 4);
    assert(strcmp(sent[0][1].name, "MP_V_V5") == 0);
    assert(fabsf(sent[0][1].value - 5.0f) < 0.0001f);
    assert(strcmp(sent[0][2].name, "MP_V_VCAN") == 0);
    assert(std::isnan(sent[0][2].value)); // allocation failure cannot look like zero volts
    assert(strcmp(sent[0][3].name, "MP_V_BAT") == 0);
    assert(fabsf(sent[0][3].value - 12.0f) < 0.0001f);
    for (const auto& msg : sent[0]) {
        assert(msg.time_boot_ms == 100);
    }
    assert(analog.sources[4].reads == 1);
    space[0] = 100;
    points.update(199);
    assert(sent[0].size() == 4);
    ground_station.links[1].private_link = false;
    points.update(200);
    assert(sent[0].size() == 8 && sent[1].size() == 4);
    // Disabled telemetry clears partial snapshots, and resumes immediately.
    space[0] = 0;
    points.update(300);
    rate(points, 0);
    space[0] = 100;
    points.update(310);
    assert(sent[0].size() == 8);
    rate(points, -1);
    points.update(320);
    assert(sent[0].size() == 8);
    rate(points, 1000);
    points.update(321);
    assert(sent[0].size() == 12);
    points.update(340);
    assert(sent[0].size() == 12);
    points.update(341);
    assert(sent[0].size() == 16); // rates above 50 are capped
    // Millisecond wrap is safe, and non-divisor rates round down in frequency.
    rate(points, 0);
    points.update(0xfffffff0U);
    rate(points, 3);
    points.update(0xfffffff0U);
    const size_t before_wrap = sent[0].size();
    points.update(317);
    assert(sent[0].size() == before_wrap);
    points.update(318);
    assert(sent[0].size() == before_wrap + 4);
    // Inactive links discard unsent fields.
    space[0] = 0;
    points.update(652);
    ground_station.links[0].active = false;
    space[0] = 100;
    points.update(662);
    assert(sent[0].size() == before_wrap + 4);
    puts("PASS: MP scaling, keys, snapshots, rates, disable, wrap, missing ADC and link retries");
#else
    assert(analog.pins.empty());
    rate(points, 50);
    points.update(200);
    assert(sent[0].empty() && sent[1].empty() && sent[2].empty());
    puts("PASS: boards without MP declarations allocate no ADCs and send no MP values");
#endif
}
