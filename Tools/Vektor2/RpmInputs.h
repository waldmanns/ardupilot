#pragma once

#include "Telemetry.h"

namespace Vektor2 {

class RpmInputs : private NamedFloatTelemetry {
public:
    void init();
    void update(uint32_t now_ms, int16_t telemetry_hz);

    // Zero-based channel index, in revolutions per minute.
    float rpm(uint8_t channel) const;

private:
    struct Input {
        void on_falling_edge(uint8_t pin, bool state, uint32_t timestamp_us);

        uint32_t last_edge_us = 0;
        uint32_t period_us = 0;
        bool have_edge = false;
        float rpm = 0;
    };

    Input _inputs[2];
};

} // namespace Vektor2
