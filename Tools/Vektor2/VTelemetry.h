#pragma once

#include "VTelemetryCore.h"
#include "VTelemetryParameters.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

namespace Vektor2 {

class App;

// UART-only telemetry; does not own any actuator or alter MAVLink streams.
// Sensor reads and mapping edits occur in App::loop() at 10 Hz. Dedicated
// low-priority I/O tasks handle the timing-sensitive receiver polls using
// copies of that snapshot, never accessing the estimator or flight control.
class VTelemetry {
public:
    void init(App& application, VTelemetryParameters& parameters, AP_SerialManager& serial);
    void update(App& application, VTelemetryParameters& parameters, uint32_t now_ms);

private:
    static bool read_source(uint16_t id, VTelemetryCore::Sample& sample, void* context);
    void ibus_task();
    void hott_task();

    VTelemetryCore::Router _router;
    HAL_Semaphore _sem;
    AP_HAL::UARTDriver* _ibus = nullptr;
    AP_HAL::UARTDriver* _hott = nullptr;
    uint32_t _last_update_ms = 0;
    bool _initialized = false;
};

} // namespace Vektor2
