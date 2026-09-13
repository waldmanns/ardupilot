/*
   Vektor standalone ArduPilot application scaffold.

   This proves the Waf target, application entry point, board capability
   inventory, parameter store, and the first Vektor Serial Protocol surface.
 */

#include "Vektor.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace Vektor {

App vektor;

void App::setup()
{
    load_parameters();
    _active_capability = &default_capability_for_build();
    _runtime.init(default_service_rate_hz, AP_HAL::micros64());
    _vsp.reset();
    _serial_protocol.init(hal.serial(0),
                          *_active_capability,
                          g,
                          _runtime,
                          _vsp);
}

void App::loop()
{
    const uint64_t now_us = AP_HAL::micros64();
    _runtime.begin_loop(now_us);
    _vsp.update(now_us);
    _serial_protocol.update();
    _runtime.end_loop(AP_HAL::micros64());
    hal.scheduler->delay(1000U / default_service_rate_hz);
}

} // namespace Vektor

void setup()
{
    Vektor::vektor.setup();
}

void loop()
{
    Vektor::vektor.loop();
}

AP_HAL_MAIN();
