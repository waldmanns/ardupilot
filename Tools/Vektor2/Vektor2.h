#pragma once

#include "Config.h"
#include "GCS_MAVLink.h"
#include "Logic.h"
#include "Parameters.h"
#include "PwmOut.h"
#include "RcInput.h"
#include "RouteParameters.h"
#include "Routing.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#if AP_SIM_ENABLED
#include <SITL/SITL.h>
#endif

namespace Vektor2 {

class RCRegistry : public RC_Channels {
public:
    RC_Channel *channel(uint8_t) override { return nullptr; }
    int8_t flight_mode_channel_number() const override { return -1; }
};

class App {
public:
    App();

    void setup();
    void loop();

    static const AP_Param::Info var_info[];

    // Deliberately public service objects. Vektor2 is a composition root,
    // not a vehicle base class. Future routing code can consume these without
    // creating another global framework.
    AP_BoardConfig board_config;
    AP_Int16 format_version;
    Compass compass;
    AP_SerialManager serial_manager;
    AP_InertialSensor ins;
    AP_Baro baro;
    AP_Logger logger;
    AP_Scheduler scheduler;
#if AP_SIM_ENABLED
    SITL::SIM sitl;
#endif
    RCRegistry _rc_registry;
    AP_GPS gps;
    AP_AHRS ahrs;
    RcInput rcin;
    PwmOut pwm;
    Logic logic;
    Routing routing;
    RoutingParameters route_params;

    // ChibiOS RCOutput currently asks SRV_Channels whether output pins were
    // reassigned as GPIO/alarm during hal.rcout->init(). This object exists
    // only to satisfy that HAL registry contract. Vektor2 does not call
    // SRV_Channels::init(), register SERVO parameters, or use servo functions.
    SRV_Channels _rcout_registry;
#if HAL_GCS_ENABLED
    GCS_Vektor2 _gcs;
#endif

private:
    void load_parameters();
    void init_estimator();
    void update_mavlink(uint32_t now_ms);
    void sync_route_parameters(uint32_t now_ms);

    AP_Param param_loader{var_info};

    uint32_t _last_heartbeat_ms = 0;
    uint32_t _last_route_sync_ms = 0;
    bool _route_config_valid = true;
};

extern App app;

} // namespace Vektor2

void setup();
void loop();
