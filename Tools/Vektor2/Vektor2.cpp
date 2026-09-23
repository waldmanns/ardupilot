/*
   Vektor2: minimal ArduPilot-library application.

   Intentionally present:
     - AP_HAL / board support
     - AP_InertialSensor + AP_AHRS + EKF3
     - AP_SerialManager
     - AP_RCProtocol / HAL RC input
     - HAL RC output
     - AP_GPS, disabled by default but ready for later use
     - standard MAVLink/GCS parameter and telemetry transport

   Intentionally absent:
     - AP_Vehicle
     - AP_Arming
     - vehicle modes
     - mission architecture
     - vehicle failsafe state machine
     - RC_Channel / SRV_Channels
 */

#include "Vektor2.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace Vektor2 {

App app;

App::App() :
    ahrs(AP_AHRS::FLAG_ALWAYS_USE_EKF)
{
}

void App::setup()
{
    load_parameters();

    // BoardConfig stays because it is hardware infrastructure, not vehicle
    // behaviour. It applies board orientation and board-level driver options.
    board_config.init();

    // RC protocol must exist before SerialManager attaches a port configured
    // as SERIALx_PROTOCOL=23 (RCIN).
    rcin.init();

#if HAL_GCS_ENABLED
    // Register the lightweight GCS singleton before serial ports are scanned.
    gcs().init();
#endif

    // Starts configured UARTs. MAVLink, GPS and RCIN all share this one clean
    // serial-role surface instead of Vektor owning a second protocol router.
    serial_manager.init();

#if HAL_GCS_ENABLED
    gcs().setup_console();
    gcs().setup_uarts();
#endif

    // GPS_TYPE defaults to none in current ArduPilot. Calling init() is safe
    // without hardware and makes GPS a parameter-only addition later.
    gps.init();
    compass.init();

    scheduler.init(nullptr, 0, 0);
    init_estimator();
    pwm.init();

    // Build the runtime route table from the standard MAVLink/AP_Param-backed
    // RTn_SRC/RTn_DST configuration before the first loop iteration.
    _route_config_valid = route_params.sync(routing, pwm, true);

#if HAL_GCS_ENABLED
    gcs().send_text(MAV_SEVERITY_INFO,
                    "Vektor2 %u.%u.%u ready",
                    unsigned(firmware_major),
                    unsigned(firmware_minor),
                    unsigned(firmware_patch));
    GCS_SEND_MESSAGE(MSG_HEARTBEAT);
#endif
}

void App::init_estimator()
{
#if HAL_NAVEKF3_AVAILABLE
    ahrs.set_ekf_type(AP_AHRS::EKFType::THREE);
#endif
    ahrs.set_fly_forward(false);
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::UNKNOWN);

    // This ordering follows current ArduPilot startup: create/configure AHRS,
    // initialise the IMU at the application loop rate, then reset the AHRS.
    ahrs.init();
    ins.init(loop_rate_hz);
    ahrs.reset();
}

void App::loop()
{
    // The IMU is the application clock. There is no vehicle scheduler or mode
    // layer here. wait_for_sample() gives deterministic estimator timing.
    ins.wait_for_sample();
    ins.update();

    gps.update();
    rcin.update();

#if HAL_GCS_ENABLED
    if (rcin.take_protocol_changed()) {
        const char* protocol = rcin.protocol_name();
        if (protocol != nullptr) {
            gcs().send_text(MAV_SEVERITY_INFO, "RCIN detected: %s", protocol);
        }
    }
#endif

    // Compass samples can feed the estimator and standard IMU telemetry.
    const uint32_t compass_now_ms = AP_HAL::millis();
    if (compass_now_ms - _last_compass_read_ms >= 100) {
        _last_compass_read_ms = compass_now_ms;
        compass.read();
    }

    // We already updated INS explicitly, so tell AHRS not to do it again.
    ahrs.update(true);

    const uint32_t now_ms = AP_HAL::millis();

    // Pick up RTn_SRC/RTn_DST changes made through the normal MAVLink parameter
    // protocol. This is intentionally low-rate configuration work.
    sync_route_parameters(now_ms);
    report_imu_health(now_ms);

    // Route uint16 microsecond signals through the two logic components and
    // on to final PWM consumers. Sources may fan out; each consumer has only
    // one producer. Component dependency order is compiled when routes change.
    routing.update(rcin, logic, pwm);

    update_mavlink(now_ms);
}


void App::sync_route_parameters(uint32_t now_ms)
{
    if (now_ms - _last_route_sync_ms < route_parameter_poll_ms) {
        return;
    }
    _last_route_sync_ms = now_ms;

    const bool valid = route_params.sync(routing, pwm);
#if HAL_GCS_ENABLED
    if (valid != _route_config_valid) {
        gcs().send_text(valid ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                        valid ? "Routing configuration valid"
                              : "Routing configuration has invalid route(s)");
    }
#endif
    _route_config_valid = valid;
}

void App::report_imu_health(uint32_t now_ms)
{
#if HAL_GCS_ENABLED
    // Report once after startup, then only on a health transition.
    if (now_ms - _last_imu_health_ms < 1000) {
        return;
    }
    _last_imu_health_ms = now_ms;
    const bool healthy = ins.get_gyro_count() > 0 &&
                         ins.get_accel_count() > 0 && ins.healthy();
    if (_imu_health_reported && healthy == _imu_healthy) {
        return;
    }
    _imu_health_reported = true;
    _imu_healthy = healthy;
    gcs().send_text(healthy ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                    "IMU %s: %u gyro, %u accel",
                    healthy ? "healthy" : "unhealthy",
                    unsigned(ins.get_gyro_count()),
                    unsigned(ins.get_accel_count()));
#else
    (void)now_ms;
#endif
}

void App::update_mavlink(uint32_t now_ms)
{
#if HAL_GCS_ENABLED
    // Keep command/parameter handling responsive on every estimator cycle.
    gcs().update_receive();

    if (now_ms - _last_heartbeat_ms >= heartbeat_period_ms) {
        _last_heartbeat_ms = now_ms;
        GCS_SEND_MESSAGE(MSG_HEARTBEAT);
    }

    // ATTITUDE, GPS_RAW_INT, RC_CHANNELS and any future telemetry are owned
    // by the standard GCS scheduler. Ground stations can use stream requests
    // or MAV_CMD_SET_MESSAGE_INTERVAL without fighting an application timer.
    gcs().update_send();
#else
    (void)now_ms;
#endif
}

} // namespace Vektor2

void setup()
{
    Vektor2::app.setup();
}

void loop()
{
    Vektor2::app.loop();
}

AP_HAL_MAIN();
