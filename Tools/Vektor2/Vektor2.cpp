/*
   Vektor2: minimal ArduPilot-library application.

   Intentionally present:
     - AP_HAL / board support
     - AP_InertialSensor + AP_AHRS + EKF3
     - AP_SerialManager
     - AP_RCProtocol / HAL RC input
     - HAL RC output
     - AP_GPS, active on a configured GPS serial port
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

#if AP_RCPROTOCOL_ENABLED && !AP_RC_CHANNEL_ENABLED
    // RC_Channels normally supplies this mask. Vektor2 has no RC_Channels,
    // so explicitly enable AP_RCProtocol's built-in decoders (bit 0 = all).
    AP::RC().set_rc_protocols(1);
#endif

    // board_config.init() owns ChibiOS RC-input initialization. It creates
    // AP::RC(), which must happen exactly once before SerialManager attaches
    // a port configured as SERIALx_PROTOCOL=23 (RCIN).

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

    // AP_GPS discovers the configured GPS serial port and probes for a
    // receiver. It is safe to initialize when no GPS is connected.
    gps.init();
    _gps_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 0);
    if (_gps_port != nullptr) {
        _gps_port->set_monitor_read_buffer(&_gps_monitor);
    }
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
#if HAL_NAVEKF3_AVAILABLE
    _compassless_ekf = compass.get_num_enabled() == 0;
    if (_compassless_ekf) {
        // Older Vektor2 firmware saved source set 2 as unaided. Use this set
        // as the compass-less fallback without rewriting saved parameters.
        AP_Param::set_by_name("EK3_SRC2_POSXY", 3);
        AP_Param::set_by_name("EK3_SRC2_VELXY", 3);
        AP_Param::set_by_name("EK3_SRC2_POSZ", 0);
        AP_Param::set_by_name("EK3_SRC2_VELZ", 3);
        AP_Param::set_by_name("EK3_SRC2_YAW", 8);
        ahrs.set_posvelyaw_source_set(AP_NavEKF_Source::SourceSetSelection::SECONDARY);
    }
#endif
}

void App::loop()
{
    // The IMU is the application clock. There is no vehicle scheduler or mode
    // layer here. wait_for_sample() gives deterministic estimator timing.
    ins.wait_for_sample();
    ins.update();

    gps.update();
    // This buffer contains copies of bytes AP_GPS has already read. Reading it
    // cannot take bytes away from the GPS parser.
    uint8_t gps_byte;
    while (_gps_monitor.read_byte(&gps_byte)) {
        _gps_rx_bytes++;
        if (_gps_previous_byte == '$' && (gps_byte == 'G' || gps_byte == 'P')) {
            _gps_nmea_seen = true;
        }
        if (_gps_previous_byte == 0xB5 && gps_byte == 0x62) {
            _gps_ubx_seen = true;
        }
        _gps_previous_byte = gps_byte;
    }
    rcin.update();



    // Compass samples can feed the estimator and standard IMU telemetry.
    const uint32_t compass_now_ms = AP_HAL::millis();
    if (compass_now_ms - _last_compass_read_ms >= 100) {
        _last_compass_read_ms = compass_now_ms;
        compass.read();
#if COMPASS_CAL_ENABLED
        compass.cal_update();
#endif
    }

#if HAL_NAVEKF3_AVAILABLE
    // Vektor2 has no arming state. Let EKF3 run its GSF yaw estimator once
    // a compass-less vehicle is moving with a GPS fix. The GSF itself checks
    // whether its yaw estimate is usable before aligning the main filter.
    if (_compassless_ekf &&
        gps.status(0) >= AP_GPS::GPS_OK_FIX_3D &&
        gps.ground_speed() >= 1.0f) {
        ahrs.set_takeoff_expected(true);
    }
#endif

    // We already updated INS explicitly, so tell AHRS not to do it again.
    ahrs.update(true);

    const uint32_t now_ms = AP_HAL::millis();

    // Pick up RTn_SRC/RTn_DST changes made through the normal MAVLink parameter
    // protocol. This is intentionally low-rate configuration work.
    sync_route_parameters(now_ms);
    report_imu_health(now_ms);
    report_gps_diagnostics(now_ms);
    report_ekf_startup(now_ms);
    report_rc_protocol(now_ms);

    // Route uint16 microsecond signals through the two logic components and
    // on to final PWM consumers. Sources may fan out; each consumer has only
    // one producer. Component dependency order is compiled when routes change.
    routing.update(rcin, logic, pwm);

    vsp_telemetry.update(now_ms, logic.vsp_tel_hz.get(), logic);

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

void App::report_gps_diagnostics(uint32_t now_ms)
{
#if HAL_GCS_ENABLED
    if (_gps_port == nullptr) {
        if (now_ms >= 2000 && !_gps_settings_reported) {
            _gps_settings_reported = true;
            gcs().send_text(MAV_SEVERITY_WARNING, "GPS1: no GPS serial port");
        }
        return;
    }

    const uint32_t baud = _gps_port->get_baud_rate();
    if (baud != _gps_last_baud) {
        _gps_last_baud = baud;
        if (_gps_baud_reports < 5) {
            _gps_baud_reports++;
            gcs().send_text(MAV_SEVERITY_INFO, "GPS1 probing %lu baud",
                            (unsigned long)baud);
        }
    }

    if (now_ms >= 2000 && !_gps_settings_reported) {
        _gps_settings_reported = true;
        enum ap_var_type type;
        AP_Param *auto_config = AP_Param::find("GPS_AUTO_CONFIG", &type);
        const int auto_value = auto_config != nullptr && type == AP_PARAM_INT8 ?
            static_cast<AP_Int8 *>(auto_config)->get() : -1;
        AP_Param *driver_options = AP_Param::find("GPS_DRV_OPTIONS", &type);
        const int options_value = driver_options != nullptr && type == AP_PARAM_INT16 ?
            static_cast<AP_Int16 *>(driver_options)->get() : -1;
        gcs().send_text(MAV_SEVERITY_INFO, "GPS1 type=%u auto=%d drv=%d",
                        unsigned(gps.get_type(0)), auto_value, options_value);
    }

    if (gps.status(0) != AP_GPS::NO_GPS ||
        now_ms - _last_gps_diagnostic_ms < 15000) {
        return;
    }
    _last_gps_diagnostic_ms = now_ms;
    const char *rx = _gps_rx_bytes == 0 ? "no RX" :
                     _gps_nmea_seen && _gps_ubx_seen ? "NMEA+UBX" :
                     _gps_nmea_seen ? "NMEA" :
                     _gps_ubx_seen ? "UBX" : "unknown";
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "GPS1 RX=%s baud=%lu; no backend", rx,
                    (unsigned long)baud);
#else
    (void)now_ms;
#endif
}

void App::report_ekf_startup(uint32_t now_ms)
{
#if HAL_GCS_ENABLED && HAL_NAVEKF3_AVAILABLE
    if (now_ms < 10000 || now_ms - _last_ekf_startup_ms < 15000) {
        return;
    }
    nav_filter_status status{};
    ahrs.get_filter_status(status);
    if (status.flags.attitude) {
        return;
    }
    _last_ekf_startup_ms = now_ms;
    if (status.flags.initalized) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "EKF3 attitude pending: compass=%u GPS=%u",
                        unsigned(compass.get_num_enabled()),
                        unsigned(gps.status(0)));
    } else {
        float gps_lag_sec;
        const bool gps_lag_known = gps.get_lag(0, gps_lag_sec);
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "EKF3 waiting: GPS lag=%u free=%lu",
                        unsigned(gps_lag_known),
                        (unsigned long)hal.util->available_memory());
    }
#else
    (void)now_ms;
#endif
}

void App::report_rc_protocol(uint32_t now_ms)
{
#if HAL_GCS_ENABLED
    if (now_ms - _last_rc_protocol_report_ms < 5000) {
        return;
    }
    _last_rc_protocol_report_ms = now_ms;
    const char* protocol = rcin.valid() ? rcin.protocol_name() : nullptr;
    gcs().send_named_string("RC_PROTO",
                            protocol != nullptr ? protocol : "NONE");

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
