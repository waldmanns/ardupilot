/*
   Vektor2: minimal ArduPilot-library application.

   Intentionally present:
     - AP_HAL / board support
     - AP_InertialSensor + AP_AHRS + EKF3
     - AP_SerialManager
     - AP_RCProtocol / HAL RC input
     - HAL RC output
     - board-gated CAN manager / DroneCAN sensor and PWM output backends
     - AP_GPS, active on a configured GPS serial port
     - standard MAVLink/GCS parameter and telemetry transport

   Intentionally absent:
     - AP_Vehicle
     - AP_Arming
     - vehicle modes
     - mission architecture
     - vehicle failsafe state machine
     - vehicle servo-function mixer
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

void App::init_status_led()
{
#ifdef AP_NOTIFY_GPIO_LED_1_PIN
    // The hwdef declares the single board LED, but Vektor2 has no vehicle
    // notification/arming state machine. Drive a control-loop heartbeat.
#if HAL_GPIO_LED_ON != 0
    // Preserve open-drain configuration on active-low LED boards.
    hal.gpio->pinMode(AP_NOTIFY_GPIO_LED_1_PIN, HAL_GPIO_OUTPUT);
#endif
    _status_led_on = true;
    _last_status_led_ms = AP_HAL::millis();
    hal.gpio->write(AP_NOTIFY_GPIO_LED_1_PIN, HAL_GPIO_LED_ON);
#endif
}

void App::update_status_led(uint32_t now_ms)
{
#ifdef AP_NOTIFY_GPIO_LED_1_PIN
    if (now_ms - _last_status_led_ms < 500) {
        return;
    }
    _last_status_led_ms = now_ms;
    _status_led_on = !_status_led_on;
    hal.gpio->write(AP_NOTIFY_GPIO_LED_1_PIN,
                    _status_led_on ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
#else
    (void)now_ms;
#endif
}

void App::setup()
{
    load_parameters();

    // BoardConfig stays because it is hardware infrastructure, not vehicle
    // behaviour. It applies board orientation and board-level driver options.
    board_config.init();
    init_status_led();
    measure_points.init();

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
    rcin.init();

#if HAL_GCS_ENABLED
    gcs().setup_console();
    gcs().setup_uarts();
#endif

#if HAL_CANMANAGER_ENABLED
    // Start standard CAN protocol drivers before the sensor frontends probe
    // for DroneCAN devices. CAN manager owns interfaces and driver threads.
    can_mgr.init();
#endif

    // AP_GPS discovers the configured GPS serial port and probes for a
    // receiver. It is safe to initialize when no GPS is connected.
    gps.init();
    _gps_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 0);
    if (_gps_port != nullptr) {
        _gps_port->set_monitor_read_buffer(&_gps_monitor);
    }
    compass.init();

    // Only log maintenance uses the scheduler. Sensor/control/PWM timing stays
    // in this IMU-clocked loop. The logger's startup writer needs a real task
    // budget; a scheduler with no tasks would leave log metadata unfinished.
    static const AP_Scheduler::Task tasks[] = {
        SCHED_TASK_CLASS(App, this, logging_task, LOOP_RATE, 1000, 100),
    };
    scheduler.init(tasks, ARRAY_SIZE(tasks), 0);
    init_estimator();
    pwm.init();
    rpm_inputs.init();

    // Build the runtime route table from the standard MAVLink/AP_Param-backed
    // RTn_SRC/RTn_DST configuration before the first loop iteration.
    routing.attach_sequencer(sequencer.engine);
    _route_config_valid = route_params.sync(routing, pwm, true);
    init_logging();
    storage.init();
    sequencer.init();
    vtel.init(*this, vtel_params, serial_manager);

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
    if (!storage.formatting()) {
        sequencer.end_maintenance();
    }
    sequencer.update(now_ms);
    routing.update(rcin, logic, pwm);
#if HAL_ENABLE_DRONECAN_DRIVERS
    pwm.push_can();
#endif

    update_navigation_status(now_ms);
    nav_telemetry.update(now_ms, nav_supervisor.status());
    dp_telemetry.update(now_ms, dp_tel_hz.get(), nav_dp.output(), nav_heading.output());
    function_telemetry.update(now_ms, logic);
    vsp_telemetry.update(now_ms, logic.vsp_tel_hz.get(), logic);
    thruster_telemetry.update(now_ms, logic.thr_tel_hz.get(), logic);
    measure_points.update(now_ms);
    rpm_inputs.update(now_ms, rpm_tel_hz.get());
    vtel.update(*this, vtel_params, now_ms);

    update_mavlink(now_ms);
    update_status_led(now_ms);
    scheduler.tick();
    scheduler.run(1500);
}


void App::update_navigation_status(uint32_t now_ms)
{
    // M3: calculate vessel-level DP / shared heading diagnostics only.
    // Manual routing continues unchanged; no automatic output ownership.
    // The ordinary VFN route/slot evaluator has already sampled the RC
    // switch. Read the single NavMode action rather than creating another
    // channel parser or hard-coding a switch number. Keep VNAV_MODE
    // readable as the live request, but never save RC changes to flash.
    const auto rc_selection = logic.vfn.navigation_selection(rcin.valid());
    if (rc_selection.assigned) {
        if (navigation.mode.get() != rc_selection.mode) {
            navigation.mode.set(rc_selection.mode);
        }
    } else if (_navigation_rc_was_assigned) {
        // Removing the last RC assignment must not leave an old automatic
        // request latched in RAM. Manual parameter control then resumes.
        navigation.mode.set(0);
    }
    _navigation_rc_was_assigned = rc_selection.assigned;

    NavigationSupervisor::Input input;
    input.requested_mode = navigation.mode.get();
    input.enabled = navigation.enable.get() != 0;
    input.heading_enabled = navigation.hdg_enable.get() != 0 &&
                            logic.vfn.permits(Functions::Type::Heading);
    input.dp_enabled = navigation.dp_enable.get() != 0;
    input.rc_valid = rcin.valid();

    nav_filter_status filter{};
    const bool status_ok = ahrs.get_filter_status(filter) && filter.flags.initalized &&
                           ins.healthy();
    const float heading_deg = ahrs.get_yaw_deg();
    input.heading_valid = status_ok && filter.flags.attitude && std::isfinite(heading_deg);
    Location location{};
    Vector3f velocity{};
    // AHRS/EKF3's own body-origin position; do not use raw GPS coordinates
    // or reapply GPS antenna lever arms here. Check the same point at capture.
    input.position_valid = status_ok && filter.flags.horiz_pos_abs &&
                           ahrs.get_location(location) &&
                           NavigationDP::point_valid({location.lat, location.lng});
    input.velocity_valid = status_ok && filter.flags.horiz_vel &&
                           ahrs.get_velocity_NED(velocity) &&
                           std::isfinite(velocity.x) && std::isfinite(velocity.y);

    const uint32_t elapsed_ms = _last_navigation_ms == 0 ? 10U : now_ms - _last_navigation_ms;
    input.timing_ok = elapsed_ms > 0U && elapsed_ms <= 100U;
    _last_navigation_ms = now_ms;
    // Reuse AHRS/EKF3 reset notifications (yaw, horizontal position and
    // horizontal velocity). Never create an independent jump detector.
    float yaw_reset_angle = 0;
    Vector2f pos_reset_ne{};
    Vector2f vel_reset_ne{};
    const uint32_t yaw_reset_ms = ahrs.getLastYawResetAngle(yaw_reset_angle);
    const uint32_t pos_reset_ms = ahrs.getLastPosNorthEastReset(pos_reset_ne);
    const uint32_t vel_reset_ms = ahrs.getLastVelNorthEastReset(vel_reset_ne);
    input.estimator_reset = _nav_ekf_resets.update(yaw_reset_ms,
                                                    pos_reset_ms, vel_reset_ms);

    const int8_t mode = input.requested_mode;
    const bool hdg_capture = navigation.hdg_capture.get() != 0;
    const bool dp_hdg_capture = navigation.dp_capture_hdg.get() != 0;
    const bool dp_pos_capture = navigation.dp_capture_pos.get() != 0;
    const bool request_hdg_capture = (hdg_capture && !_previous_hdg_capture) ||
                                     (dp_hdg_capture && !_previous_dp_hdg_capture);
    const bool request_pos_capture = dp_pos_capture && !_previous_dp_pos_capture;
    _previous_hdg_capture = hdg_capture;
    _previous_dp_hdg_capture = dp_hdg_capture;
    _previous_dp_pos_capture = dp_pos_capture;

    if (mode == 0 || mode < 0 || mode > 2) {
        nav_heading.clear_target();
        nav_dp.clear_target();
        _heading_capture_pending = false;
        _dp_capture_pending = false;
    } else if (input.estimator_reset) {
        // No silent re-acquisition after an estimator yaw / position / velocity
        // reset. An explicit capture request or mode transition is required.
        nav_heading.clear_target();
        nav_dp.clear_target();
        _heading_capture_pending = false;
        _dp_capture_pending = false;
    } else {
        if (_previous_nav_mode == 0) {
            _heading_capture_pending = true;
        }
        if (mode == 2 && _previous_nav_mode != 2) {
            _dp_capture_pending = true;
        }
        if (mode != 2) {
            // Do not publish an old geographic position target in Heading Hold.
            nav_dp.clear_target();
            _dp_capture_pending = false;
        }
        if (request_hdg_capture) {
            nav_heading.clear_target();
            _heading_capture_pending = true;
        }
        if (mode == 2 && request_pos_capture) {
            nav_dp.clear_target();
            _dp_capture_pending = true;
        }
        if (_heading_capture_pending && input.enabled && input.heading_enabled &&
            input.rc_valid && input.heading_valid) {
            if (nav_heading.capture(heading_deg)) {
                _heading_capture_pending = false;
            }
        }
        if (mode == 2 && _dp_capture_pending && input.enabled && input.dp_enabled &&
            input.rc_valid && input.position_valid && input.velocity_valid) {
            NavigationDP::Point actual{location.lat, location.lng};
            if (nav_dp.capture(actual)) {
                _dp_capture_pending = false;
            }
        }
    }
    _previous_nav_mode = mode;

    const auto& gyro = ins.get_gyro();
    float yaw_rate_deg_s = NAN;
    if (input.heading_valid && input.timing_ok) {
        NavigationHeading::earth_yaw_rate(gyro.y, gyro.z,
                ahrs.get_roll_deg(), ahrs.get_pitch_deg(), yaw_rate_deg_s);
    }
    NavigationHeading::Config heading_config{};
    heading_config.kp = navigation.hdg_kp.get();
    heading_config.kd = navigation.hdg_kd.get();
    heading_config.max_yaw_percent = navigation.hdg_max_yaw.get();
    heading_config.deadband_deg = navigation.hdg_deadband.get();
    // Keep actual estimated heading observable even with automation disabled.
    // Controller output validity remains gated by mode, timing and target.
    nav_heading.update(heading_deg, yaw_rate_deg_s, input.heading_valid,
                       heading_config,
                       input.timing_ok && !input.estimator_reset &&
                       (mode == 1 || mode == 2) && input.enabled && input.heading_enabled);

    NavigationDP::Input dp_in{};
    dp_in.actual = {location.lat, location.lng};
    dp_in.position_valid = input.position_valid;
    dp_in.velocity_valid = input.velocity_valid && mode == 2 &&
                           input.enabled && input.dp_enabled;
    dp_in.heading_valid = nav_heading.output().valid &&
                          input.heading_valid && input.timing_ok && !input.estimator_reset;
    dp_in.velocity_n_ms = velocity.x;
    dp_in.velocity_e_ms = velocity.y;
    dp_in.heading_deg = heading_deg;
    dp_in.estimator_reset = input.estimator_reset;
    NavigationDP::Config dp_config{};
    dp_config.position_kp = navigation.dp_pos_kp.get();
    dp_config.velocity_kp = navigation.dp_vel_kp.get();
    dp_config.max_speed_ms = navigation.dp_max_speed.get();
    dp_config.max_accel_ms2 = navigation.dp_accel_limit.get();
    dp_config.radius_m = navigation.dp_radius.get();
    dp_config.velocity_filter_hz = navigation.dp_vel_filter.get();
    nav_dp.update(dp_in, dp_config, float(elapsed_ms) * 0.001f);

    // DP-specific configuration failures must not inhibit an otherwise
    // functioning shared Heading Hold controller during DP_WAIT.
    input.configuration_valid = rc_selection.valid && !rc_selection.conflict &&
        (mode == 0 || NavigationHeading::config_valid(heading_config));
    input.dp_controller_reasons = nav_dp.output().reason_bits;
    input.heading_valid = input.heading_valid && nav_heading.output().valid;
    input.heading_target_valid = nav_heading.target_valid();
    input.position_target_valid = nav_dp.target_valid();
    // No actuator ownership ACK exists yet. Never report HDG_ACTIVE/DP_ACTIVE.
    input.output_ready = false;
    input.yaw_owned = false;
    input.translation_owned = false;
    nav_supervisor.update(input);
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
    _gcs.update_storage_information(now_ms);
    _gcs.update_sequencer(now_ms);
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
