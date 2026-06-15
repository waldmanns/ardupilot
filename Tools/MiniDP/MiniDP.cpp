/*
   MiniDP standalone ArduPilot application shell.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#include "MiniDP.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/LogStructure.h>
#include <StorageManager/StorageManager.h>
#include <stdio.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

MiniDP minidp;

#if AP_SIM_ENABLED
SITL::SIM sitl;
#endif
#if AP_EXTERNAL_AHRS_ENABLED
AP_ExternalAHRS external_ahrs;
#endif
AP_Scheduler scheduler;

const LogStructure MiniDP::log_structure[] = {
    LOG_COMMON_STRUCTURES,
};

void MiniDP::load_parameters()
{
    AP_Param::setup_sketch_defaults();
    AP_Param::check_var_info();

    if (!g.format_version.load() || g.format_version != k_format_version) {
        StorageManager::erase();
        AP_Param::erase_all();
        g.format_version.set_and_save(k_format_version);
    }
    g.format_version.set_default(k_format_version);
    AP_Param::load_all();
}

void MiniDP::disable_outputs()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    for (uint8_t channel = 0; channel < output_count; channel++) {
        hal.rcout->disable_ch(channel);
    }
#endif
}

void MiniDP::setup()
{
    printf("MiniDP basic firmware shell\n");

    load_parameters();
    printf("MiniDP parameters ready\n");
    board_config.init();
    printf("MiniDP board config ready\n");
    serial_manager.init();
    printf("MiniDP serial manager ready\n");
    rc_channels.init();
    printf("MiniDP RC frontend ready\n");

    disable_outputs();

#if HAL_LOGGING_ENABLED
    logger.init(g.log_bitmask, log_structure, ARRAY_SIZE(log_structure));
    logger.set_force_log_disarmed(true);
    logger.Write_Message("MiniDP basic firmware shell");
    printf("MiniDP logger ready\n");
#endif

    ins.init(100);
    printf("MiniDP IMU frontend ready\n");
    barometer.init();
    if (barometer.num_instances() > 0) {
        barometer.calibrate(false);
    }
    printf("MiniDP barometer frontend ready\n");
    compass.init();
    printf("MiniDP compass frontend ready\n");
    gps.init();
    printf("MiniDP GPS frontend ready\n");
    ahrs.init();
    ahrs.set_fly_forward(false);
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::GROUND);
    ahrs.reset();
    printf("MiniDP AHRS frontend ready\n");
    mode_manager.init(AP_HAL::micros64());
    authority.init(AP_HAL::micros64());

    notify.init();
    AP_Notify::flags.initialising = false;

#if HAL_GCS_ENABLED
    gcs().init();
    gcs().setup_console();
    gcs().setup_uarts();
#endif
    report_mode_transition();
    report_authority_transition();

    printf("MiniDP initialization complete; outputs disabled\n");
}

void MiniDP::update_authority(const uint32_t now_ms)
{
    const bool have_rc_backend = (hal.rcin != nullptr) && (hal.rcin->num_channels() > 0);
    const bool new_rc_input = have_rc_backend && rc_channels.read_input();
    if (new_rc_input) {
        last_rc_input_ms = now_ms;
    }

    bool rc_protocol_failsafe = false;
#if AP_RCPROTOCOL_ENABLED
    rc_protocol_failsafe = AP::RC().failsafe_active();
#endif

    const uint32_t rc_timeout_ms =
        MAX(uint32_t(50), uint32_t(MAX(g.auth_rc_timeout.get(), 0.0f) * 1000.0f));
    const uint32_t mavlink_timeout_ms =
        MAX(uint32_t(100), uint32_t(MAX(g.auth_mav_timeout.get(), 0.0f) * 1000.0f));
#if HAL_GCS_ENABLED
    const uint32_t mavlink_last_seen_ms = gcs().sysid_mygcs_last_seen_time_ms();
#else
    const uint32_t mavlink_last_seen_ms = 0;
#endif

    authority_status = {};
    authority_status.time_us = AP_HAL::micros64();
    authority_status.rc_healthy =
        have_rc_backend &&
        last_rc_input_ms != 0 &&
        now_ms - last_rc_input_ms <= rc_timeout_ms &&
        !rc_protocol_failsafe;
    authority_status.mavlink_healthy =
        mavlink_last_seen_ms != 0 &&
        now_ms - mavlink_last_seen_ms <= mavlink_timeout_ms;
    authority_status.mavlink_manual_authorized = g.auth_mav_manual != 0;
    authority_status.mavlink_target_authorized = g.auth_mav_target != 0;
    authority_status.actuator_test_authorized = g.auth_test != 0;
    authority_status.failsafe_active =
        mode_manager.mode() == MiniDP_Mode::FAILSAFE;

    MiniDP_AuthorityPolicy policy{};
    policy.rc_takeover_from_mavlink = g.auth_rc_takeover != 0;
    policy.rc_fallback_on_mavlink_loss = g.auth_rc_fallback != 0;
    policy.mavlink_manual_takeover_on_rc_loss = g.auth_mav_fallback != 0;
    switch (g.auth_target_loss.get()) {
    case 1:
        policy.target_link_loss_action = MiniDP_TargetLinkLossAction::HOLD_TARGET;
        break;
    case 2:
        policy.target_link_loss_action = MiniDP_TargetLinkLossAction::FALLBACK_TO_RC;
        break;
    case 0:
    default:
        policy.target_link_loss_action = MiniDP_TargetLinkLossAction::FAILSAFE;
        break;
    }

    authority.set_policy(policy);
    authority.update(authority_status);
    rc_channels.set_input_valid(authority_status.rc_healthy);

    if (authority.owner() == MiniDP_ControlOwner::FAILSAFE &&
        mode_manager.mode() != MiniDP_Mode::FAILSAFE) {
        (void)mode_manager.request_mode(
            MiniDP_Mode::FAILSAFE,
            MiniDP_ModeReason::FAILSAFE_TRIGGERED,
            get_state());
    }
}

void MiniDP::report_authority_transition()
{
    const MiniDP_AuthorityTransition &transition = authority.last_transition();
    if (transition.sequence == last_authority_transition_sequence) {
        return;
    }
    last_authority_transition_sequence = transition.sequence;

    const char *from = MiniDP_AuthorityArbiter::owner_name(transition.from);
    const char *to = MiniDP_AuthorityArbiter::owner_name(transition.to);
    const char *reason = MiniDP_AuthorityArbiter::reason_name(transition.reason);
    printf("MiniDP owner: %s -> %s reason=%s\n", from, to, reason);

#if HAL_LOGGING_ENABLED
    logger.Write_MessageF("Owner %s->%s (%s)", from, to, reason);
#endif
#if HAL_GCS_ENABLED
    (void)from;
    (void)to;
    (void)reason;
#endif
}

void MiniDP::report_mode_transition()
{
    const MiniDP_ModeTransition &transition = mode_manager.last_transition();
    if (transition.sequence == last_mode_transition_sequence) {
        return;
    }
    last_mode_transition_sequence = transition.sequence;

    const char *from = MiniDP_ModeManager::mode_name(transition.from);
    const char *to = MiniDP_ModeManager::mode_name(transition.to);
    const char *reason = MiniDP_ModeManager::reason_name(transition.reason);
    printf("MiniDP mode: %s -> %s reason=%s\n", from, to, reason);

#if HAL_LOGGING_ENABLED
    logger.Write_MessageF("Mode %s->%s (%s)", from, to, reason);
#endif
#if HAL_GCS_ENABLED
    (void)from;
    (void)to;
    (void)reason;
#endif
}

void MiniDP::report_status()
{
    const MiniDP_State &state = get_state();
    printf(
        "MiniDP state: mode=%s owner=%s IMU=%s compass=%s EKF=%s att=%s yaw=%s "
        "pos=%s vel=%s GPS=%u sats=%u RC=%s MAV=%s origin=%lu resets=%lu "
        "outputs=disabled\n",
        MiniDP_ModeManager::mode_name(get_mode()),
        MiniDP_AuthorityArbiter::owner_name(get_control_owner()),
        state.imu_healthy ? "healthy" : "unhealthy",
        state.compass_healthy ? "healthy" : "unhealthy",
        state.ekf_healthy ? "healthy" : "unhealthy",
        state.attitude_valid ? "valid" : "invalid",
        state.yaw_valid ? "valid" : "invalid",
        state.position_valid ? "valid" : "invalid",
        state.velocity_valid ? "valid" : "invalid",
        unsigned(state.gps_fix_type),
        unsigned(state.gps_num_sats),
        authority_status.rc_healthy ? "healthy" : "missing",
        authority_status.mavlink_healthy ? "healthy" : "missing",
        (unsigned long)state.origin_id,
        (unsigned long)state.reset_counter);
}

void MiniDP::loop()
{
    const uint32_t now_ms = AP_HAL::millis();

    disable_outputs();

    ins.update();
    barometer.update();
    compass.read();
    gps.update();
    ahrs.update(true);
    state_source.update(ahrs, ins, compass, gps);
    if (now_ms - last_notify_ms >= 20U) {
        last_notify_ms = now_ms;
        notify.update();
    }

#if HAL_GCS_ENABLED
    gcs().update_receive();
#endif
    mode_manager.update(get_state());
    update_authority(now_ms);
    report_authority_transition();
    report_mode_transition();
#if HAL_GCS_ENABLED
    if (gcs().num_gcs() > 0) {
        gcs().update_send();
    }
#endif

    if (now_ms - last_status_ms >= 1000U) {
        last_status_ms = now_ms;
#if HAL_GCS_ENABLED
        gcs_backend.send_minidp_heartbeat();
#endif
        report_status();
    }

    hal.scheduler->delay(10);
}

void setup()
{
    minidp.setup();
}

void loop()
{
    minidp.loop();
}

AP_HAL_MAIN();
