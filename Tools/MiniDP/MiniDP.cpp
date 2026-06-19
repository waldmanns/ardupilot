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

namespace {

void set_channel_enabled_by_mask(const uint32_t mask, const bool enabled)
{
    if (hal.rcout == nullptr) {
        return;
    }

    const uint8_t max_channel = MIN(uint8_t(NUM_SERVO_CHANNELS), uint8_t(32));
    for (uint8_t channel = 0; channel < max_channel; channel++) {
        if ((mask & (1U << channel)) == 0) {
            continue;
        }
        if (enabled) {
            hal.rcout->enable_ch(channel);
        } else {
            hal.rcout->disable_ch(channel);
        }
    }
}

uint32_t auxiliary_output_channel_mask(
    const uint32_t managed_mask,
    const uint32_t disabled_mask)
{
    uint32_t mask = 0;
    const uint8_t max_channel = MIN(uint8_t(NUM_SERVO_CHANNELS), uint8_t(32));
    for (uint8_t channel = 0; channel < max_channel; channel++) {
        const uint32_t channel_mask = 1U << channel;
        if ((managed_mask & channel_mask) ||
            (disabled_mask & channel_mask)) {
            continue;
        }

        const SRV_Channel::Function function =
            SRV_Channels::channel_function(channel);
        if (SRV_Channel::valid_function(function)) {
            mask |= channel_mask;
        }
    }
    return mask;
}

} // namespace

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

MiniDP_InputConfig MiniDP::make_input_config() const
{
    MiniDP_InputConfig config{};
    config.rc_surge_channel =
        uint8_t(MAX(int16_t(g.in_rc_surge_channel.get()), int16_t(0)));
    config.rc_sway_channel =
        uint8_t(MAX(int16_t(g.in_rc_sway_channel.get()), int16_t(0)));
    config.rc_yaw_channel =
        uint8_t(MAX(int16_t(g.in_rc_yaw_channel.get()), int16_t(0)));
    config.rc_kill_channel =
        uint8_t(MAX(int16_t(g.in_rc_kill_channel.get()), int16_t(0)));
    config.rc_kill_pwm =
        uint16_t(MAX(int16_t(g.in_rc_kill_pwm.get()), int16_t(0)));
    config.manual_deadband = g.manual_deadband.get();
    config.manual_surge_limit = g.manual_surge_limit.get();
    config.manual_sway_limit = g.manual_sway_limit.get();
    config.manual_yaw_limit = g.manual_yaw_limit.get();
    config.mavlink_manual_timeout_ms =
        uint32_t(MAX(g.manual_mavlink_timeout.get(), 0.0f) * 1000.0f);
    return config;
}

MiniDP_AxisLimiterConfig MiniDP::make_axis_limiter_config() const
{
    MiniDP_AxisLimiterConfig config{};
    config.deadband = g.axis_deadband.get();
    config.surge_limit = g.axis_surge_limit.get();
    config.sway_limit = g.axis_sway_limit.get();
    config.yaw_limit = g.axis_yaw_limit.get();
    config.surge_slew_rate = g.axis_surge_slew.get();
    config.sway_slew_rate = g.axis_sway_slew.get();
    config.yaw_slew_rate = g.axis_yaw_slew.get();
    return config;
}

MiniDP_ControllerConfig MiniDP::make_controller_config() const
{
    MiniDP_ControllerConfig config{};
    config.yaw_p = g.ctrl_yaw_p.get();
    config.yaw_d = g.ctrl_yaw_d.get();
    config.position_p = g.ctrl_position_p.get();
    config.velocity_d = g.ctrl_velocity_d.get();
    config.surge_limit = g.ctrl_surge_limit.get();
    config.sway_limit = g.ctrl_sway_limit.get();
    config.yaw_limit = g.ctrl_yaw_limit.get();
    return config;
}

MiniDP_OutputState MiniDP::desired_output_state() const
{
    if (authority_status.rc_kill) {
        return MiniDP_OutputState::KILL;
    }
    if (mode_manager.mode() == MiniDP_Mode::FAILSAFE ||
        authority.owner() == MiniDP_ControlOwner::FAILSAFE) {
        return MiniDP_OutputState::FAILSAFE;
    }
    if (mode_manager.mode() == MiniDP_Mode::ACTUATOR_TEST &&
        authority.owner() == MiniDP_ControlOwner::TEST &&
        actuator_test.active()) {
        return MiniDP_OutputState::ACTUATOR_TEST;
    }
    if (mode_manager.mode() == MiniDP_Mode::MANUAL &&
        active_manual_command.valid &&
        ((authority.owner() == MiniDP_ControlOwner::RC &&
          active_manual_command.source == MiniDP_ManualSource::RC) ||
         (authority.owner() == MiniDP_ControlOwner::MAVLINK_MANUAL &&
          active_manual_command.source == MiniDP_ManualSource::MAVLINK))) {
        return MiniDP_OutputState::ARMED_ACTIVE;
    }
    if ((mode_manager.mode() == MiniDP_Mode::HEADING_HOLD ||
         mode_manager.mode() == MiniDP_Mode::DP_HOLD) &&
        authority.owner() == MiniDP_ControlOwner::MAVLINK_TARGET) {
        return MiniDP_OutputState::ARMED_ACTIVE;
    }
    return MiniDP_OutputState::DISARMED;
}

void MiniDP::setup_motor_output_defaults()
{
    for (uint8_t i = 0; i < MiniDP_OutputManager::max_actuators; i++) {
        const MiniDP_ActuatorConfig &config = output_manager.actuator_config(i);
        if (!config.enabled) {
            continue;
        }

        const SRV_Channel::Function function =
            SRV_Channels::get_motor_function(i);
        if (!SRV_Channels::set_aux_channel_default(function, i)) {
            printf(
                "MiniDP %s default output assignment skipped\n",
                MiniDP_OutputManager::motor_name(i));
        }
        SRV_Channels::set_output_min_max_defaults(
            function,
            config.pwm_min,
            config.pwm_max);
    }

    SRV_Channels::update_aux_servo_function();
}

uint32_t MiniDP::motor_output_channel_mask() const
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < MiniDP_OutputManager::max_actuators; i++) {
        if (!output_manager.actuator_config(i).enabled) {
            continue;
        }
        mask |= SRV_Channels::get_output_channel_mask(
            SRV_Channels::get_motor_function(i));
    }
    return mask;
}

void MiniDP::apply_outputs(const MiniDP_OutputFrame &frame)
{
    if (hal.rcout == nullptr) {
        return;
    }

    const uint32_t managed_mask = motor_output_channel_mask();
    const uint32_t saved_disabled_mask =
        SRV_Channels::get_disabled_channel_mask();
    uint32_t active_mask = 0;
    for (uint8_t i = 0; i < MiniDP_OutputManager::max_actuators; i++) {
        const MiniDP_ActuatorOutput &output = frame.actuator[i];
        if (output.pwm_enabled) {
            active_mask |= SRV_Channels::get_output_channel_mask(
                SRV_Channels::get_motor_function(i));
        }
    }

    servo_channels.cork();
    set_channel_enabled_by_mask(
        auxiliary_output_channel_mask(managed_mask, saved_disabled_mask),
        true);
    SRV_Channels::set_disabled_channel_mask(saved_disabled_mask | managed_mask);
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
    SRV_Channels::set_disabled_channel_mask(saved_disabled_mask);

    set_channel_enabled_by_mask(active_mask, true);
    for (uint8_t i = 0; i < MiniDP_OutputManager::max_actuators; i++) {
        const MiniDP_ActuatorOutput &output = frame.actuator[i];
        if (output.pwm_enabled) {
            SRV_Channels::set_output_pwm(
                SRV_Channels::get_motor_function(i),
                output.pwm_us);
        }
    }
    set_channel_enabled_by_mask(managed_mask & ~active_mask, false);
    servo_channels.push();
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
    input_mapper.init();
    input_mapper.set_config(make_input_config());
    axis_limiter.init(AP_HAL::millis());
    axis_limiter.set_config(make_axis_limiter_config());
    controller.init();
    controller.set_config(make_controller_config());

    output_manager.init(g.frame_type.get());
    setup_motor_output_defaults();
    servo_channels.init(motor_output_channel_mask());
    apply_outputs(output_manager.frame());
    printf(
        "MiniDP frame type %d (%s) ready\n",
        int(output_manager.frame_type()),
        MiniDP_OutputManager::frame_type_name(output_manager.frame_type()));

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

    printf("MiniDP initialization complete; outputs safe\n");
}

void MiniDP::capture_rc_input_frame(const bool rc_healthy)
{
    rc_input_frame = {};
    rc_input_frame.healthy = rc_healthy;
    if (!rc_healthy) {
        return;
    }

    uint8_t channel_count = 0;
    if (hal.rcin != nullptr) {
        channel_count = RC_Channels::get_valid_channel_count();
    }
    rc_input_frame.channel_count =
        MIN(channel_count, MiniDP_InputMapper::max_rc_channels);

    for (uint8_t i = 0; i < MiniDP_InputMapper::max_rc_channels; i++) {
        RC_Channel *channel = rc_channels.channel(i);
        if (channel == nullptr) {
            continue;
        }
        rc_input_frame.norm[i] = channel->norm_input_dz();
        rc_input_frame.pwm[i] = channel->get_radio_in();
    }
}

void MiniDP::update_authority(const uint32_t now_ms)
{
    input_mapper.set_config(make_input_config());

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
    capture_rc_input_frame(authority_status.rc_healthy);
    authority_status.rc_kill = input_mapper.rc_kill_active(rc_input_frame);

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

void MiniDP::record_mavlink_manual_control(
    const uint32_t now_ms,
    const int16_t x,
    const int16_t y,
    const int16_t r)
{
    input_mapper.record_mavlink_manual_control(now_ms, x, y, r);
}

MiniDP_ModeCommandResult MiniDP::request_mode(
    const MiniDP_Mode requested,
    const MiniDP_ModeReason reason)
{
    MiniDP_AuthorityStatus request_status = authority_status;
    request_status.time_us = AP_HAL::micros64();
    request_status.mavlink_healthy = true;
    request_status.mavlink_manual_authorized = g.auth_mav_manual != 0;
    request_status.mavlink_target_authorized = g.auth_mav_target != 0;
    request_status.actuator_test_authorized = g.auth_test != 0;
    request_status.rc_kill = input_mapper.rc_kill_active(rc_input_frame);
    request_status.failsafe_active =
        mode_manager.mode() == MiniDP_Mode::FAILSAFE;

    switch (requested) {
    case MiniDP_Mode::MANUAL: {
        const MiniDP_ModeRequestResult mode_result =
            mode_manager.request_mode(requested, reason, get_state());
        if (mode_result.accepted &&
            authority.owner() == MiniDP_ControlOwner::MAVLINK_TARGET) {
            (void)authority.request_owner(
                MiniDP_ControlOwner::NONE,
                MiniDP_AuthorityReason::RELEASED,
                request_status);
        }
        return {
            mode_result.accepted,
            mode_result.rejection,
            MiniDP_AuthorityReject::NONE,
        };
    }

    case MiniDP_Mode::HEADING_HOLD:
    case MiniDP_Mode::DP_HOLD: {
        const MiniDP_ControlOwner previous_owner = authority.owner();
        const MiniDP_AuthorityRequestResult authority_result =
            authority.request_owner(
                MiniDP_ControlOwner::MAVLINK_TARGET,
                MiniDP_AuthorityReason::MAVLINK_TARGET_REQUEST,
                request_status);
        if (!authority_result.accepted) {
            return {
                false,
                MiniDP_ModeReject::NONE,
                authority_result.rejection,
            };
        }

        const MiniDP_ModeRequestResult mode_result =
            mode_manager.request_mode(requested, reason, get_state());
        if (!mode_result.accepted &&
            previous_owner != MiniDP_ControlOwner::MAVLINK_TARGET &&
            authority.owner() == MiniDP_ControlOwner::MAVLINK_TARGET) {
            (void)authority.request_owner(
                MiniDP_ControlOwner::NONE,
                MiniDP_AuthorityReason::RELEASED,
                request_status);
        }
        return {
            mode_result.accepted,
            mode_result.rejection,
            MiniDP_AuthorityReject::NONE,
        };
    }

    case MiniDP_Mode::ACTUATOR_TEST:
    case MiniDP_Mode::FAILSAFE:
        break;
    }

    return {
        false,
        MiniDP_ModeReject::UNSUPPORTED_MODE,
        MiniDP_AuthorityReject::NONE,
    };
}

void MiniDP::update_manual_input(const uint32_t now_ms)
{
    rc_manual_command = input_mapper.map_rc(rc_input_frame, now_ms);
    mavlink_manual_command = input_mapper.mavlink_manual_command(now_ms);
    active_manual_command = {};

    if (g.manual_enable == 0) {
        if (authority.owner() == MiniDP_ControlOwner::RC ||
            authority.owner() == MiniDP_ControlOwner::MAVLINK_MANUAL) {
            (void)authority.request_owner(
                MiniDP_ControlOwner::NONE,
                MiniDP_AuthorityReason::RELEASED,
                authority_status);
        }
        return;
    }

    if (authority_status.rc_kill ||
        mode_manager.mode() == MiniDP_Mode::FAILSAFE ||
        mode_manager.mode() == MiniDP_Mode::ACTUATOR_TEST ||
        authority.owner() == MiniDP_ControlOwner::FAILSAFE ||
        authority.owner() == MiniDP_ControlOwner::TEST ||
        actuator_test.active()) {
        return;
    }

    if (rc_manual_command.valid) {
        const MiniDP_AuthorityRequestResult rc_result =
            authority.request_owner(
                MiniDP_ControlOwner::RC,
                MiniDP_AuthorityReason::RC_REQUEST,
                authority_status);
        if (rc_result.accepted &&
            authority.owner() == MiniDP_ControlOwner::RC) {
            if (mode_manager.mode() != MiniDP_Mode::MANUAL) {
                (void)mode_manager.manual_override(get_state());
            }
            active_manual_command = rc_manual_command;
            return;
        }
    } else if (authority.owner() == MiniDP_ControlOwner::RC) {
        (void)authority.request_owner(
            MiniDP_ControlOwner::NONE,
            MiniDP_AuthorityReason::RELEASED,
            authority_status);
    }

    if (mavlink_manual_command.valid) {
        const MiniDP_AuthorityRequestResult mavlink_result =
            authority.request_owner(
                MiniDP_ControlOwner::MAVLINK_MANUAL,
                MiniDP_AuthorityReason::MAVLINK_MANUAL_REQUEST,
                authority_status);
        if (mavlink_result.accepted &&
            authority.owner() == MiniDP_ControlOwner::MAVLINK_MANUAL) {
            if (mode_manager.mode() != MiniDP_Mode::MANUAL) {
                (void)mode_manager.manual_override(get_state());
            }
            active_manual_command = mavlink_manual_command;
        }
        return;
    }

    if (authority.owner() == MiniDP_ControlOwner::MAVLINK_MANUAL) {
        (void)authority.request_owner(
            MiniDP_ControlOwner::NONE,
            MiniDP_AuthorityReason::MAVLINK_MANUAL_TIMEOUT,
            authority_status);
    }
}

MiniDP_ActuatorTestStartResult MiniDP::request_actuator_test(
    const MiniDP_ActuatorTestRequest &request)
{
    authority_status.time_us = AP_HAL::micros64();
    authority_status.actuator_test_authorized = g.auth_test != 0;
    authority_status.failsafe_active =
        mode_manager.mode() == MiniDP_Mode::FAILSAFE;

    if (!authority_status.actuator_test_authorized) {
        return {false, MiniDP_ActuatorTestReject::NOT_AUTHORIZED};
    }

    const MiniDP_AuthorityRequestResult authority_result =
        authority.request_owner(
            MiniDP_ControlOwner::TEST,
            MiniDP_AuthorityReason::TEST_REQUEST,
            authority_status);
    if (!authority_result.accepted) {
        return {false, MiniDP_ActuatorTestReject::AUTHORITY_REJECTED};
    }

    const MiniDP_ModeRequestResult mode_result =
        mode_manager.request_mode(
            MiniDP_Mode::ACTUATOR_TEST,
            MiniDP_ModeReason::ACTUATOR_TEST_REQUEST,
            get_state(),
            true);
    if (!mode_result.accepted) {
        (void)authority.request_owner(
            MiniDP_ControlOwner::NONE,
            MiniDP_AuthorityReason::RELEASED,
            authority_status);
        return {false, MiniDP_ActuatorTestReject::MODE_REJECTED};
    }

    const MiniDP_ActuatorTestStartResult test_result =
        actuator_test.start(AP_HAL::millis(), request);
    if (!test_result.accepted) {
        (void)authority.request_owner(
            MiniDP_ControlOwner::NONE,
            MiniDP_AuthorityReason::RELEASED,
            authority_status);
        (void)mode_manager.manual_override(get_state());
        return test_result;
    }

#if HAL_LOGGING_ENABLED
    logger.Write_MessageF(
        "Actuator test %u type=%u value=%0.1f timeout=%0.1f",
        unsigned(request.actuator_index + 1U),
        unsigned(request.throttle_type),
        double(request.throttle_value),
        double(request.timeout_s));
#endif

    return test_result;
}

void MiniDP::update_actuator_test(const uint32_t now_ms)
{
    const bool safe_to_continue =
        mode_manager.mode() == MiniDP_Mode::ACTUATOR_TEST &&
        authority.owner() == MiniDP_ControlOwner::TEST &&
        authority_status.actuator_test_authorized &&
        !authority_status.rc_kill &&
        authority.owner() != MiniDP_ControlOwner::FAILSAFE;

    const bool was_active = actuator_test.active();
    actuator_test.update(now_ms, safe_to_continue);

    if ((was_active && !actuator_test.active()) ||
        (mode_manager.mode() == MiniDP_Mode::ACTUATOR_TEST &&
         authority.owner() != MiniDP_ControlOwner::TEST)) {
#if HAL_LOGGING_ENABLED
        logger.Write_MessageF(
            "Actuator test stopped (%s)",
            MiniDP_ActuatorTest::stop_reason_name(
                actuator_test.last_stop_reason()));
#endif
        if (authority.owner() == MiniDP_ControlOwner::TEST) {
            (void)authority.request_owner(
                MiniDP_ControlOwner::NONE,
                MiniDP_AuthorityReason::RELEASED,
                authority_status);
        }
        if (mode_manager.mode() == MiniDP_Mode::ACTUATOR_TEST) {
            (void)mode_manager.manual_override(get_state());
        }
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
        "frame=%s outputs=%s pwm=%u\n",
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
        (unsigned long)state.reset_counter,
        MiniDP_OutputManager::frame_type_name(output_manager.frame_type()),
        MiniDP_OutputManager::state_name(output_manager.frame().state),
        unsigned(output_manager.frame().active_pwm_count));
}

void MiniDP::loop()
{
    const uint32_t now_ms = AP_HAL::millis();

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
    update_actuator_test(now_ms);
    update_manual_input(now_ms);

    const MiniDP_OutputState output_state = desired_output_state();
    MiniDP_AxisCommand output_command{};
    axis_limiter.set_config(make_axis_limiter_config());
    controller.set_config(make_controller_config());
    bool have_active_command = false;
    MiniDP_AxisCommand active_command{};
    if (output_state == MiniDP_OutputState::ARMED_ACTIVE) {
        if (active_manual_command.valid) {
            active_command = active_manual_command.axes;
            have_active_command = true;
        } else if (authority.owner() == MiniDP_ControlOwner::MAVLINK_TARGET) {
            active_command = controller.update(
                mode_manager.mode(),
                mode_manager.target(),
                get_state());
            have_active_command = true;
        }
    }
    if (have_active_command) {
        output_command = axis_limiter.update(now_ms, active_command);
    } else {
        axis_limiter.reset(now_ms);
    }
    const MiniDP_OutputFrame &output_frame =
        output_state == MiniDP_OutputState::ACTUATOR_TEST ?
            output_manager.update_actuator_test(actuator_test.command()) :
            output_manager.update(output_state, output_command);
    apply_outputs(output_frame);
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
