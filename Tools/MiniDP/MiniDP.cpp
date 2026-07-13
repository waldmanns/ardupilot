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
#include <AP_Math/AP_Math.h>
#include <StorageManager/StorageManager.h>
#include <math.h>
#include <stdio.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace {

enum MiniDP_LogMessage : uint8_t {
    LOG_MINIDP_TARGET_MSG,
    LOG_MINIDP_AXIS_MSG,
    LOG_MINIDP_OUTPUT_MSG,
    LOG_MINIDP_STATUS_MSG,
};

static_assert(
    LOG_MINIDP_STATUS_MSG < 32,
    "MiniDP log messages must stay in vehicle-specific ID range");

struct PACKED log_MiniDP_Target {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t mode;
    uint32_t target_id;
    uint16_t flags;
    float target_yaw_rad;
    float yaw_rad;
    float error_n_m;
    float error_e_m;
    float velocity_n_m_s;
    float velocity_e_m_s;
    float gps_hacc_m;
    float gps_sacc_m_s;
};

struct PACKED log_MiniDP_Axis {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t mode;
    uint8_t owner;
    uint8_t output_state;
    float raw_surge;
    float raw_sway;
    float raw_yaw;
    float limited_surge;
    float limited_sway;
    float limited_yaw;
};

struct PACKED log_MiniDP_Output {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t output_state;
    uint8_t saturated;
    float motor1_demand;
    float motor2_demand;
    float motor3_demand;
    float motor4_demand;
    float motor5_demand;
    float motor6_demand;
    uint16_t motor1_pwm_us;
    uint16_t motor2_pwm_us;
    uint16_t motor3_pwm_us;
    uint16_t motor4_pwm_us;
    uint16_t motor5_pwm_us;
    uint16_t motor6_pwm_us;
    uint8_t active_pwm_count;
};

struct PACKED log_MiniDP_Status {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t armed;
    uint8_t soft_armed;
    uint8_t arm_required;
    uint8_t arm_gps_requirement;
    uint8_t last_arm_reject;
    uint8_t mode;
    uint8_t owner;
    uint8_t output_state;
    uint8_t rc_healthy;
    uint8_t mavlink_healthy;
    uint8_t rc_kill;
    uint8_t gps_fix;
    uint8_t gps_sats;
};

constexpr uint32_t minidp_log_attitude = 1U << 0;
constexpr uint32_t minidp_log_gps = 1U << 1;
constexpr uint32_t minidp_log_power = MiniDP::log_power_bit;
constexpr uint32_t minidp_log_imu = 1U << 3;
constexpr uint32_t minidp_log_compass = 1U << 4;
constexpr uint32_t minidp_log_rc = 1U << 5;
constexpr uint32_t minidp_log_dp = 1U << 6;

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

MiniDP_OutputSafeAction safe_action_from_param(
    const int8_t value,
    const MiniDP_OutputSafeAction fallback)
{
    switch (value) {
    case int8_t(MiniDP_OutputSafeAction::DISABLE_PWM):
        return MiniDP_OutputSafeAction::DISABLE_PWM;
    case int8_t(MiniDP_OutputSafeAction::SEND_NEUTRAL):
        return MiniDP_OutputSafeAction::SEND_NEUTRAL;
    case int8_t(MiniDP_OutputSafeAction::SEND_FAILSAFE):
        return MiniDP_OutputSafeAction::SEND_FAILSAFE;
    default:
        return fallback;
    }
}

} // namespace

MiniDP minidp;

#if AP_SIM_ENABLED
SITL::SIM sitl;
#endif
#if AP_EXTERNAL_AHRS_ENABLED
AP_ExternalAHRS external_ahrs;
#endif

constexpr int8_t MiniDP::battery_failsafe_priorities[4];

#define SCHED_TASK(func, rate_hz, _max_time_micros, _priority) SCHED_TASK_CLASS(MiniDP, &minidp, func, rate_hz, _max_time_micros, _priority)

const AP_Scheduler::Task MiniDP::scheduler_tasks[] = {
    SCHED_TASK(read_radio,             50,    200,   3),
    SCHED_TASK(update_ahrs,           400,    400,   6),
    SCHED_TASK(update_current_mode,   400,    250,  12),
    SCHED_TASK(set_servos,            400,    300,  15),
    SCHED_TASK_CLASS(AP_GPS,              &minidp.gps,              update,         50,  300,  18),
#if MINIDP_BARO_ENABLED
    SCHED_TASK_CLASS(AP_Baro,             &minidp.barometer,        update,         10,  200,  21),
#endif
    SCHED_TASK(update_compass,         10,    200,  39),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,    10,    500,  45),
#endif
#if HAL_GCS_ENABLED
    SCHED_TASK_CLASS(GCS,                 (GCS*)&minidp.gcs_backend, update_receive, 400, 500,  51),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&minidp.gcs_backend, update_send,    400, 1000, 54),
    SCHED_TASK_CLASS(GCS_MiniDP,          &minidp.gcs_backend, send_minidp_thruster_status, 4, 300, 55),
#endif
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&minidp.rc_channels, read_mode_switch, 7, 200, 57),
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&minidp.rc_channels, read_aux_all,    10, 200, 60),
    SCHED_TASK_CLASS(AP_Notify,           &minidp.notify,           update,         50,  300,  78),
    SCHED_TASK(one_hz_loop,             1,   1500,  96),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Logger,           &minidp.logger,           periodic_tasks, 50,  300, 108),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,   &minidp.ins,              periodic,      400,  200, 111),
    SCHED_TASK_CLASS(AP_Scheduler,        &minidp.scheduler,        update_logging, 0.1, 200, 114),
};

const LogStructure MiniDP::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_MINIDP_TARGET_MSG, sizeof(log_MiniDP_Target),
      "MDTG", "QBIHffffffff",
      "TimeUS,Mode,TId,Flags,TYaw,Yaw,ErrN,ErrE,VelN,VelE,HAcc,SAcc",
      "s---rrmmnnmn", "F---00000000", true },
    { LOG_MINIDP_AXIS_MSG, sizeof(log_MiniDP_Axis),
      "MDAX", "QBBBffffff",
      "TimeUS,Mode,Own,Out,RSrg,RSw,RYaw,LSrg,LSw,LYaw",
      "s---------", "F---------", true },
    { LOG_MINIDP_OUTPUT_MSG, sizeof(log_MiniDP_Output),
      "MDOT", "QBBffffffHHHHHHB",
      "TimeUS,State,Sat,M1,M2,M3,M4,M5,M6,P1,P2,P3,P4,P5,P6,Act",
      "s--------YYYYYY-", "F---------------", true },
    { LOG_MINIDP_STATUS_MSG, sizeof(log_MiniDP_Status),
      "MDST", "QBBBBBBBBBBBBB",
      "TimeUS,Arm,SArm,Req,GReq,Rej,Mode,Own,Out,RC,Mav,Kill,GPS,Sats",
      "s------------S", "F-------------", true },
};

void MiniDP::load_parameters()
{
    AP_Param::setup_sketch_defaults();
    serial_manager.init_console();
    AP_Param::set_default_by_name("SCHED_LOOP_RATE", 400);
    AP_Param::check_var_info();

    const bool format_version_loaded = g.format_version.load();
    if (!format_version_loaded) {
        g.format_version.set(k_format_version);
        g.format_version.save_sync(true, false);
    } else if (g.format_version != k_format_version) {
        StorageManager::erase();
        AP_Param::erase_all();
        g.format_version.set(k_format_version);
        g.format_version.save_sync(true, false);
    }
    g.format_version.set_default(k_format_version);
    AP_Param::load_all();
    update_azipod_param_visibility();
}

MiniDP_ArmingConfig MiniDP::make_arming_config() const
{
    MiniDP_ArmingConfig config{};
    config.arming_required = g.arm_require != 0;
    switch (g.arm_gps_require.get()) {
    case 1:
        config.gps_requirement = MiniDP_ArmGpsRequirement::FIX;
        break;
    case 2:
        config.gps_requirement = MiniDP_ArmGpsRequirement::DP_READY;
        break;
    case 0:
    default:
        config.gps_requirement = MiniDP_ArmGpsRequirement::NONE;
        break;
    }
    config.gps_hacc_max_m = g.arm_hacc_max.get();
    config.gps_sacc_max_m_s = g.arm_sacc_max.get();
    return config;
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
    config.rc_arm_channel =
        uint8_t(MAX(int16_t(g.in_rc_arm_channel.get()), int16_t(0)));
    config.rc_arm_pwm =
        uint16_t(MAX(int16_t(g.in_rc_arm_pwm.get()), int16_t(0)));
    config.rc_disarm_channel =
        uint8_t(MAX(int16_t(g.in_rc_disarm_channel.get()), int16_t(0)));
    config.rc_disarm_pwm =
        uint16_t(MAX(int16_t(g.in_rc_disarm_pwm.get()), int16_t(0)));
    config.manual_deadband = g.manual_deadband.get();
    config.manual_surge_limit = g.manual_surge_limit.get();
    config.manual_sway_limit = g.manual_sway_limit.get();
    config.manual_yaw_limit = g.manual_yaw_limit.get();
    config.mavlink_manual_timeout_ms =
        uint32_t(MAX(g.manual_mavlink_timeout.get(), 0.0f) * 1000.0f);
    return config;
}

MiniDP_FrameGeometryConfig MiniDP::make_frame_geometry_config() const
{
    MiniDP_FrameGeometryConfig config{};
    switch (g.frame_screw_position.get()) {
    case int8_t(MiniDP_ScrewPosition::FORWARD):
        config.screw_position = MiniDP_ScrewPosition::FORWARD;
        break;
    case int8_t(MiniDP_ScrewPosition::CENTER):
        config.screw_position = MiniDP_ScrewPosition::CENTER;
        break;
    case int8_t(MiniDP_ScrewPosition::AFT):
    default:
        config.screw_position = MiniDP_ScrewPosition::AFT;
        break;
    }
    config.screw_yaw_scale = g.frame_screw_yaw_scale.get();
    return config;
}

void MiniDP::update_azipod_param_visibility()
{
    const AP_Param::GroupInfo *wanted_var_info =
        g.frame_type.get() == int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW) ?
        MiniDP_AzipodParams::var_info :
        nullptr;
    if (azipod_var_info != wanted_var_info) {
        azipod_var_info = wanted_var_info;
        AP_Param::invalidate_count();
    }
}

void MiniDP::sync_azipod_params()
{
    for (uint8_t i = 0; i < 2U; i++) {
        (void)output_manager.set_azipod_config(i, azipod_params_storage.config(i));
    }
}

MiniDP_ModeConfig MiniDP::make_mode_config() const
{
    MiniDP_ModeConfig config{};
    config.dp_hacc_max_m = g.dp_hacc_max.get();
    config.dp_sacc_max_m = g.dp_sacc_max.get();
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
    config.yaw_p = g.dp_yaw_p.get();
    config.yaw_i = g.dp_yaw_i.get();
    config.yaw_d = g.dp_yaw_d.get();
    config.position_p = g.dp_position_p.get();
    config.position_i = g.dp_position_i.get();
    config.velocity_d = g.dp_velocity_d.get();
    config.position_radius_m = g.dp_position_radius.get();
    config.position_deadband_m = g.dp_position_deadband.get();
    config.surge_limit = g.dp_surge_limit.get();
    config.sway_limit = g.dp_sway_limit.get();
    config.yaw_limit = g.dp_yaw_limit.get();
    config.position_i_limit = g.dp_position_imax.get();
    config.yaw_i_limit = g.dp_yaw_imax.get();
    return config;
}

bool MiniDP::outputs_armed() const
{
    if (!arming.outputs_allowed()) {
        return false;
    }
    return hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}

MiniDP_OutputState MiniDP::desired_output_state() const
{
    if (authority_status.rc_kill) {
        return MiniDP_OutputState::KILL;
    }
    if (!outputs_armed()) {
        return MiniDP_OutputState::DISARMED;
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

void MiniDP::sync_frame_config_from_params()
{
    if (output_manager.frame_type() != g.frame_type.get()) {
        if (!output_manager.set_frame_type(g.frame_type.get())) {
            (void)output_manager.set_frame_type(
                MiniDP_OutputManager::default_frame_type);
        }
    }
    update_azipod_param_visibility();
    output_manager.set_frame_geometry(make_frame_geometry_config());
    sync_azipod_params();
}

void MiniDP::sync_output_config_from_servo_params()
{
    SRV_Channels::update_aux_servo_function();

    const MiniDP_OutputSafeAction disarmed_action =
        safe_action_from_param(
            g.out_disarmed_action.get(),
            MiniDP_OutputSafeAction::DISABLE_PWM);
    const MiniDP_OutputSafeAction failsafe_action =
        safe_action_from_param(
            g.out_failsafe_action.get(),
            MiniDP_OutputSafeAction::SEND_NEUTRAL);
    const MiniDP_OutputSafeAction kill_action =
        safe_action_from_param(
            g.out_kill_action.get(),
            MiniDP_OutputSafeAction::DISABLE_PWM);
    const uint16_t configured_failsafe_pwm =
        uint16_t(MAX(int16_t(g.out_failsafe_pwm.get()), int16_t(0)));

    for (uint8_t i = 0; i < MiniDP_OutputManager::max_actuators; i++) {
        MiniDP_ActuatorConfig config = output_manager.actuator_config(i);
        if (!config.enabled) {
            continue;
        }

        config.disarmed_action = disarmed_action;
        config.failsafe_action = failsafe_action;
        config.kill_action = kill_action;
        config.pwm_failsafe = configured_failsafe_pwm;

        const SRV_Channel::Function function =
            SRV_Channels::get_motor_function(i);
        uint8_t channel = 0;
        if (SRV_Channels::find_channel(function, channel)) {
            config.pwm_channel = channel;
        }
        const SRV_Channel *srv_channel =
            SRV_Channels::get_channel_for(function);
        if (srv_channel != nullptr) {
            config.pwm_min = srv_channel->get_output_min();
            config.pwm_trim = srv_channel->get_trim();
            config.pwm_max = srv_channel->get_output_max();
            config.reversed = srv_channel->get_reversed();
            if (configured_failsafe_pwm == 0U) {
                config.pwm_failsafe = config.pwm_trim;
            }
        }

        (void)output_manager.set_actuator_config(i, config);
    }
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

void MiniDP::update_battery(const uint32_t now_ms)
{
#if AP_BATTERY_ENABLED
    if (now_ms - last_battery_read_ms < 100U) {
        return;
    }
    last_battery_read_ms = now_ms;

    battery.read();
    if (!battery.has_failsafed()) {
        battery_failsafe_latched = false;
        last_battery_failsafe_action = 0;
    }
#else
    (void)now_ms;
#endif
}

void MiniDP::setup()
{
    printf("MiniDP basic firmware shell\n");

    load_parameters();
    printf("MiniDP parameters ready\n");
    scheduler.init(scheduler_tasks, ARRAY_SIZE(scheduler_tasks), uint32_t(-1));
    printf("MiniDP scheduler ready\n");
#if HAL_CANMANAGER_ENABLED
    can_mgr.init();
    printf("MiniDP CAN manager ready\n");
#endif
#if HAL_GCS_ENABLED
    gcs().init();
#endif
    serial_manager.init();
    printf("MiniDP serial manager ready\n");
#if HAL_GCS_ENABLED
    gcs().setup_console();
#endif
    board_config.init();
    printf("MiniDP board config ready\n");
#if HAL_GCS_ENABLED
    gcs().setup_uarts();
#endif
#if AP_BATTERY_ENABLED
    battery.init();
    printf("MiniDP battery monitor ready\n");
#endif
#if AP_RSSI_ENABLED
    rssi.init();
    printf("MiniDP RSSI frontend ready\n");
#endif
    rc_channels.init();
    printf("MiniDP RC frontend ready\n");
    input_mapper.init();
    input_mapper.set_config(make_input_config());
    axis_limiter.init(AP_HAL::millis());
    axis_limiter.set_config(make_axis_limiter_config());
    controller.init();
    controller.set_config(make_controller_config());
    arming.init();
    arming.set_config(make_arming_config());
    sync_soft_armed();

    output_manager.init(g.frame_type.get());
    sync_frame_config_from_params();
    setup_motor_output_defaults();
    servo_channels.init(motor_output_channel_mask());
    sync_output_config_from_servo_params();
    (void)output_manager.update(MiniDP_OutputState::DISARMED, {});
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

    ins.init(scheduler.get_loop_rate_hz());
    printf("MiniDP IMU frontend ready\n");
#if MINIDP_BARO_ENABLED
    barometer.init();
    if (barometer.num_instances() > 0) {
        barometer.calibrate(false);
    }
    printf("MiniDP barometer frontend ready\n");
#else
    printf("MiniDP barometer frontend disabled\n");
#endif
    compass.init();
    printf("MiniDP compass frontend ready\n");
#if HAL_LOGGING_ENABLED
    gps.set_log_gps_bit(minidp_log_gps);
    ins.set_log_raw_bit(minidp_log_imu);
#endif
    gps.init();
    printf("MiniDP GPS frontend ready\n");
    ahrs.init();
    ahrs.set_fly_forward(false);
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::GROUND);
    ahrs.reset();
    printf("MiniDP AHRS frontend ready\n");
#if HAL_LOGGING_ENABLED
    write_startup_log_messages();
#endif
    mode_manager.init(AP_HAL::micros64());
    mode_manager.set_config(make_mode_config());
    authority.init(AP_HAL::micros64());

    notify.init();
    AP_Notify::flags.initialising = false;

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

void MiniDP::read_radio()
{
    if (hal.rcin == nullptr) {
        return;
    }

    if (rc_channels.read_input()) {
        last_rc_input_ms = AP_HAL::millis();
    }
}

void MiniDP::update_authority(const uint32_t now_ms)
{
    input_mapper.set_config(make_input_config());

    const bool have_rc_driver = hal.rcin != nullptr;
    const bool have_rc_backend =
        have_rc_driver &&
        RC_Channels::get_valid_channel_count() > 0;

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
    rc_channels.set_input_valid(authority_status.rc_healthy);
    authority_status.mavlink_healthy =
        mavlink_last_seen_ms != 0 &&
        now_ms - mavlink_last_seen_ms <= mavlink_timeout_ms;
    authority_status.mavlink_manual_authorized = g.auth_mav_manual != 0;
    authority_status.mavlink_target_authorized = g.auth_mav_target != 0;
    authority_status.actuator_test_authorized = g.auth_test != 0;
    authority_status.failsafe_active =
        battery_failsafe_latched ||
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

void MiniDP::record_mavlink_radio_rc_channels(
    const uint8_t sysid,
    const uint8_t compid,
    const uint8_t count,
    const uint8_t flags,
    const int16_t channel1_pwm)
{
    radio_rc_packet_count++;
    last_radio_rc_ms = AP_HAL::millis();
    last_radio_rc_sysid = sysid;
    last_radio_rc_compid = compid;
    last_radio_rc_count = count;
    last_radio_rc_flags = flags;
    last_radio_rc_channel1_pwm = channel1_pwm;
}

void MiniDP::record_mavlink_rc_override(
    const uint8_t sysid,
    const uint8_t compid,
    const uint8_t count,
    const uint16_t channel1_pwm)
{
    rc_override_packet_count++;
    last_rc_override_ms = AP_HAL::millis();
    last_rc_override_sysid = sysid;
    last_rc_override_compid = compid;
    last_rc_override_count = count;
    last_rc_override_channel1_pwm = channel1_pwm;
}

void MiniDP::sync_soft_armed()
{
    const bool soft_armed = outputs_armed();
    hal.util->set_soft_armed(soft_armed);
    AP_Notify::flags.armed = arming.armed();
    AP_Notify::flags.flying = soft_armed;
#if HAL_LOGGING_ENABLED
    logger.set_vehicle_armed(soft_armed);
#endif
}

MiniDP_ArmResult MiniDP::request_arm(const bool arm, const bool force)
{
    arming.set_config(make_arming_config());

    MiniDP_ArmResult result{};
    if (arm) {
        const bool failsafe =
            mode_manager.mode() == MiniDP_Mode::FAILSAFE ||
            authority.owner() == MiniDP_ControlOwner::FAILSAFE;
        result = arming.arm(
            get_state(),
            authority_status.rc_kill,
            failsafe,
            force);
    } else {
        result = arming.disarm();
        if (authority.owner() != MiniDP_ControlOwner::FAILSAFE) {
            (void)authority.request_owner(
                MiniDP_ControlOwner::NONE,
                MiniDP_AuthorityReason::RELEASED,
                authority_status);
        }
        if (mode_manager.mode() != MiniDP_Mode::FAILSAFE &&
            mode_manager.mode() != MiniDP_Mode::MANUAL) {
            (void)mode_manager.manual_override(get_state());
        }
    }

    sync_soft_armed();

#if HAL_LOGGING_ENABLED
    if (result.accepted) {
        logger.Write_MessageF(
            "%s%s",
            arm ? "Armed" : "Disarmed",
            force ? " force" : "");
    } else {
        logger.Write_MessageF(
            "Arm rejected: %s",
            MiniDP_Arming::reject_name(result.rejection));
    }
#endif

    return result;
}

void MiniDP::update_rc_arm_switches()
{
    if (!rc_input_frame.healthy) {
        last_rc_arm_switch = false;
        last_rc_disarm_switch = false;
        rc_arm_switches_initialised = false;
        return;
    }

    const bool arm_switch = input_mapper.rc_arm_active(rc_input_frame);
    const bool disarm_switch = input_mapper.rc_disarm_active(rc_input_frame);

    if (!rc_arm_switches_initialised) {
        last_rc_arm_switch = arm_switch;
        last_rc_disarm_switch = disarm_switch;
        rc_arm_switches_initialised = true;
        return;
    }

    if (disarm_switch && !last_rc_disarm_switch) {
        const MiniDP_ArmResult result = request_arm(false, false);
#if HAL_GCS_ENABLED
        gcs_backend.send_minidp_text(
            result.accepted ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
            result.accepted ? "RC disarm accepted" : "RC disarm rejected");
#endif
    } else if (arm_switch && !last_rc_arm_switch && !disarm_switch) {
        const MiniDP_ArmResult result = request_arm(true, false);
#if HAL_GCS_ENABLED
        if (result.accepted) {
            gcs_backend.send_minidp_text(
                MAV_SEVERITY_INFO,
                "RC arm accepted");
        } else {
            char text[50];
            AP_HAL::get_HAL().util->snprintf(
                text,
                sizeof(text),
                "RC arm rejected: %s",
                MiniDP_Arming::reject_name(result.rejection));
            gcs_backend.send_minidp_text(MAV_SEVERITY_WARNING, text);
        }
#endif
    }

    last_rc_arm_switch = arm_switch;
    last_rc_disarm_switch = disarm_switch;
}

void MiniDP::handle_battery_failsafe(
    const char *type_str,
    const int8_t action)
{
    last_battery_failsafe_action = action;

    char text[50];
    AP_HAL::get_HAL().util->snprintf(
        text,
        sizeof(text),
        "Battery %s action=%d",
        type_str == nullptr ? "failsafe" : type_str,
        int(action));

#if HAL_LOGGING_ENABLED
    logger.Write_Message(text);
#endif
#if HAL_GCS_ENABLED
    gcs_backend.send_minidp_text(MAV_SEVERITY_WARNING, text);
#endif

    if (action <= 0) {
        return;
    }

    battery_failsafe_latched = true;
    authority_status.failsafe_active = true;
    authority_status.time_us = AP_HAL::micros64();
    (void)authority.request_owner(
        MiniDP_ControlOwner::FAILSAFE,
        MiniDP_AuthorityReason::FAILSAFE_TRIGGERED,
        authority_status);
    (void)mode_manager.request_mode(
        MiniDP_Mode::FAILSAFE,
        MiniDP_ModeReason::FAILSAFE_TRIGGERED,
        get_state());

    if (action >= 2) {
        (void)request_arm(false, false);
    }
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
        battery_failsafe_latched ||
        mode_manager.mode() == MiniDP_Mode::FAILSAFE;
    mode_manager.set_config(make_mode_config());

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

        const bool force_target_update =
            requested == MiniDP_Mode::DP_HOLD &&
            g.dp_retarget != 0;
        const MiniDP_ModeRequestResult mode_result =
            mode_manager.request_mode(
                requested,
                reason,
                get_state(),
                false,
                force_target_update);
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

MiniDP_ModeCommandResult MiniDP::request_dp_target(
    const float pos_n_m,
    const float pos_e_m,
    const bool yaw_valid,
    const float yaw_rad,
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
        battery_failsafe_latched ||
        mode_manager.mode() == MiniDP_Mode::FAILSAFE;
    mode_manager.set_config(make_mode_config());

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

    const MiniDP_State &state = get_state();
    MiniDP_ModeTarget target{};
    target.position_valid = true;
    target.pos_n_m = pos_n_m;
    target.pos_e_m = pos_e_m;
    target.origin_id = state.origin_id;
    target.reset_counter = state.reset_counter;

    const MiniDP_ModeTarget &current_target = mode_manager.target();
    if (yaw_valid && isfinite(yaw_rad)) {
        target.yaw_valid = true;
        target.yaw_rad = yaw_rad;
    } else if (current_target.yaw_valid &&
               current_target.reset_counter == state.reset_counter) {
        target.yaw_valid = true;
        target.yaw_rad = current_target.yaw_rad;
    } else if (state.yaw_valid) {
        target.yaw_valid = true;
        target.yaw_rad = state.yaw_rad;
    }

    const MiniDP_ModeRequestResult mode_result =
        mode_manager.request_dp_target(target, reason, state);
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
        battery_failsafe_latched ||
        mode_manager.mode() == MiniDP_Mode::FAILSAFE;

    if (!authority_status.actuator_test_authorized) {
        return {false, MiniDP_ActuatorTestReject::NOT_AUTHORIZED};
    }
    if (!outputs_armed()) {
        return {false, MiniDP_ActuatorTestReject::NOT_ARMED};
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
        outputs_armed() &&
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
    char text[50];
    AP_HAL::get_HAL().util->snprintf(
        text,
        sizeof(text),
        "Owner %s->%s (%s)",
        from,
        to,
        reason);
    gcs_backend.send_minidp_text(MAV_SEVERITY_INFO, text);
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
    char text[50];
    AP_HAL::get_HAL().util->snprintf(
        text,
        sizeof(text),
        "Mode %s->%s (%s)",
        from,
        to,
        reason);
    gcs_backend.send_minidp_text(MAV_SEVERITY_INFO, text);
#endif
}

void MiniDP::report_status()
{
    const MiniDP_State &state = get_state();
    printf(
        "MiniDP state: armed=%s soft=%s arm_gps=%s mode=%s owner=%s "
        "IMU=%s compass=%s EKF=%s att=%s yaw=%s "
        "pos=%s vel=%s GPS=%u sats=%u RC=%s MAV=%s origin=%lu resets=%lu "
        "frame=%s screw=%s batt_fs=%d outputs=%s pwm=%u\n",
        arming.armed() ? "yes" : "no",
        outputs_armed() ? "yes" : "no",
        MiniDP_Arming::gps_requirement_name(arming.config().gps_requirement),
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
        MiniDP_OutputManager::screw_position_name(
            make_frame_geometry_config().screw_position),
        int(last_battery_failsafe_action),
        MiniDP_OutputManager::state_name(output_manager.frame().state),
        unsigned(output_manager.frame().active_pwm_count));

    send_named_status_values();
}

void MiniDP::send_named_status_values()
{
#if HAL_GCS_ENABLED
    if (gcs().num_gcs() == 0) {
        return;
    }

    const MiniDP_State &state = get_state();
    const MiniDP_ModeTarget &target = mode_manager.target();
    const MiniDP_OutputFrame &frame = output_manager.frame();
    const MiniDP_ModeConfig mode_config = make_mode_config();
    const bool gps_hacc_bad =
        mode_config.dp_hacc_max_m > 0.0f &&
        (!isfinite(state.gps_hacc_m) ||
         state.gps_hacc_m > mode_config.dp_hacc_max_m);
    const bool gps_sacc_bad =
        mode_config.dp_sacc_max_m > 0.0f &&
        (!isfinite(state.gps_sacc_m) ||
         state.gps_sacc_m > mode_config.dp_sacc_max_m);
    const bool dp_ready =
        state.ekf_healthy &&
        state.yaw_valid &&
        state.origin_valid &&
        state.position_valid &&
        state.velocity_valid &&
        !gps_hacc_bad &&
        !gps_sacc_bad;
    const MiniDP_ModeTransition &transition = mode_manager.last_transition();
    const bool dp_fallback =
        mode_manager.mode() == MiniDP_Mode::HEADING_HOLD &&
        transition.from == MiniDP_Mode::DP_HOLD;

    gcs().send_named_float("DP_MODE", float(uint8_t(mode_manager.mode())));
    gcs().send_named_float("DP_OWN", float(uint8_t(authority.owner())));
    gcs().send_named_float("DP_READY", dp_ready ? 1.0f : 0.0f);
    gcs().send_named_float(
        "DP_HOLD",
        mode_manager.mode() == MiniDP_Mode::DP_HOLD ? 1.0f : 0.0f);
    gcs().send_named_float("DP_FALLB", dp_fallback ? 1.0f : 0.0f);
    gcs().send_named_float(
        "DP_EKF_BAD",
        state.ekf_healthy ? 0.0f : 1.0f);
    gcs().send_named_float(
        "DP_GPS_BAD",
        (gps_hacc_bad || gps_sacc_bad) ? 1.0f : 0.0f);
    gcs().send_named_float("DP_TGT_ID", float(target.target_id));
    gcs().send_named_float("DP_SAT", frame.saturated ? 1.0f : 0.0f);
    gcs().send_named_float("DP_OUT", float(frame.active_pwm_count));
    gcs().send_named_float("MAV_CH", float(gcs().num_gcs()));
    gcs().send_named_float("GPS_FIX", float(state.gps_fix_type));
    gcs().send_named_float("GPS_SATS", float(state.gps_num_sats));
    gcs().send_named_float(
        "RC_OK",
        authority_status.rc_healthy ? 1.0f : 0.0f);
    gcs().send_named_float(
        "RC_CH",
        hal.rcin != nullptr ? float(RC_Channels::get_valid_channel_count()) : 0.0f);
#if AP_RCPROTOCOL_ENABLED
    gcs().send_named_float(
        "RC_UART",
        AP::RC().has_uart() ? 1.0f : 0.0f);
    gcs().send_named_float(
        "RC_PROT",
        float(uint8_t(AP::RC().protocol_detected())));
#endif
    if (last_rc_input_ms != 0) {
        gcs().send_named_float(
            "RC_AGE",
            float(AP_HAL::millis() - last_rc_input_ms) * 0.001f);
    }
    gcs().send_named_float("RRC_CNT", float(radio_rc_packet_count));
    if (last_radio_rc_ms != 0) {
        gcs().send_named_float(
            "RRC_AGE",
            float(AP_HAL::millis() - last_radio_rc_ms) * 0.001f);
        gcs().send_named_float("RRC_CH", float(last_radio_rc_count));
        gcs().send_named_float("RRC_FLG", float(last_radio_rc_flags));
        gcs().send_named_float("RRC_C1", float(last_radio_rc_channel1_pwm));
        gcs().send_named_float("RRC_SYS", float(last_radio_rc_sysid));
    }
    gcs().send_named_float("RCO_CNT", float(rc_override_packet_count));
    if (last_rc_override_ms != 0) {
        gcs().send_named_float(
            "RCO_AGE",
            float(AP_HAL::millis() - last_rc_override_ms) * 0.001f);
        gcs().send_named_float("RCO_CH", float(last_rc_override_count));
        gcs().send_named_float("RCO_C1", float(last_rc_override_channel1_pwm));
        gcs().send_named_float("RCO_SYS", float(last_rc_override_sysid));
    }

    float rtk_fix = 0.0f;
    if (state.gps_fix_type >= uint8_t(AP_GPS::GPS_OK_FIX_3D_RTK_FIXED)) {
        rtk_fix = 2.0f;
    } else if (state.gps_fix_type >= uint8_t(AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT)) {
        rtk_fix = 1.0f;
    }
    gcs().send_named_float("RTK_FIX", rtk_fix);

    if (target.position_valid &&
        state.position_valid &&
        target.origin_id == state.origin_id) {
        const float err_n = target.pos_n_m - state.pos_n_m;
        const float err_e = target.pos_e_m - state.pos_e_m;
        gcs().send_named_float(
            "DP_ERR_M",
            sqrtf((err_n * err_n) + (err_e * err_e)));
        gcs().send_named_float("DP_TGT_N", target.pos_n_m);
        gcs().send_named_float("DP_TGT_E", target.pos_e_m);
    }

    if (target.yaw_valid && state.yaw_valid) {
        gcs().send_named_float(
            "DP_YERR",
            degrees(wrap_PI(target.yaw_rad - state.yaw_rad)));
    }

    if (isfinite(state.gps_hacc_m)) {
        gcs().send_named_float("DP_HACC", state.gps_hacc_m);
    }
    if (isfinite(state.gps_sacc_m)) {
        gcs().send_named_float("DP_SACC", state.gps_sacc_m);
    }
#endif
}

#if HAL_LOGGING_ENABLED
void MiniDP::write_startup_log_messages()
{
    logger.Write_Message("MiniDP startup logging ready");
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

void MiniDP::log_useful_data(const uint32_t now_ms)
{
    if (now_ms - last_standard_log_ms >= 100U) {
        last_standard_log_ms = now_ms;

        if (logger.should_log(minidp_log_attitude)) {
            const MiniDP_ModeTarget &target = mode_manager.target();
            const float target_yaw_deg = target.yaw_valid ?
                degrees(target.yaw_rad) :
                0.0f;
            const Vector3f targets(0.0f, 0.0f, target_yaw_deg);
            ahrs.Write_Attitude(targets);
            ahrs.Log_Write();
        }

        if (logger.should_log(minidp_log_imu)) {
            ins.Write_IMU();
            ins.Write_Vibration();
        }

        if (logger.should_log(minidp_log_rc)) {
            logger.Write_RCIN();
            logger.Write_RCOUT();
#if AP_RSSI_ENABLED
            if (rssi.enabled()) {
                logger.Write_RSSI();
            }
#endif
        }
    }

    if (now_ms - last_slow_log_ms >= 1000U) {
        last_slow_log_ms = now_ms;

        if (logger.should_log(minidp_log_power)) {
            logger.Write_Power();
        }
        if (logger.should_log(minidp_log_compass)) {
            logger.Write_Compass();
        }

        if (logger.should_log(minidp_log_dp)) {
            const MiniDP_OutputFrame &frame = output_manager.frame();
            const struct log_MiniDP_Status status_pkt = {
                LOG_PACKET_HEADER_INIT(LOG_MINIDP_STATUS_MSG),
                time_us             : AP_HAL::micros64(),
                armed               : arming.armed() ? uint8_t(1) : uint8_t(0),
                soft_armed          : outputs_armed() ? uint8_t(1) : uint8_t(0),
                arm_required        : arming.config().arming_required ? uint8_t(1) : uint8_t(0),
                arm_gps_requirement : uint8_t(arming.config().gps_requirement),
                last_arm_reject     : uint8_t(arming.last_rejection()),
                mode                : uint8_t(mode_manager.mode()),
                owner               : uint8_t(authority.owner()),
                output_state        : uint8_t(frame.state),
                rc_healthy          : authority_status.rc_healthy ? uint8_t(1) : uint8_t(0),
                mavlink_healthy     : authority_status.mavlink_healthy ? uint8_t(1) : uint8_t(0),
                rc_kill             : authority_status.rc_kill ? uint8_t(1) : uint8_t(0),
                gps_fix             : get_state().gps_fix_type,
                gps_sats            : get_state().gps_num_sats,
            };
            logger.WriteBlock(&status_pkt, sizeof(status_pkt));
        }
    }
}

void MiniDP::log_control_frame(
    const uint32_t now_ms,
    const MiniDP_AxisCommand &raw_command,
    const MiniDP_AxisCommand &limited_command,
    const MiniDP_OutputFrame &frame)
{
    if (now_ms - last_control_log_ms < 100U) {
        return;
    }
    if (!logger.should_log(minidp_log_dp)) {
        return;
    }
    last_control_log_ms = now_ms;

    const MiniDP_State &state = get_state();
    const MiniDP_ModeTarget &target = mode_manager.target();
    const float nan = logger.quiet_nanf();
    const bool position_error_valid =
        target.position_valid &&
        state.position_valid &&
        target.origin_id == state.origin_id;

    uint16_t flags = 0;
    if (target.yaw_valid) {
        flags |= 1U << 0;
    }
    if (target.position_valid) {
        flags |= 1U << 1;
    }
    if (state.yaw_valid) {
        flags |= 1U << 2;
    }
    if (state.position_valid) {
        flags |= 1U << 3;
    }
    if (state.velocity_valid) {
        flags |= 1U << 4;
    }
    if (state.origin_valid) {
        flags |= 1U << 5;
    }
    if (state.ekf_healthy) {
        flags |= 1U << 6;
    }

    const struct log_MiniDP_Target target_pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MINIDP_TARGET_MSG),
        time_us         : AP_HAL::micros64(),
        mode            : uint8_t(mode_manager.mode()),
        target_id       : target.target_id,
        flags           : flags,
        target_yaw_rad  : target.yaw_valid ? target.yaw_rad : nan,
        yaw_rad         : state.yaw_valid ? state.yaw_rad : nan,
        error_n_m       : position_error_valid ?
            target.pos_n_m - state.pos_n_m :
            nan,
        error_e_m       : position_error_valid ?
            target.pos_e_m - state.pos_e_m :
            nan,
        velocity_n_m_s  : state.velocity_valid ? state.vel_n_m_s : nan,
        velocity_e_m_s  : state.velocity_valid ? state.vel_e_m_s : nan,
        gps_hacc_m      : state.gps_hacc_m,
        gps_sacc_m_s    : state.gps_sacc_m,
    };
    logger.WriteBlock(&target_pkt, sizeof(target_pkt));

    const struct log_MiniDP_Axis axis_pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MINIDP_AXIS_MSG),
        time_us         : AP_HAL::micros64(),
        mode            : uint8_t(mode_manager.mode()),
        owner           : uint8_t(authority.owner()),
        output_state    : uint8_t(frame.state),
        raw_surge       : raw_command.surge,
        raw_sway        : raw_command.sway,
        raw_yaw         : raw_command.yaw,
        limited_surge   : limited_command.surge,
        limited_sway    : limited_command.sway,
        limited_yaw     : limited_command.yaw,
    };
    logger.WriteBlock(&axis_pkt, sizeof(axis_pkt));

    const MiniDP_ActuatorOutput &motor1 = frame.actuator[0];
    const MiniDP_ActuatorOutput &motor2 = frame.actuator[1];
    const MiniDP_ActuatorOutput &motor3 = frame.actuator[2];
    const MiniDP_ActuatorOutput &motor4 = frame.actuator[3];
    const MiniDP_ActuatorOutput &motor5 = frame.actuator[4];
    const MiniDP_ActuatorOutput &motor6 = frame.actuator[5];
    const struct log_MiniDP_Output output_pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MINIDP_OUTPUT_MSG),
        time_us             : AP_HAL::micros64(),
        output_state        : uint8_t(frame.state),
        saturated           : frame.saturated ? uint8_t(1) : uint8_t(0),
        motor1_demand       : motor1.configured ? motor1.demand : nan,
        motor2_demand       : motor2.configured ? motor2.demand : nan,
        motor3_demand       : motor3.configured ? motor3.demand : nan,
        motor4_demand       : motor4.configured ? motor4.demand : nan,
        motor5_demand       : motor5.configured ? motor5.demand : nan,
        motor6_demand       : motor6.configured ? motor6.demand : nan,
        motor1_pwm_us       : motor1.pwm_enabled ? motor1.pwm_us : uint16_t(0),
        motor2_pwm_us       : motor2.pwm_enabled ? motor2.pwm_us : uint16_t(0),
        motor3_pwm_us       : motor3.pwm_enabled ? motor3.pwm_us : uint16_t(0),
        motor4_pwm_us       : motor4.pwm_enabled ? motor4.pwm_us : uint16_t(0),
        motor5_pwm_us       : motor5.pwm_enabled ? motor5.pwm_us : uint16_t(0),
        motor6_pwm_us       : motor6.pwm_enabled ? motor6.pwm_us : uint16_t(0),
        active_pwm_count    : frame.active_pwm_count,
    };
    logger.WriteBlock(&output_pkt, sizeof(output_pkt));
}
#endif

void MiniDP::update_ahrs()
{
    ins.update();
    ahrs.update(true);
    state_source.update(ahrs, ins, compass, gps);
}

void MiniDP::update_current_mode()
{
    const uint32_t now_ms = AP_HAL::millis();
    state_source.update(ahrs, ins, compass, gps);
    mode_manager.set_config(make_mode_config());
    mode_manager.update(get_state());
    update_authority(now_ms);
    arming.set_config(make_arming_config());
    update_rc_arm_switches();
    sync_soft_armed();
    update_battery(now_ms);
}

void MiniDP::set_servos()
{
    const uint32_t now_ms = AP_HAL::millis();

    update_actuator_test(now_ms);
    update_manual_input(now_ms);

    sync_frame_config_from_params();
    sync_output_config_from_servo_params();
    const MiniDP_OutputState output_state = desired_output_state();
    MiniDP_AxisCommand output_command{};
    axis_limiter.set_config(make_axis_limiter_config());
    controller.set_config(make_controller_config());
    bool have_active_command = false;
    bool controller_command_active = false;
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
            controller_command_active = true;
        }
    }
    if (have_active_command) {
        if (!controller_command_active) {
            controller.reset();
        }
        output_command = axis_limiter.update(now_ms, active_command);
    } else {
        controller.reset();
        axis_limiter.reset(now_ms);
    }
    const MiniDP_OutputFrame &output_frame =
        output_state == MiniDP_OutputState::ACTUATOR_TEST ?
            output_manager.update_actuator_test(actuator_test.command()) :
            output_manager.update(output_state, output_command);
#if HAL_LOGGING_ENABLED
    log_control_frame(now_ms, active_command, output_command, output_frame);
#endif
    apply_outputs(output_frame);
    report_authority_transition();
    report_mode_transition();
}

void MiniDP::update_compass()
{
    compass.read();
}

#if HAL_LOGGING_ENABLED
void MiniDP::ten_hz_logging_loop()
{
    log_useful_data(AP_HAL::millis());
}
#endif

void MiniDP::one_hz_loop()
{
    last_status_ms = AP_HAL::millis();
    report_status();
}

void MiniDP::loop()
{
    scheduler.loop();
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
