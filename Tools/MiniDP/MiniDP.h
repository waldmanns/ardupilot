#pragma once

#define AP_PARAM_VEHICLE_NAME minidp

#include "Config.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Param/AP_Param.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <SITL/SITL.h>

#include "ActuatorTest.h"
#include "AxisLimiter.h"
#include "Authority.h"
#include "Controller.h"
#include "GCS_MAVLink.h"
#include "Input.h"
#include "Mode.h"
#include "Output.h"
#include "State.h"

class MiniDP_RC_Channel : public RC_Channel {};

class MiniDP_RC_Channels : public RC_Channels {
public:
    RC_Channel *channel(uint8_t channel_number) override
    {
        if (channel_number >= ARRAY_SIZE(obj_channels)) {
            return nullptr;
        }
        return &obj_channels[channel_number];
    }

    void set_input_valid(bool valid) { input_valid = valid; }
    bool has_valid_input() const override { return input_valid; }
    bool in_rc_failsafe() const override { return !input_valid; }

    MiniDP_RC_Channel obj_channels[NUM_RC_CHANNELS];

private:
    bool input_valid = false;
    int8_t flight_mode_channel_number() const override { return -1; }
};

class Parameters {
public:
    enum {
        k_param_format_version = 0,
        k_param_log_bitmask,
        k_param_board_config,
        k_param_serial_manager,
        k_param_ins,
        k_param_compass,
        k_param_gps,
        k_param_logger,
        k_param_notify,
        k_param_rc_channels,
        k_param_servo_channels,
        k_param_gcs_backend,
        k_param_barometer,
        k_param_ahrs,
        k_param_NavEKF2,
        k_param_NavEKF3,
        k_param_auth_mav_manual,
        k_param_auth_mav_target,
        k_param_auth_test,
        k_param_auth_rc_takeover,
        k_param_auth_rc_fallback,
        k_param_auth_mav_fallback,
        k_param_auth_target_loss,
        k_param_auth_rc_timeout,
        k_param_auth_mav_timeout,
        k_param_frame_type,
        k_param_in_rc_surge_channel,
        k_param_in_rc_sway_channel,
        k_param_in_rc_yaw_channel,
        k_param_in_rc_kill_channel,
        k_param_in_rc_kill_pwm,
        k_param_manual_enable,
        k_param_manual_deadband,
        k_param_manual_surge_limit,
        k_param_manual_sway_limit,
        k_param_manual_yaw_limit,
        k_param_manual_mavlink_timeout,
        k_param_axis_deadband,
        k_param_axis_surge_limit,
        k_param_axis_sway_limit,
        k_param_axis_yaw_limit,
        k_param_axis_surge_slew,
        k_param_axis_sway_slew,
        k_param_axis_yaw_slew,
        k_param_dp_yaw_p,
        k_param_dp_yaw_d,
        k_param_dp_position_p,
        k_param_dp_velocity_d,
        k_param_dp_surge_limit,
        k_param_dp_sway_limit,
        k_param_dp_yaw_limit,
        k_param_dp_retarget,
    };

    AP_Int16 format_version;
    AP_Int32 log_bitmask;
    AP_Int8 auth_mav_manual;
    AP_Int8 auth_mav_target;
    AP_Int8 auth_test;
    AP_Int8 auth_rc_takeover;
    AP_Int8 auth_rc_fallback;
    AP_Int8 auth_mav_fallback;
    AP_Int8 auth_target_loss;
    AP_Float auth_rc_timeout;
    AP_Float auth_mav_timeout;
    AP_Int16 frame_type;
    AP_Int8 in_rc_surge_channel;
    AP_Int8 in_rc_sway_channel;
    AP_Int8 in_rc_yaw_channel;
    AP_Int8 in_rc_kill_channel;
    AP_Int16 in_rc_kill_pwm;
    AP_Int8 manual_enable;
    AP_Float manual_deadband;
    AP_Float manual_surge_limit;
    AP_Float manual_sway_limit;
    AP_Float manual_yaw_limit;
    AP_Float manual_mavlink_timeout;
    AP_Float axis_deadband;
    AP_Float axis_surge_limit;
    AP_Float axis_sway_limit;
    AP_Float axis_yaw_limit;
    AP_Float axis_surge_slew;
    AP_Float axis_sway_slew;
    AP_Float axis_yaw_slew;
    AP_Float dp_yaw_p;
    AP_Float dp_yaw_d;
    AP_Float dp_position_p;
    AP_Float dp_velocity_d;
    AP_Float dp_surge_limit;
    AP_Float dp_sway_limit;
    AP_Float dp_yaw_limit;
    AP_Int8 dp_retarget;
};

struct MiniDP_ModeCommandResult {
    bool accepted;
    MiniDP_ModeReject mode_rejection;
    MiniDP_AuthorityReject authority_rejection;
};

class MiniDP {
public:
    static constexpr uint16_t k_format_version = 1;
    static constexpr uint8_t output_count = 6;

    void setup();
    void loop();

    const MiniDP_State &get_state() const { return state_source.get(); }
    MiniDP_Mode get_mode() const { return mode_manager.mode(); }
    const MiniDP_ModeTarget &get_mode_target() const { return mode_manager.target(); }
    MiniDP_ControlOwner get_control_owner() const { return authority.owner(); }
    MiniDP_ActuatorTestStartResult request_actuator_test(
        const MiniDP_ActuatorTestRequest &request);
    MiniDP_ModeCommandResult request_mode(
        MiniDP_Mode requested,
        MiniDP_ModeReason reason);
    void record_mavlink_manual_control(
        uint32_t now_ms,
        int16_t x,
        int16_t y,
        int16_t r);

    Parameters g;

    AP_BoardConfig board_config;
    AP_SerialManager serial_manager;
    AP_InertialSensor ins;
    Compass compass;
    AP_GPS gps;
    AP_Baro barometer;
    AP_AHRS ahrs{AP_AHRS::FLAG_ALWAYS_USE_EKF};
    AP_Logger logger;
    AP_Notify notify;
    MiniDP_RC_Channels rc_channels;
    SRV_Channels servo_channels;
#if HAL_GCS_ENABLED
    GCS_MiniDP gcs_backend;
#endif
    static const AP_Param::Info var_info[];

private:
    AP_Param param_loader{var_info};

    static const LogStructure log_structure[];

    uint32_t last_notify_ms = 0;
    uint32_t last_status_ms = 0;
    uint32_t last_mode_transition_sequence = 0;
    uint32_t last_authority_transition_sequence = 0;
    uint32_t last_rc_input_ms = 0;
    MiniDP_StateSource state_source;
    MiniDP_ModeManager mode_manager;
    MiniDP_AuthorityArbiter authority;
    MiniDP_OutputManager output_manager;
    MiniDP_InputMapper input_mapper;
    MiniDP_AxisLimiter axis_limiter;
    MiniDP_Controller controller;
    MiniDP_ActuatorTest actuator_test;
    MiniDP_AuthorityStatus authority_status{};
    MiniDP_RCInputFrame rc_input_frame{};
    MiniDP_ManualCommand rc_manual_command{};
    MiniDP_ManualCommand mavlink_manual_command{};
    MiniDP_ManualCommand active_manual_command{};

    void load_parameters();
    MiniDP_InputConfig make_input_config() const;
    MiniDP_AxisLimiterConfig make_axis_limiter_config() const;
    MiniDP_ControllerConfig make_controller_config() const;
    MiniDP_OutputState desired_output_state() const;
    void setup_motor_output_defaults();
    uint32_t motor_output_channel_mask() const;
    void apply_outputs(const MiniDP_OutputFrame &frame);
    void update_authority(uint32_t now_ms);
    void capture_rc_input_frame(bool rc_healthy);
    void update_manual_input(uint32_t now_ms);
    void update_actuator_test(uint32_t now_ms);
    void report_authority_transition();
    void report_mode_transition();
    void report_status();
};

extern MiniDP minidp;

void setup();
void loop();
