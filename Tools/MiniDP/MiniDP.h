#pragma once

#define AP_PARAM_VEHICLE_NAME minidp

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
#include <SITL/SITL.h>

#include "Authority.h"
#include "GCS_MAVLink.h"
#include "Mode.h"
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
    MiniDP_AuthorityStatus authority_status{};

    void load_parameters();
    void disable_outputs();
    void update_authority(uint32_t now_ms);
    void report_authority_transition();
    void report_mode_transition();
    void report_status();
};

extern MiniDP minidp;

void setup();
void loop();
