#pragma once

#include "Config.h"

#include <GCS_MAVLink/GCS.h>

#if HAL_GCS_ENABLED

class GCS_MAVLINK_MiniDP : public GCS_MAVLINK {
public:
    using GCS_MAVLINK::GCS_MAVLINK;

    void send_minidp_text(MAV_SEVERITY severity, const char *text) const;
    bool send_minidp_thruster_status();

protected:
    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;
    void handle_message(const mavlink_message_t &msg) override;
    bool try_send_message(enum ap_message id) override;
    uint64_t capabilities() const override;
    MAV_RESULT handle_command_int_packet(
        const mavlink_command_int_t &packet,
        const mavlink_message_t &msg) override;
    MAV_RESULT handle_preflight_reboot(
        const mavlink_command_int_t &packet,
        const mavlink_message_t &msg) override;
    void handle_manual_control_axes(
        const mavlink_manual_control_t &packet,
        uint32_t tnow) override;

    void send_nav_controller_output() const override;
    void send_position_target_global_int() override;
    void send_position_target_local_ned() override;
    void send_pid_tuning() override {}
    uint8_t send_available_mode(uint8_t index) const override;

private:
    void send_minidp_sys_status() const;
    void send_minidp_firmware_identity() const;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    MAV_RESULT request_dp_target_ne(
        float pos_n_m,
        float pos_e_m,
        bool yaw_valid,
        float yaw_rad,
        const char *accepted_text,
        const char *rejected_prefix);
    MAV_RESULT request_dp_target_location(
        const Location &location,
        bool yaw_valid,
        float yaw_rad,
        const char *accepted_text,
        const char *rejected_prefix);
    bool current_or_target_position(float &pos_n_m, float &pos_e_m) const;
    void handle_set_position_target_local_ned(const mavlink_message_t &msg);
    void handle_set_position_target_global_int(const mavlink_message_t &msg);
    MAV_RESULT handle_mav_cmd_do_motor_test(
        const mavlink_command_int_t &packet);
    MAV_RESULT handle_mav_cmd_do_set_mode(
        const mavlink_command_int_t &packet);
    MAV_RESULT handle_mav_cmd_do_reposition(
        const mavlink_command_int_t &packet);
    MAV_RESULT handle_mav_cmd_component_arm_disarm(
        const mavlink_command_int_t &packet);

    uint8_t thruster_status_next = 0;
};

class GCS_MiniDP : public GCS {
public:
    using GCS::GCS;

    void send_minidp_heartbeat() const;
    void send_minidp_text(MAV_SEVERITY severity, const char *text) const;
    void send_minidp_thruster_status();

protected:
    GCS_MAVLINK_MiniDP *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override
    {
        return NEW_NOTHROW GCS_MAVLINK_MiniDP(uart);
    }

private:
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_MiniDP);

    MAV_TYPE frame_type() const override { return MAV_TYPE_SURFACE_BOAT; }
    uint32_t custom_mode() const override;
};

#endif
