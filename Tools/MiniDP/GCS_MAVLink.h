#pragma once

#include "Config.h"

#include <GCS_MAVLink/GCS.h>

#if HAL_GCS_ENABLED

class GCS_MAVLINK_MiniDP : public GCS_MAVLINK {
public:
    using GCS_MAVLINK::GCS_MAVLINK;

    void send_minidp_text(MAV_SEVERITY severity, const char *text) const;

protected:
    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;
    bool try_send_message(enum ap_message id) override;
    MAV_RESULT handle_command_int_packet(
        const mavlink_command_int_t &packet,
        const mavlink_message_t &msg) override;
    void handle_manual_control_axes(
        const mavlink_manual_control_t &packet,
        uint32_t tnow) override;

    void send_nav_controller_output() const override {}
    void send_pid_tuning() override {}
    uint8_t send_available_mode(uint8_t index) const override;

private:
    void send_minidp_sys_status() const;
    void send_minidp_firmware_identity() const;
    MAV_RESULT handle_mav_cmd_do_motor_test(
        const mavlink_command_int_t &packet);
    MAV_RESULT handle_mav_cmd_do_set_mode(
        const mavlink_command_int_t &packet);
};

class GCS_MiniDP : public GCS {
public:
    using GCS::GCS;

    void send_minidp_heartbeat() const;
    void send_minidp_text(MAV_SEVERITY severity, const char *text) const;

protected:
    uint16_t min_loop_time_remaining_for_message_send_us() const override
    {
        // MiniDP currently runs a manually paced loop without scheduler tasks.
        return 0;
    }

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
