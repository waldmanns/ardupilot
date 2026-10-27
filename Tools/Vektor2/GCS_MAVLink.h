#pragma once

#include <GCS_MAVLink/GCS.h>

#if HAL_GCS_ENABLED

// Minimal ArduPilot MAVLink backend, patterned after AP_Periph rather than a
// flight vehicle. It deliberately exposes no modes, missions, arming state or
// guided behaviour.
class GCS_MAVLINK_Vektor2 : public GCS_MAVLINK {
public:
    using GCS_MAVLINK::GCS_MAVLINK;
    void update_storage_information(uint32_t now_ms);

private:
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet,
                                         const mavlink_message_t &msg) override;
    bool send_storage_information();
    uint32_t _storage_revision = UINT32_MAX;
    uint32_t _storage_last_ms = 0;
    uint32_t _storage_period_ms = 0;
    bool _storage_pending = false;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override
    {
        (void)cmd;
        return false;
    }

    MAV_RESULT handle_preflight_reboot(const mavlink_command_int_t &packet,
                                       const mavlink_message_t &msg) override;

protected:
    bool try_send_message(enum ap_message id) override;

    uint8_t base_mode() const override
    {
        return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    }

    MAV_STATE vehicle_system_status() const override
    {
        return MAV_STATE_STANDBY;
    }

    void send_nav_controller_output() const override {}
    void send_pid_tuning() override {}
    uint8_t send_available_mode(uint8_t index) const override
    {
        (void)index;
        return 0;
    }
};

class GCS_Vektor2 : public GCS {
public:
    using GCS::GCS;
    void update_storage_information(uint32_t now_ms);

protected:
    GCS_MAVLINK_Vektor2 *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override
    {
        return NEW_NOTHROW GCS_MAVLINK_Vektor2(uart);
    }

    // Control/MAVLink run directly in the IMU loop, outside the bounded logger
    // scheduler task. Its per-task budget is not the MAVLink send budget.
    uint16_t min_loop_time_remaining_for_message_send_us() const override
    {
        return 0;
    }

private:
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Vektor2);

    MAV_TYPE frame_type() const override
    {
        return MAV_TYPE_SURFACE_BOAT;
    }

    uint32_t custom_mode() const override
    {
        return 0;
    }
};

#endif // HAL_GCS_ENABLED
