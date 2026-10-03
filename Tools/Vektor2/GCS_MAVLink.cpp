#include "GCS_MAVLink.h"

#include "Vektor2.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

extern const AP_HAL::HAL &hal;

#if HAL_GCS_ENABLED

bool GCS_MAVLINK_Vektor2::try_send_message(enum ap_message id)
{
    // Publish the routed RC snapshot, including standard MAVLink overrides,
    // as the standard RC_CHANNELS message.
    if (id == MSG_RC_CHANNELS) {
        if (!check_payload_size(MAVLINK_MSG_ID_RC_CHANNELS_LEN)) {
            return false;
        }

        uint16_t values[18];
        for (uint8_t i = 0; i < ARRAY_SIZE(values); i++) {
            values[i] = UINT16_MAX;
        }

        const auto& rc = Vektor2::app.rcin;
        uint8_t count = 0;
        if (rc.valid()) {
            count = rc.channel_count();
            if (count > ARRAY_SIZE(values)) {
                count = ARRAY_SIZE(values);
            }
            for (uint8_t i = 0; i < count; i++) {
                values[i] = rc.channel_us(i);
            }
        }

        uint8_t rssi = 255; // unknown
#if AP_RCPROTOCOL_ENABLED
        const int16_t raw_rssi = AP::RC().get_RSSI();
        if (raw_rssi >= 0 && raw_rssi <= 255) {
            rssi = uint8_t(raw_rssi);
        }
#endif

        mavlink_msg_rc_channels_send(
            get_chan(),
            AP_HAL::millis(),
            count,
            values[0], values[1], values[2], values[3],
            values[4], values[5], values[6], values[7],
            values[8], values[9], values[10], values[11],
            values[12], values[13], values[14], values[15],
            values[16], values[17],
            rssi);
        return true;
    }

    // The normal ArduPilot sender goes through SRV_Channels. Vektor2 bypasses
    // that application layer, so publish the values actually written through
    // the tiny PwmOut facade. SERVO_OUTPUT_RAW is standard MAVLink and keeps
    // the units in microseconds. The standard message exposes 16 outputs.
    if (id == MSG_SERVO_OUTPUT_RAW) {
        if (!check_payload_size(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN)) {
            return false;
        }

        const auto& pwm = Vektor2::app.pwm;
        uint16_t values[16] {};
        for (uint8_t i = 0; i < ARRAY_SIZE(values); i++) {
            if (pwm.enabled(i)) {
                values[i] = pwm.value_us(i);
            }
        }

        mavlink_msg_servo_output_raw_send(
            get_chan(),
            AP_HAL::micros(),
            0,
            values[0], values[1], values[2], values[3],
            values[4], values[5], values[6], values[7],
            values[8], values[9], values[10], values[11],
            values[12], values[13], values[14], values[15]);
        return true;
    }

    return GCS_MAVLINK::try_send_message(id);
}

MAV_RESULT GCS_MAVLINK_Vektor2::handle_preflight_reboot(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    // Keep this handler limited to reboot requests. The base implementation
    // also accepts developer fault-injection commands, which Vektor2 does not
    // expose through its MAVLink interface.
    if (!is_equal(packet.param1, 1.0f) && !is_equal(packet.param1, 3.0f)) {
        return MAV_RESULT_UNSUPPORTED;
    }

    // The reboot does not return. Acknowledge the request before resetting,
    // as the ArduPilot vehicle handler does.
    mavlink_msg_command_ack_send(get_chan(), packet.command, MAV_RESULT_ACCEPTED,
                                 0, 0, msg.sysid, msg.compid);
    hal.scheduler->reboot(is_equal(packet.param1, 3.0f));
    return MAV_RESULT_FAILED;
}

#endif // HAL_GCS_ENABLED
