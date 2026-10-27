#include "GCS_MAVLink.h"

#include "Vektor2.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

extern const AP_HAL::HAL &hal;

#if HAL_GCS_ENABLED

bool GCS_MAVLINK_Vektor2::send_storage_information()
{
    if (!HAVE_PAYLOAD_SPACE(chan, STORAGE_INFORMATION)) {
        return false;
    }
    const auto card = Vektor2::app.storage.status();
    mavlink_storage_information_t packet{};
    packet.time_boot_ms = AP_HAL::millis();
    packet.storage_id = 1;
    packet.storage_count = card.count;
    packet.status = card.state;
    if (card.state == STORAGE_STATUS_READY) {
        packet.total_capacity = card.total_bytes / (1024.0f * 1024.0f);
        packet.available_capacity = card.free_bytes / (1024.0f * 1024.0f);
        packet.used_capacity = (card.total_bytes - card.free_bytes) / (1024.0f * 1024.0f);
    }
#if AP_FILESYSTEM_FATFS_ENABLED
    packet.type = STORAGE_TYPE_MICROSD;
    strncpy(packet.name, "SD Card", sizeof(packet.name) - 1);
#else
    packet.type = STORAGE_TYPE_OTHER;
    strncpy(packet.name, "Local filesystem", sizeof(packet.name) - 1);
#endif
    // Speeds remain zero: no destructive or blocking card benchmark is run.
    mavlink_msg_storage_information_send_struct(chan, &packet);
    return true;
}

void GCS_MAVLINK_Vektor2::update_storage_information(uint32_t now_ms)
{
    const auto revision = Vektor2::app.storage.status().revision;
    if (!_storage_pending && !is_active()) {
        return;
    }
    if (_storage_pending || revision != _storage_revision ||
        (_storage_period_ms != 0 && now_ms - _storage_last_ms >= _storage_period_ms)) {
        if (send_storage_information()) {
            _storage_pending = false;
            _storage_revision = revision;
            _storage_last_ms = now_ms;
        }
    }
}

void GCS_Vektor2::update_storage_information(uint32_t now_ms)
{
    for (uint8_t n = 0; n < num_gcs(); n++) {
        if (auto* link = chan(n)) {
            link->update_storage_information(now_ms);
        }
    }
}

MAV_RESULT GCS_MAVLINK_Vektor2::handle_command_int_packet(const mavlink_command_int_t &packet,
                                                        const mavlink_message_t &msg)
{
    // Keep storage transport local to this small application. Standard GCS
    // handling still owns parameter/FTP/log protocols and all other commands.
    if (packet.command == MAV_CMD_REQUEST_MESSAGE &&
        is_equal(packet.param1, float(MAVLINK_MSG_ID_STORAGE_INFORMATION))) {
        if (!is_zero(packet.param2) && !is_equal(packet.param2, 1.0f)) {
            return MAV_RESULT_UNSUPPORTED;
        }
        _storage_pending = true;
        return MAV_RESULT_ACCEPTED;
    }
    if (packet.command == MAV_CMD_REQUEST_MESSAGE &&
        is_equal(packet.param1, float(MAVLINK_MSG_ID_MESSAGE_INTERVAL)) &&
        is_equal(packet.param2, float(MAVLINK_MSG_ID_STORAGE_INFORMATION))) {
        auto query = packet;
        query.command = MAV_CMD_GET_MESSAGE_INTERVAL;
        query.param1 = MAVLINK_MSG_ID_STORAGE_INFORMATION;
        return handle_command_int_packet(query, msg);
    }
    if (packet.command == MAV_CMD_REQUEST_STORAGE_INFORMATION) {
        if ((!is_zero(packet.param1) && !is_equal(packet.param1, 1.0f)) ||
            (!is_zero(packet.param2) && !is_equal(packet.param2, 1.0f))) {
            return MAV_RESULT_UNSUPPORTED;
        }
        _storage_pending |= is_equal(packet.param2, 1.0f);
        return MAV_RESULT_ACCEPTED;
    }
    if (packet.command == MAV_CMD_SET_MESSAGE_INTERVAL &&
        is_equal(packet.param1, float(MAVLINK_MSG_ID_STORAGE_INFORMATION))) {
        if (!isfinite(packet.param2) || (packet.param2 < 0 && !is_equal(packet.param2, -1.0f)) ||
            (!is_zero(packet.param3) && !is_equal(packet.param3, 1.0f))) {
            return MAV_RESULT_DENIED;
        }
        // Default is on-change/on-request only. Bound optional storage streams
        // to 0.017..10 Hz, using the cached IO-thread snapshot.
        _storage_period_ms = packet.param2 <= 0 ? 0 :
            uint32_t(constrain_float(packet.param2 * 0.001f, 100, 60000));
        return MAV_RESULT_ACCEPTED;
    }
    if (packet.command == MAV_CMD_GET_MESSAGE_INTERVAL &&
        is_equal(packet.param1, float(MAVLINK_MSG_ID_STORAGE_INFORMATION))) {
        if (txspace() < PAYLOAD_SIZE(chan, MESSAGE_INTERVAL) + PAYLOAD_SIZE(chan, COMMAND_ACK)) {
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
        mavlink_msg_message_interval_send(chan, MAVLINK_MSG_ID_STORAGE_INFORMATION,
                                         _storage_period_ms == 0 ? -1 : int32_t(_storage_period_ms * 1000));
        return MAV_RESULT_ACCEPTED;
    }
#if AP_FILESYSTEM_FORMAT_ENABLED
    if (packet.command == MAV_CMD_STORAGE_FORMAT &&
        is_equal(packet.param1, 1.0f) && is_equal(packet.param2, 1.0f)) {
        if (Vektor2::app.storage.formatting()) {
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
        if (Vektor2::app.logger.log_while_disarmed() || !Vektor2::app.storage.status().recording_stopped ||
            Vektor2::app.logger.in_log_download()) {
            send_text(MAV_SEVERITY_WARNING, "Stop recording: LOG_DISARMED=0 before SD format");
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
        const MAV_RESULT result = GCS_MAVLINK::handle_command_int_packet(packet, msg);
        if (result == MAV_RESULT_IN_PROGRESS) {
            Vektor2::app.storage.format_started();
        }
        return result;
    }
#endif
    return GCS_MAVLINK::handle_command_int_packet(packet, msg);
}

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
