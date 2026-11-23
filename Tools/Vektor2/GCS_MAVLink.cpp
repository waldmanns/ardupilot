#include "GCS_MAVLink.h"

#include "Vektor2.h"
#include <GCS_MAVLink/GCS_FTP.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

extern const AP_HAL::HAL &hal;

#if HAL_GCS_ENABLED

#if defined(VEKTOR2_SEQ_FTP)
namespace Vektor2 {
namespace SequencerFtp {

Path classify(const char* path)
{
    // 8.3 names have no alternate short names. Protect case and path aliases
    // for mutation, but accept a stage write only at the canonical root path.
    const char* leaf = path;
    for (const char* at = path; *at != 0; at++) {
        if (*at == '/' || *at == '\\' || *at == ':') {
            leaf = at + 1;
        }
    }
    size_t length = strlen(leaf);
    while (length > 0 && (leaf[length - 1] == ' ' || leaf[length - 1] == '.')) {
        length--;
    }
    if (length == 11 && strncasecmp(leaf, "VSEQNEW.BIN", length) == 0) {
        return strcmp(path, "VSEQNEW.BIN") == 0 || strcmp(path, "/VSEQNEW.BIN") == 0 ?
            Path::Stage : Path::Protected;
    }
    if (length == 9 && (strncasecmp(leaf, "VSEQ0.BIN", length) == 0 ||
                        strncasecmp(leaf, "VSEQ1.BIN", length) == 0)) {
        return Path::Protected;
    }
    return Path::Ordinary;
}

size_t begin(uint8_t system, uint8_t component)
{
    return app.sequencer.begin_upload({system, component}) ? SequencerDocument::max_bytes : 0;
}

void end(bool complete)
{
    app.sequencer.end_upload(complete);
}
} // namespace SequencerFtp
} // namespace Vektor2
#endif

using SeqBackend = Vektor2::SequencerBackend;
using SeqResult = SeqBackend::Result;
using SeqOperation = SeqBackend::Operation;

static bool is_sequencer_command(uint16_t command)
{
    return command == MAV_CMD_USER_1 || command == MAV_CMD_USER_2;
}

void GCS_MAVLINK_Vektor2::send_sequencer_ack(uint16_t command, uint8_t system, uint8_t component,
                                           SeqResult result, bool pending)
{
    if (!HAVE_PAYLOAD_SPACE(chan, COMMAND_ACK)) {
        return;
    }
    MAV_RESULT ack;
    switch (result) {
    case SeqResult::Success:
        ack = pending ? MAV_RESULT_IN_PROGRESS : MAV_RESULT_ACCEPTED;
        break;
    case SeqResult::Busy:
        ack = MAV_RESULT_TEMPORARILY_REJECTED;
        break;
    case SeqResult::Unsupported:
        ack = MAV_RESULT_UNSUPPORTED;
        break;
    case SeqResult::Storage:
    case SeqResult::Unavailable:
        ack = MAV_RESULT_FAILED;
        break;
    default:
        ack = MAV_RESULT_DENIED;
        break;
    }
    mavlink_msg_command_ack_send(chan, command, ack, pending ? UINT8_MAX : 0, int32_t(result),
                                 system, component);
}

void GCS_MAVLINK_Vektor2::handle_sequencer_command(const mavlink_message_t& msg, uint16_t command,
                                                  uint8_t system, uint8_t component,
                                                  const float (&fields)[7])
{
    if (system != mavlink_system.sysid || component != mavlink_system.compid) {
        return;
    }
    uint16_t args[7];
    for (uint8_t n = 0; n < ARRAY_SIZE(args); n++) {
        if (!isfinite(fields[n]) || fields[n] < 0 || fields[n] > UINT16_MAX ||
            fields[n] > uint16_t(fields[n])) {
            send_sequencer_ack(command, msg.sysid, msg.compid, SeqResult::Invalid, false);
            return;
        }
        args[n] = fields[n];
    }

    SeqBackend::Request request;
    request.sender = {msg.sysid, msg.compid};
    request.link = chan;
    request.revision = uint32_t(args[2]) | uint32_t(args[3]) << 16;
    request.token = args[4];
    request.crc = uint32_t(args[5]) | uint32_t(args[6]) << 16;
    bool valid = request.token != 0 && msg.sysid != 0 && msg.compid != 0;
    if (command == MAV_CMD_USER_1) {
        request.operation = SeqOperation(args[0]);
        const bool lane = args[0] == uint8_t(SeqOperation::LaneStart) ||
            args[0] == uint8_t(SeqOperation::LaneStop);
        valid = valid && args[0] >= 1 && args[0] <= uint8_t(SeqOperation::Discard) &&
            (lane ? args[1] >= 1 && args[1] <= Vektor2::Sequencer::lane_count : args[1] == 0) &&
            request.crc == 0;
        if (valid && lane) {
            request.target = args[1] - 1;
        }
        const bool revision_required = request.operation == SeqOperation::PowerOn ||
            request.operation == SeqOperation::SharedStart || request.operation == SeqOperation::LaneStart;
        valid = valid && (revision_required || request.revision == 0);
    } else {
        request.operation = args[0] == 1 ? SeqOperation::ApplyParameters : SeqOperation::ApplyFile;
        valid = valid && args[1] == 0 && (args[0] == 2 || (args[0] == 1 && request.crc == 0));
    }

    bool pending = false;
    const auto result = valid ? Vektor2::app.sequencer.enqueue(request, pending) : SeqResult::Invalid;
    send_sequencer_ack(command, msg.sysid, msg.compid, result, pending);
}

void GCS_MAVLINK_Vektor2::handle_message(const mavlink_message_t& msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
        mavlink_param_set_t packet;
        mavlink_msg_param_set_decode(&msg, &packet);
        char name[AP_MAX_NAME_SIZE + 1] {};
        memcpy(name, packet.param_id, AP_MAX_NAME_SIZE);
        if (Vektor2::SequencerParameters::owns(name)) {
            if ((packet.target_system != 0 && packet.target_system != mavlink_system.sysid) ||
                (packet.target_component != 0 && packet.target_component != mavlink_system.compid)) {
                return;
            }
            auto& sequencer = Vektor2::app.sequencer;
            if (sequencer.busy() || !Vektor2::SequencerParameters::valid_write(name, packet.param_value)) {
                ap_var_type type;
                if (auto* param = AP_Param::find(name, &type)) {
                    send_text(MAV_SEVERITY_WARNING, "Sequencer parameter denied: %.16s", name);
                    send_parameter_value(name, type, param->cast_to_float(type));
                }
                return;
            }
            sequencer.parameter_changed();
        }
    }
#if AP_MAVLINK_COMMAND_LONG_ENABLED
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t p;
        mavlink_msg_command_long_decode(&msg, &p);
        if (is_sequencer_command(p.command)) {
            const float fields[] = {p.param1, p.param2, p.param3, p.param4, p.param5, p.param6, p.param7};
            handle_sequencer_command(msg, p.command, p.target_system, p.target_component, fields);
            return;
        }
    }
#endif
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        mavlink_command_int_t p;
        mavlink_msg_command_int_decode(&msg, &p);
        if (is_sequencer_command(p.command)) {
            if (p.target_system != mavlink_system.sysid || p.target_component != mavlink_system.compid) {
                return;
            }
            if (p.frame != MAV_FRAME_MISSION || p.current != 0 || p.autocontinue != 0 ||
                p.x < 0 || p.x > UINT16_MAX || p.y < 0 || p.y > UINT16_MAX) {
                send_sequencer_ack(p.command, msg.sysid, msg.compid, SeqResult::Invalid, false);
                return;
            }
            const float fields[] = {p.param1, p.param2, p.param3, p.param4, float(p.x), float(p.y), p.z};
            handle_sequencer_command(msg, p.command, p.target_system, p.target_component, fields);
            return;
        }
    }
    GCS_MAVLINK::handle_message(msg);
}

static constexpr uint8_t snapshot_fields = 5 + 5 * Vektor2::Sequencer::lane_count;

void GCS_MAVLINK_Vektor2::send_sequencer_field(uint8_t index)
{
    char key[11] {};
    uint32_t value = 0;
    switch (index) {
    case 0:
        strcpy(key, "VS_BEGIN");
        value = _seq_serial;
        break;
    case 1:
        strcpy(key, "VS_REV");
        value = _seq_snapshot.revision;
        break;
    case 2:
        strcpy(key, "VS_STATE");
        value = _seq_snapshot.state;
        break;
    case 3:
        strcpy(key, "VS_SHARED");
        value = _seq_snapshot.shared_run;
        break;
    case snapshot_fields - 1:
        strcpy(key, "VS_END");
        value = _seq_serial;
        break;
    default: {
        const uint8_t lane = (index - 4) / 5;
        const uint8_t field = (index - 4) % 5;
        const auto& s = _seq_snapshot.lanes[lane];
        static const char* const suffixes[] = {"OUT", "BLK", "TIME", "EL", "RUN"};
        hal.util->snprintf(key, sizeof(key), "VSL%02u_%s", unsigned(lane + 1), suffixes[field]);
        switch (field) {
        case 0:
            value = uint32_t(s.logical) | (uint32_t(s.pulse) << 16);
            break;
        case 1:
            value = uint32_t(s.block) | (uint32_t(s.position) << 9) | (uint32_t(s.holding) << 15) |
                (uint32_t(s.state) << 16) | (uint32_t(s.valid) << 19) | (uint32_t(s.trigger_valid) << 20) |
                (uint32_t(s.reason) << 21) | (uint32_t(s.binary) << 25);
            break;
        case 2:
            value = uint32_t(s.block_elapsed) | (uint32_t(s.duration) << 16);
            break;
        case 3:
            value = s.elapsed;
            break;
        case 4:
            value = s.run;
            break;
        }
        break;
    }
    }
    mavlink_msg_named_value_int_send(chan, _seq_snapshot.time, key, int32_t(value));
}

void GCS_MAVLINK_Vektor2::update_sequencer(uint32_t now_ms)
{
    auto& seq = Vektor2::app.sequencer;
    for (uint8_t n = 0; n < 4 && HAVE_PAYLOAD_SPACE(chan, COMMAND_ACK); n++) {
        SeqBackend::Ack ack;
        if (!seq.next_ack(chan, ack)) {
            break;
        }
        const auto& r = ack.request;
        const bool apply = r.operation == SeqOperation::ApplyParameters || r.operation == SeqOperation::ApplyFile;
        send_sequencer_ack(apply ? MAV_CMD_USER_2 : MAV_CMD_USER_1, r.sender.system,
                           r.sender.component, ack.result, false);
    }

    _seq_requested |= seq.take_status_request(chan);
    if (!is_active() && !_seq_requested && _seq_field == UINT8_MAX) {
        _seq_budget_ms = now_ms;
        return;
    }
    const uint8_t rate = seq.telemetry_rate();
    const uint32_t revision = seq.status_serial();
    const uint32_t elapsed = now_ms - _seq_last_ms;
    if (_seq_field == UINT8_MAX && (_seq_requested ||
        ((seq.is_active() || _seq_serial != 0) && _seq_observed != revision && elapsed >= 200) ||
        (seq.is_active() && rate > 0 && elapsed >= 1000U / rate))) {
        _seq_snapshot = seq.snapshot(now_ms);
        _seq_field = 0;
        ++_seq_serial;
        _seq_observed = revision;
        _seq_last_ms = now_ms;
        _seq_requested = false;
    }

    // Allow short bursts while preserving link time for parameter, FTP and
    // standard telemetry traffic. Unsent fields retain their original sample.
    const uint32_t step = MIN(now_ms - _seq_budget_ms, 1000U);
    _seq_budget_ms = now_ms;
    _seq_credit = MIN(4000U, _seq_credit + step * get_uart()->bw_in_bytes_per_second() / 2500U);
    const uint16_t field_size = PAYLOAD_SIZE(chan, NAMED_VALUE_INT);
    for (uint8_t n = 0; n < 10 && _seq_field != UINT8_MAX && _seq_credit >= field_size &&
         HAVE_PAYLOAD_SPACE(chan, NAMED_VALUE_INT); n++) {
        send_sequencer_field(_seq_field++);
        _seq_credit -= field_size;
        if (_seq_field == snapshot_fields) {
            _seq_field = UINT8_MAX;
        }
    }
}

void GCS_Vektor2::update_sequencer(uint32_t now_ms)
{
    for (uint8_t n = 0; n < num_gcs(); n++) {
        if (auto* link = chan(n)) {
            link->update_sequencer(now_ms);
        }
    }
}

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
        if (!Vektor2::app.sequencer.begin_maintenance()) {
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
        const MAV_RESULT result = GCS_MAVLINK::handle_command_int_packet(packet, msg);
        if (result == MAV_RESULT_IN_PROGRESS) {
            Vektor2::app.storage.format_started();
        } else {
            Vektor2::app.sequencer.end_maintenance();
        }
        return result;
    }
#endif
    return GCS_MAVLINK::handle_command_int_packet(packet, msg);
}

bool GCS_MAVLINK_Vektor2::try_send_message(enum ap_message id)
{
    if (id == MSG_NEXT_PARAM) {
        const uint32_t discovery = Vektor2::app.sequencer.activation_serial();
        if (discovery != _seq_discovery) {
            _seq_discovery = discovery;
            if (_queued_parameter != nullptr) {
                restart_parameter_list();
            }
        }
    }
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

    if (!Vektor2::app.sequencer.begin_maintenance()) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // The reboot does not return. Acknowledge the request before resetting,
    // as the ArduPilot vehicle handler does.
    mavlink_msg_command_ack_send(get_chan(), packet.command, MAV_RESULT_ACCEPTED,
                                 0, 0, msg.sysid, msg.compid);
    hal.rcout->force_safety_on();
    hal.scheduler->register_delay_callback(nullptr, 5);
    AP_Param::flush();
    // Allow the HAL storage thread to finish the queued parameter writes.
    hal.scheduler->delay(200);
    hal.scheduler->reboot(is_equal(packet.param1, 3.0f));
    return MAV_RESULT_FAILED;
}

#endif // HAL_GCS_ENABLED
