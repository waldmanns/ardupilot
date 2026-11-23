#pragma once

#include "SequencerDocument.h"
#include "SequencerParameters.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

namespace Vektor2 {

class SequencerBackend {
public:
    enum class Config : uint8_t { Loading, Active = 2, Invalid };
    enum class Result : uint8_t { Success, Locked, Busy, Stale, Invalid, Unsupported, Storage, Unavailable, Inhibited };

    enum class Operation : uint8_t {
        PowerOn = 1, PowerOff, SharedStart, SharedStop, LaneStart, LaneStop, StopAll, Status, Discard,
        ApplyParameters, ApplyFile
    };
    struct Sender {
        Sender() = default;
        Sender(uint8_t sysid, uint8_t compid) : system(sysid), component(compid) {}
        uint8_t system = 0;
        uint8_t component = 0;
        bool operator==(const Sender& other) const { return system == other.system && component == other.component; }
    };
    struct Request {
        Sender sender;
        uint32_t revision = 0;
        uint32_t crc = 0;
        uint16_t token = 0;
        Operation operation = Operation::Status;
        uint8_t target = Sequencer::shared_slot;
        mavlink_channel_t link = MAVLINK_COMM_0;
    };
    struct Ack {
        Request request;
        Result result = Result::Success;
    };
    struct Snapshot {
        Sequencer::Status lanes[Sequencer::lane_count];
        uint32_t time = 0;
        uint32_t revision = 0;
        uint32_t state = 0;
        uint32_t shared_run = 0;
    };

    void init();
    void update(uint32_t now_ms);
    void parameter_changed() { _refresh_pending = true; }
    Result enqueue(const Request& request, bool& pending);
    bool next_ack(mavlink_channel_t link, Ack& ack);
    bool take_status_request(mavlink_channel_t link);

    Sequencer engine;
    SequencerParameters params;

    Snapshot snapshot(uint32_t now_ms) const;
    uint32_t status_serial() const;
    uint32_t activation_serial() const { return _activation_serial; }
    uint8_t telemetry_rate() const;
    bool is_active() const { return engine.active(); }

    bool begin_upload(Sender sender);
    void end_upload(bool complete);
    bool begin_maintenance();
    void end_maintenance();
    bool busy() const { return _busy; }

private:
    enum class Upload : uint8_t { Empty, Open, Closed, Reserved };
    struct Client {
        Ack ack;
        uint32_t last_ms = 0;
        bool used = false;
        bool pending = false;
        bool notify = false;
    };
    struct Work {
        bool pending = false;
        uint8_t candidate = 0;
        uint32_t crc = 0;
    };
    struct Completed {
        bool ready = false;
        Result result = Result::Success;
    };

    void io_timer();
    Result import_file(const Work& work);
    void refresh_configuration(uint32_t now_ms);
    Result queue_import(Client& client);
    void finish(Client& client, Result result);
    Result execute(const Request& request, uint32_t now_ms, uint32_t stop_mask);

    static constexpr uint8_t client_count = 4;
    static constexpr uint32_t client_timeout_ms = 120000;
    static constexpr uint32_t all_targets = (1UL << (Sequencer::lane_count + 1)) - 1;

    HAL_Semaphore _io_sem;
    Work _work;
    Completed _completed;
    Upload _upload = Upload::Empty;
    Sender _uploader;
    bool _maintenance = false;
    Sequencer::Document _documents[2];
    uint8_t _bytes[SequencerDocument::max_bytes] {};
    uint8_t _active = 0;
    uint32_t _observed_crc = 0;
    uint32_t _last_refresh_ms = 0;
    bool _refresh_pending = true;
    bool _valid = false;
    bool _busy = false;
    int8_t _applying_client = -1;
    Config _config = Config::Loading;
    uint32_t _backend_serial = 0;
    uint32_t _activation_serial = 0;
    uint32_t _status_requests = 0;
    Client _clients[client_count];
};

} // namespace Vektor2
