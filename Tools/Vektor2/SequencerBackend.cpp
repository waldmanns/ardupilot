#include "SequencerBackend.h"

#include <AP_Filesystem/AP_Filesystem.h>
#include <errno.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

namespace Vektor2 {

using Operation = SequencerBackend::Operation;
using Result = SequencerBackend::Result;

static uint32_t le32(const uint8_t* bytes)
{
    return uint32_t(bytes[0]) | uint32_t(bytes[1]) << 8 |
        uint32_t(bytes[2]) << 16 | uint32_t(bytes[3]) << 24;
}

void SequencerBackend::init()
{
    const uint32_t now_ms = AP_HAL::millis();
    engine.activate(params.enable.get() == 1);
    refresh_configuration(now_ms);
    engine.schedule_boot_start(now_ms);
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&SequencerBackend::io_timer, void));
#endif
}

void SequencerBackend::refresh_configuration(uint32_t now_ms)
{
    _refresh_pending = false;
    _last_refresh_ms = now_ms;
    const uint8_t index = 1 - _active;
    auto& document = _documents[index];
    params.read(document);
    document.revision = 0;
    SequencerDocument::encode(document, _bytes);
    const uint32_t crc = le32(_bytes + 16);
    if (_config != Config::Loading && crc == _observed_crc) {
        return;
    }

    _observed_crc = crc;
    _valid = document.valid();
    _config = _valid ? Config::Active : Config::Invalid;
    if (_valid) {
        // Content revisions stay stable when the same parameters reload at boot.
        document.revision = crc == 0 ? 1 : crc;
        _active = index;
    }
    engine.load(_valid ? &_documents[_active] : nullptr);
    ++_backend_serial;
}

Result SequencerBackend::enqueue(const Request& request, bool& pending)
{
    pending = false;
    const uint32_t now = AP_HAL::millis();
    Client* free = nullptr;
    for (auto& client : _clients) {
        if (client.used && !client.pending && !client.notify && now - client.last_ms > client_timeout_ms) {
            client.used = false;
        }
        if (!client.used) {
            if (free == nullptr) {
                free = &client;
            }
            continue;
        }
        if (!(client.ack.request.sender == request.sender)) {
            continue;
        }
        client.last_ms = now;
        const auto& previous = client.ack.request;
        const uint16_t advance = request.token - previous.token;
        if (advance == 0) {
            if (request.operation != previous.operation || request.revision != previous.revision ||
                request.crc != previous.crc || request.target != previous.target) {
                return Result::Stale;
            }
            if (request.operation == Operation::Status) {
                _status_requests |= 1UL << request.link;
            }
            pending = client.pending;
            return client.ack.result;
        }
        if (advance >= 32768) {
            return Result::Stale;
        }
        if (client.pending || client.notify) {
            return Result::Busy;
        }
        free = &client;
        break;
    }
    if (free == nullptr) {
        return Result::Busy;
    }
    free->used = true;
    free->pending = true;
    free->last_ms = now;
    free->ack.request = request;
    free->ack.result = Result::Success;
    pending = true;
    return Result::Success;
}

bool SequencerBackend::next_ack(mavlink_channel_t link, Ack& ack)
{
    for (auto& client : _clients) {
        if (client.notify && client.ack.request.link == link) {
            client.notify = false;
            ack = client.ack;
            return true;
        }
    }
    return false;
}

bool SequencerBackend::take_status_request(mavlink_channel_t link)
{
    const auto mask = 1UL << link;
    const bool requested = _status_requests & mask;
    _status_requests &= ~mask;
    return requested;
}

void SequencerBackend::finish(Client& client, Result result)
{
    client.ack.result = result;
    client.pending = false;
    client.notify = true;
    client.last_ms = AP_HAL::millis();
}

Result SequencerBackend::queue_import(Client& client)
{
#if !AP_FILESYSTEM_FILE_WRITING_ENABLED
    return Result::Unsupported;
#else
    if (!engine.active()) {
        return Result::Locked;
    }
    if (_busy || engine.powered()) {
        return Result::Busy;
    }
    const auto& request = client.ack.request;
    if (request.revision != (_valid ? _documents[_active].revision : 0)) {
        return Result::Stale;
    }

    WITH_SEMAPHORE(_io_sem);
    if (_maintenance || _upload == Upload::Open) {
        return Result::Busy;
    }
    if (_upload != Upload::Closed || !(_uploader == request.sender)) {
        return Result::Invalid;
    }
    _upload = Upload::Reserved;
    _work.pending = true;
    _work.candidate = 1 - _active;
    _work.crc = request.crc;
    _applying_client = &client - _clients;
    _busy = true;
    ++_backend_serial;
    return Result::Success;
#endif
}

Result SequencerBackend::execute(const Request& request, uint32_t now_ms, uint32_t stop_mask)
{
    switch (request.operation) {
    case Operation::Status:
        _status_requests |= 1UL << request.link;
        return Result::Success;
    case Operation::PowerOff:
        engine.power(false);
        return Result::Success;
    case Operation::StopAll:
        engine.stop_all();
        return Result::Success;
    case Operation::SharedStop:
    case Operation::LaneStop:
        engine.stop(request.target);
        return Result::Success;
    case Operation::Discard:
        return Result::Unsupported;
    case Operation::ApplyParameters:
        // Compatibility for clients that explicitly validate the current values.
        if (!engine.active()) {
            return Result::Locked;
        }
        if (_busy) {
            return Result::Busy;
        }
        if (!_valid) {
            return Result::Invalid;
        }
        return request.revision == _documents[_active].revision ? Result::Success : Result::Stale;
    case Operation::ApplyFile:
        return Result::Invalid; // Queued with its owning client by update().
    case Operation::PowerOn:
    case Operation::SharedStart:
    case Operation::LaneStart:
        if (!engine.active()) {
            return Result::Locked;
        }
        if (_busy) {
            return Result::Busy;
        }
        if (!_valid) {
            return Result::Unavailable;
        }
        if (_documents[_active].revision != request.revision) {
            return Result::Stale;
        }
        if (request.operation == Operation::PowerOn) {
            return stop_mask == all_targets || !engine.power(true) ? Result::Inhibited : Result::Success;
        }
        return (stop_mask & (1UL << request.target)) || !engine.start(request.target, now_ms) ?
            Result::Inhibited : Result::Success;
    }
    return Result::Invalid;
}

void SequencerBackend::update(uint32_t now_ms)
{
    Completed completion;
    if (_busy) {
        WITH_SEMAPHORE(_io_sem);
        completion = _completed;
        if (completion.ready) {
            _completed.ready = false;
            _upload = Upload::Closed;
            _busy = false;
        }
    }
    if ((params.enable.get() == 1) != engine.active()) {
        engine.activate(params.enable.get() == 1);
        ++_activation_serial;
    }
    // A periodic refresh also picks up ordinary packed parameter uploads.
    if (!_busy && (_refresh_pending || completion.ready || now_ms - _last_refresh_ms >= 100)) {
        refresh_configuration(now_ms);
    }
    if (completion.ready) {
        finish(_clients[_applying_client], completion.result);
        _applying_client = -1;
        ++_backend_serial;
    }

    // Execute every stop first, then consume edges before executing manual
    // starts. Neither an edge nor a manual start can override a queued stop.
    uint32_t stop_mask = 0;
    for (auto& client : _clients) {
        if (!client.pending) {
            continue;
        }
        const auto& request = client.ack.request;
        switch (request.operation) {
        case Operation::PowerOff:
        case Operation::StopAll:
            stop_mask = all_targets;
            break;
        case Operation::SharedStop:
        case Operation::LaneStop:
            stop_mask |= 1UL << request.target;
            break;
        default:
            continue;
        }
        finish(client, execute(request, now_ms, stop_mask));
    }
    engine.tick(now_ms, stop_mask);
    // Reserve a queued import before considering power-on/start requests.
    for (auto& client : _clients) {
        if (client.pending && (&client - _clients) != _applying_client &&
            client.ack.request.operation == Operation::ApplyFile) {
            const auto result = queue_import(client);
            if (result != Result::Success) {
                finish(client, result);
            }
        }
    }
    for (auto& client : _clients) {
        if (client.pending && (&client - _clients) != _applying_client) {
            finish(client, execute(client.ack.request, now_ms, stop_mask));
        }
    }
}

uint32_t SequencerBackend::status_serial() const
{
    return _backend_serial + engine.status_revision();
}

uint8_t SequencerBackend::telemetry_rate() const
{
    return _valid ? _documents[_active].rate : Sequencer::default_rate_hz;
}

SequencerBackend::Snapshot SequencerBackend::snapshot(uint32_t now_ms) const
{
    Snapshot result;
    result.time = now_ms;
    result.revision = _valid ? _documents[_active].revision : 0;
    const auto& shared = engine.shared_status();
    result.state = uint32_t(engine.active()) | (uint32_t(engine.powered()) << 1) |
        (uint32_t(_valid) << 2) | (uint32_t(!AP_Param::get_eeprom_full()) << 3) |
        (uint32_t(_busy) << 4) | (uint32_t(shared.trigger_valid) << 5) |
        (uint32_t(engine.boot_start_pending()) << 6) |
        (uint32_t(shared.state) << 8) | (uint32_t(shared.reason) << 12) |
        (uint32_t(_config) << 16);
    result.shared_run = shared.run;
    for (uint8_t lane = 0; lane < Sequencer::lane_count; lane++) {
        result.lanes[lane] = engine.status(lane);
    }
    return result;
}

bool SequencerBackend::begin_upload(Sender sender)
{
    WITH_SEMAPHORE(_io_sem);
    if (_maintenance || _work.pending || _completed.ready || _upload == Upload::Open ||
        _upload == Upload::Reserved) {
        return false;
    }
    _upload = Upload::Open;
    _uploader = sender;
    return true;
}

void SequencerBackend::end_upload(bool complete)
{
    WITH_SEMAPHORE(_io_sem);
    _upload = complete ? Upload::Closed : Upload::Empty;
}

bool SequencerBackend::begin_maintenance()
{
    WITH_SEMAPHORE(_io_sem);
    if (_busy || _maintenance || _upload == Upload::Open) {
        return false;
    }
    _maintenance = true;
    _upload = Upload::Empty;
    return true;
}

void SequencerBackend::end_maintenance()
{
    WITH_SEMAPHORE(_io_sem);
    _maintenance = false;
}

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
static int read_file(const char* path, uint8_t* bytes, size_t capacity)
{
    auto& fs = AP::FS();
    const int fd = fs.open(path, O_RDONLY);
    if (fd < 0) {
        return errno == ENOENT ? 0 : -1;
    }
    size_t count = 0;
    uint8_t extra;
    while (count <= capacity) {
        const ssize_t got = fs.read(fd, count == capacity ? &extra : bytes + count,
                                    count == capacity ? 1 : capacity - count);
        if (got < 0) {
            count = 0;
            break;
        }
        if (got == 0) {
            break;
        }
        count += got;
    }
    if (fs.close(fd) != 0 || count == 0) {
        return -1;
    }
    return int(count);
}

Result SequencerBackend::import_file(const Work& work)
{
    const int size = read_file("VSEQNEW.BIN", _bytes, sizeof(_bytes));
    if (size <= 0) {
        return Result::Storage;
    }
    auto& candidate = _documents[work.candidate];
    const auto decoded = SequencerDocument::decode(_bytes, size, candidate);
    if (decoded == SequencerDocument::Decode::Unsupported) {
        return Result::Unsupported;
    }
    if (decoded != SequencerDocument::Decode::Valid ||
        SequencerDocument::checksum(_bytes, size) != work.crc) {
        return Result::Invalid;
    }
    if (AP_Param::get_eeprom_full()) {
        return Result::Storage;
    }
    params.import(candidate);
    return AP_Param::get_eeprom_full() ? Result::Storage : Result::Success;
}

void SequencerBackend::io_timer()
{
    Work work;
    {
        WITH_SEMAPHORE(_io_sem);
        if (!_work.pending) {
            return;
        }
        work = _work;
    }
    const auto result = import_file(work);
    {
        WITH_SEMAPHORE(_io_sem);
        _work.pending = false;
        _completed.result = result;
        _completed.ready = true;
    }
}
#endif

} // namespace Vektor2
