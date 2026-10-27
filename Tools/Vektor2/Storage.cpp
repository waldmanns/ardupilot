#include "Storage.h"
#include "Vektor2.h"

#include <AP_Filesystem/AP_Filesystem.h>

extern const AP_HAL::HAL& hal;

namespace Vektor2 {
void Storage::init()
{
#if !AP_FILESYSTEM_FILE_WRITING_ENABLED
    _status.state = STORAGE_STATUS_NOT_SUPPORTED;
    _status.count = 0;
#endif
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&Storage::io_timer, void));
}

Storage::Status Storage::status() const
{
    WITH_SEMAPHORE(_sem);
    auto snapshot = _status;
    if (formatting()) {
        snapshot.state = STORAGE_STATUS_UNFORMATTED;
        snapshot.total_bytes = snapshot.free_bytes = -1;
    }
    return snapshot;
}

void Storage::recording_active()
{
    WITH_SEMAPHORE(_sem);
    _status.recording_stopped = false;
}

bool Storage::formatting() const
{
#if AP_FILESYSTEM_FORMAT_ENABLED
    const auto state = AP::FS().get_format_status();
    return state == AP_Filesystem_Backend::FormatStatus::PENDING ||
           state == AP_Filesystem_Backend::FormatStatus::IN_PROGRESS;
#else
    return false;
#endif
}

void Storage::format_started()
{
    WITH_SEMAPHORE(_sem);
    _status.state = STORAGE_STATUS_UNFORMATTED;
    _status.total_bytes = _status.free_bytes = -1;
    _status.revision++;
}

void Storage::io_timer()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (_polled && now_ms - _last_poll_ms < 1000) {
        return;
    }
    _last_poll_ms = now_ms;
    if (formatting()) {
        return;
    }
    // LOG_DISARMED is the recording switch in this application. Closing is
    // deliberately off the control thread, including card sync/close latency.
    Status next;
    next.recording_stopped = !app.logger.log_while_disarmed();
    if (next.recording_stopped) {
        app.logger.StopLogging();
    }

#if AP_FILESYSTEM_FILE_WRITING_ENABLED
    if (AP::FS().retry_mount()) {
        next.total_bytes = AP::FS().disk_space("/");
        next.free_bytes = AP::FS().disk_free("/");
        // Some FATFS errors are small positive FRESULT values. A real volume
        // must have at least a sector and valid, ordered capacity counters.
        if (next.total_bytes >= 512 && next.free_bytes >= 0 && next.free_bytes <= next.total_bytes &&
            next.free_bytes % 512 == 0) {
            next.state = STORAGE_STATUS_READY;
        } else {
            next.total_bytes = next.free_bytes = -1;
        }
    }
#else
    next.state = STORAGE_STATUS_NOT_SUPPORTED;
    next.count = 0;
#endif
    WITH_SEMAPHORE(_sem);
    next.revision = _status.revision + (!_polled || next.state != _status.state);
    _status = next;
    _polled = true;
}

void Storage::report_changes()
{
    const auto snapshot = status();
    if (snapshot.revision == _reported_revision) {
        return;
    }
    _reported_revision = snapshot.revision;
    if (snapshot.state == STORAGE_STATUS_READY) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SD ready: %lu MiB free",
                      (unsigned long)(snapshot.free_bytes / (1024 * 1024)));
    } else if (snapshot.state == STORAGE_STATUS_UNFORMATTED) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SD format in progress");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SD unavailable");
    }
}
} // namespace Vektor2
