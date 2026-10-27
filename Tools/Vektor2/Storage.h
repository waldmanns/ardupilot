#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

namespace Vektor2 {

// Slow filesystem operations run on the HAL IO thread. MAVLink and the main
// loop copy a small cached snapshot; they never scan the card for status.
class Storage {
public:
    struct Status {
        int64_t total_bytes = -1;
        int64_t free_bytes = -1;
        uint32_t revision = 0;
        uint8_t state = STORAGE_STATUS_EMPTY;
        uint8_t count = 1;
        bool recording_stopped = false;
    };

    void init();
    Status status() const;
    bool formatting() const;
    void format_started();
    void report_changes();
    void recording_active();

private:
    void io_timer();
    mutable HAL_Semaphore _sem;
    Status _status;
    uint32_t _last_poll_ms = 0;
    uint32_t _reported_revision = 0;
    bool _polled = false;
};
} // namespace Vektor2
