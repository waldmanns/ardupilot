#pragma once

#include <stdint.h>

namespace Vektor {

class RuntimeState {
public:
    void init(uint32_t service_rate_hz, uint64_t now_us);
    void begin_loop(uint64_t now_us);
    void end_loop(uint64_t now_us);

    uint32_t uptime_ms(uint64_t now_us) const;
    uint32_t loop_count() const { return _loop_count; }
    uint32_t last_loop_dt_us() const { return _last_loop_dt_us; }
    uint32_t last_loop_work_us() const { return _last_loop_work_us; }
    uint32_t max_loop_work_us() const { return _max_loop_work_us; }
    uint16_t service_rate_hz() const { return _service_rate_hz; }

private:
    static uint32_t clamp_u32(uint64_t value);

    uint64_t _boot_time_us = 0;
    uint64_t _last_loop_start_us = 0;
    uint32_t _loop_count = 0;
    uint32_t _last_loop_dt_us = 0;
    uint32_t _last_loop_work_us = 0;
    uint32_t _max_loop_work_us = 0;
    uint16_t _service_rate_hz = 0;
    bool _started = false;
};

} // namespace Vektor
