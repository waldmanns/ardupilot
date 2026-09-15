#include "Vektor_Runtime.h"

#include <stdint.h>

namespace Vektor {

uint32_t RuntimeState::clamp_u32(uint64_t value)
{
    if (value > UINT32_MAX) {
        return UINT32_MAX;
    }
    return uint32_t(value);
}

void RuntimeState::init(uint32_t service_rate_hz, uint64_t now_us)
{
    _boot_time_us = now_us;
    _last_loop_start_us = 0;
    _next_loop_start_us = now_us;
    _expected_loop_start_us = 0;
    _loop_count = 0;
    _last_loop_dt_us = 0;
    _last_loop_work_us = 0;
    _max_loop_work_us = 0;
    _last_loop_lateness_us = 0;
    _current_loop_lateness_us = 0;
    _late_loop_count = 0;
    _service_rate_hz = service_rate_hz > UINT16_MAX ?
        UINT16_MAX :
        uint16_t(service_rate_hz);
    _loop_period_us = service_rate_hz == 0 ? 1 :
        uint32_t((1000000ULL + service_rate_hz / 2U) / service_rate_hz);
    _last_loop_late = false;
    _have_expected_loop_start = false;
    _started = false;
}

void RuntimeState::begin_loop(uint64_t now_us)
{
    _current_loop_lateness_us = 0;
    if (_started) {
        _last_loop_dt_us = now_us >= _last_loop_start_us ?
            clamp_u32(now_us - _last_loop_start_us) :
            0;
        if (_have_expected_loop_start && now_us > _expected_loop_start_us) {
            _current_loop_lateness_us =
                clamp_u32(now_us - _expected_loop_start_us);
        }
    } else {
        _last_loop_dt_us = 0;
        _next_loop_start_us = now_us > UINT64_MAX - _loop_period_us ?
            UINT64_MAX : now_us + _loop_period_us;
        _started = true;
    }

    _last_loop_start_us = now_us;
    if (_loop_count != UINT32_MAX) {
        _loop_count++;
    }
}

uint32_t RuntimeState::delay_until_next_loop_us(uint64_t now_us)
{
    if (!_started || _next_loop_start_us == UINT64_MAX) {
        return 0;
    }

    if (now_us > _next_loop_start_us) {
        const uint32_t completion_lateness =
            clamp_u32(now_us - _next_loop_start_us);
        if (completion_lateness > _current_loop_lateness_us) {
            _current_loop_lateness_us = completion_lateness;
        }
        const uint64_t periods_missed =
            ((now_us - _next_loop_start_us) / _loop_period_us) + 1U;
        if (periods_missed >
            (UINT64_MAX - _next_loop_start_us) / _loop_period_us) {
            _next_loop_start_us = UINT64_MAX;
            return 0;
        }
        _next_loop_start_us += periods_missed * _loop_period_us;
    }

    _last_loop_lateness_us = _current_loop_lateness_us;
    _last_loop_late = _last_loop_lateness_us != 0;
    if (_last_loop_late && _late_loop_count != UINT32_MAX) {
        _late_loop_count++;
    }

    _expected_loop_start_us = _next_loop_start_us;
    _have_expected_loop_start = true;
    const uint64_t delay_us = _next_loop_start_us - now_us;
    _next_loop_start_us = _next_loop_start_us >
                                  UINT64_MAX - _loop_period_us ?
                              UINT64_MAX :
                              _next_loop_start_us + _loop_period_us;
    return clamp_u32(delay_us);
}

void RuntimeState::end_loop(uint64_t now_us)
{
    if (!_started) {
        return;
    }

    _last_loop_work_us = now_us >= _last_loop_start_us ?
        clamp_u32(now_us - _last_loop_start_us) :
        0;

    if (_last_loop_work_us > _max_loop_work_us) {
        _max_loop_work_us = _last_loop_work_us;
    }
}

uint32_t RuntimeState::uptime_ms(uint64_t now_us) const
{
    if (now_us < _boot_time_us) {
        return 0;
    }
    return clamp_u32((now_us - _boot_time_us) / 1000U);
}

} // namespace Vektor
