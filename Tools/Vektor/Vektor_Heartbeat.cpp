#include "Vektor_Heartbeat.h"

namespace Vektor {

void Heartbeat::init(uint32_t now_ms, uint32_t half_period_ms)
{
    _last_change_ms = now_ms;
    _half_period_ms = half_period_ms == 0 ? 1 : half_period_ms;
    _level = false;
}

bool Heartbeat::update(uint32_t now_ms, bool &level)
{
    if (now_ms - _last_change_ms < _half_period_ms) {
        return false;
    }

    // Toggle once from current time. Missed low-priority heartbeat periods do
    // not create a backlog that can interfere with control work.
    _last_change_ms = now_ms;
    _level = !_level;
    level = _level;
    return true;
}

} // namespace Vektor
