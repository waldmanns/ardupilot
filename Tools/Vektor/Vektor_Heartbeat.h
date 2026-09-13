#pragma once

#include <stdint.h>

namespace Vektor {

class Heartbeat {
public:
    static constexpr uint32_t default_half_period_ms = 500;

    void init(uint32_t now_ms,
              uint32_t half_period_ms = default_half_period_ms);
    bool update(uint32_t now_ms, bool &level);
    bool level() const { return _level; }

private:
    uint32_t _last_change_ms = 0;
    uint32_t _half_period_ms = default_half_period_ms;
    bool _level = false;
};

} // namespace Vektor
