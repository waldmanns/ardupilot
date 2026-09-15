#pragma once

#include <stdint.h>

namespace Vektor {

enum class SignalQuality : uint8_t {
    INVALID = 0,
    STALE = 1,
    VALID = 2,
};

template <typename T>
struct SignalSample {
    T value;
    uint64_t timestamp_us;
    SignalQuality quality;
};

} // namespace Vektor
