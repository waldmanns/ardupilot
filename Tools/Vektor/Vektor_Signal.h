#pragma once

#include <stdint.h>

namespace Vektor {

enum class SignalQuality : uint8_t {
    INVALID = 0,
    STALE = 1,
    VALID = 2,
};

enum class SignalType : uint8_t {
    BOOL = 0,
    NORMALIZED_SCALAR,
    INTEGER,
    ANGLE_RAD,
    ANGULAR_RATE_RAD_S,
    RPM,
    VOLTAGE,
    POSITION,
    PULSE_WIDTH_US,
    VECTOR2,
    VECTOR3,
    QUATERNION,
};

template <typename T>
struct SignalSample {
    T value;
    uint64_t timestamp_us;
    SignalQuality quality;
};

struct Vector2 {
    float x;
    float y;
};

struct Vector3 {
    float x;
    float y;
    float z;
};

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

} // namespace Vektor
