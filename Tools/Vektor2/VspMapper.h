#pragma once

#include <stdint.h>

namespace Vektor2 {

// Body frame: forward positive, starboard positive; angles clockwise from bow.
// Rotation is viewed from above. Rod +X is right, +Y is up.
class VspMapper {
public:
    enum class Rotation : int16_t { CW = 0, CCW = 1 };
    struct Vector { float forward; float starboard; };
    struct Config {
        Rotation rotation;
        float installation_deg;
        int16_t x_center;
        int16_t y_center;
        int16_t radius;
    };
    struct Estimate {
        bool valid;
        Vector vector;
        float direction_deg; // NaN at neutral or invalid
        float intensity;     // normalized rod excursion, NOT measured force
    };
    struct Result {
        bool valid;
        bool limited;
        uint16_t x_us;
        uint16_t y_us;
        Estimate applied;
    };

    static bool valid_config(const Config& config);
    static Vector from_polar(float direction_deg, float intensity);
    static Result map(const Config& config, Vector demand);
    // Decode final component rods, after heading/VRS and before PWM scaling.
    static Estimate estimate(const Config& config, uint16_t x_us, uint16_t y_us);
};

} // namespace Vektor2
