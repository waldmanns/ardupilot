#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VSP_PWM_MIN 1000
#define VSP_PWM_MAX 2000

// Clamp to the input endpoints and map either input polarity. With equal
// input endpoints, x <= in_min selects out_min; larger x selects out_max.
// Fractional output steps are truncated toward the lower input endpoint.
int32_t vsp_common_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

// Coordinates and limits use signed values for offsets and intermediate results.
struct Limiter {
    int32_t radius;
    int32_t x_offset;
    int32_t y_offset;
};

struct Coordinates {
    int32_t x_coordinate;
    int32_t y_coordinate;
};

struct Limiter setupLimiter(int32_t radius, int32_t x_offset, int32_t y_offset);
struct Coordinates mapCoordinates(struct Coordinates input,
                                  struct Limiter limits,
                                  int32_t angle_correction);

#ifdef __cplusplus
}
#endif
