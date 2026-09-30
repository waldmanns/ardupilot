#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
