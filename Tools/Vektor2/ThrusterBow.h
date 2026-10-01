#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t mid;
    int16_t dst;
} ThrusterBowParams;

// One input and one output, both in pulse-width microseconds.
void thruster_bow_loop(const uint16_t* inputs,
                       uint16_t* outputs,
                       const ThrusterBowParams* params);

#ifdef __cplusplus
}
#endif
