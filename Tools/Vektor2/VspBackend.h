#pragma once

#include "VspVectorControl.h"
#include "VspAllocator.h"
#include "VspRpmGovernor.h"
#include "RollStabilizationBackend.h"

namespace Vektor2 {
// VSP-only runtime state. Persistent AP_Param stores remain in Logic until
// dynamic subgroup support is verified against the target ArduPilot tree.
struct VspBackend {
    VspVectorControl vectors[2];
    VspRpmGovernor rpm_governors[2];
    VspRpmGovernor::Feedback shaft_rpm[2] {};
    VspAllocator allocator;
    bool allocated_rods = false;
    RollStabilizationBackend vrs;
    uint8_t authority[2] {};
    bool roll_enabled = false;
};
} // namespace Vektor2
