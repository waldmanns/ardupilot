#pragma once

#include "VTelemetryCore.h"
#include <AP_Param/AP_Param.h>

namespace Vektor2 {

class VTelemetryParameters {
public:
    VTelemetryParameters();
    static const AP_Param::GroupInfo var_info[];
    static const AP_Param::GroupInfo var_info2[]; // extension for slots 62..65

    // Only source IDs are stored. Changing a field's meaning requires a new
    // source ID, never silently repurposing an existing persisted assignment.
    AP_Int16 slot[VTelemetryCore::slot_count];

    // Return true when a complete parameter snapshot differs from the last.
    bool changed(int16_t (&ids)[VTelemetryCore::slot_count]);

private:
    int16_t _previous[VTelemetryCore::slot_count] {};
    bool _seen = false;
};

} // namespace Vektor2
