#pragma once

#include "Logic.h"
#include "VTelemetryCore.h"

namespace Vektor2 {
namespace VTelemetryCore {

// MAVLink NAMED_VALUE_FLOAT names are limited to ten characters.
// Field ordering exactly matches VTEL offsets 0..6 for each drive.
inline const char* azipod_named_key(uint8_t pod, uint8_t field)
{
    static const char* const keys[2][7] = {
        {"AZI1_AZPWM", "AZI1_MTPWM", "AZI1_ACMD", "AZI1_ATGT", "AZI1_MCMD", "AZI1_STATE", "AZI1_RB"},
        {"AZI2_AZPWM", "AZI2_MTPWM", "AZI2_ACMD", "AZI2_ATGT", "AZI2_MCMD", "AZI2_STATE", "AZI2_RB"}
    };
    return keys[pod][field];
}

// IDs match the VTEL source catalog. Reads only the selected runtime backend.
inline bool read_propulsion_source(uint16_t id, Sample& s, const Logic& logic)
{
    switch (id) {
    case 270: s.number = uint8_t(logic.main_propulsion_type()); return true;
    case 271:
        if (!logic.vsp_selected()) { return false; }
        s.number = logic.vsp_configuration(); return true;
    case 272:
        if (logic.main_propulsion_type() != MainPropulsionType::Azipod) { return false; }
        s.number = uint8_t(logic.azipod_layout()); return true;
    default: break;
    }
    const bool first = id >= 280 && id <= 286;
    if (!first && !(id >= 290 && id <= 296)) { return false; }
    const ComponentId component = first ? ComponentId::AZI1 : ComponentId::AZI2;
    const auto* drive = logic.azipod_controller(component);
    if (!drive) { return false; }
    const uint8_t field = id - (first ? 280 : 290);
    const auto state = drive->state();
    if (field == 5) { s.number = uint8_t(state); return true; }
    if (field == 6) { s.number = drive->reasons(); return true; }
    // A publication failure can leave the previous computed PWM in Logic.
    // Suppress command readings until the controller reinitializes.
    if (state == Components::AzipodControl::State::Blocked) { return false; }
    if (field < 2) {
        if (!logic.output_valid(component, field)) { return false; }
        s.number = logic.output(component, field); return true;
    }
    if (!logic.output_valid(component, field == 4 ? 1 : 0)) { return false; }
    if (field == 2) { s.number = drive->commanded_angle(); }
    else if (field == 3) { s.number = drive->target_angle(); }
    else { s.number = drive->motor_command(); }
    return true;
}

} // namespace VTelemetryCore
} // namespace Vektor2
