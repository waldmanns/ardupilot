#include "VTelemetryParameters.h"

namespace Vektor2 {

VTelemetryParameters::VTelemetryParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Defaults map fields to the nearest real Vektor2 measurements. Unavailable
// measurements stay invalid at runtime; 0 means there is no honest equivalent.
// Saved AP_Param values always take precedence, including an explicit 0.
const AP_Param::GroupInfo VTelemetryParameters::var_info[] = {
    // @Param: SLT1
    // @DisplayName: i-BUS sensor address 1 source ID
    // @Description: Stable VTEL source ID; factory default is an i-BUS GPS status sensor. Zero disables the position. Map i-BUS addresses contiguously from 1.
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("SLT1", 1, VTelemetryParameters, slot[0], 18),
    AP_GROUPINFO("SLT2", 2, VTelemetryParameters, slot[1], 100),
    AP_GROUPINFO("SLT3", 3, VTelemetryParameters, slot[2], 1),
    AP_GROUPINFO("SLT4", 4, VTelemetryParameters, slot[3], 10),
    AP_GROUPINFO("SLT5", 5, VTelemetryParameters, slot[4], 11),
    AP_GROUPINFO("SLT6", 6, VTelemetryParameters, slot[5], 12),
    AP_GROUPINFO("SLT7", 7, VTelemetryParameters, slot[6], 9),
    AP_GROUPINFO("SLT8", 8, VTelemetryParameters, slot[7], 3),
    AP_GROUPINFO("SLT9", 9, VTelemetryParameters, slot[8], 8),
    AP_GROUPINFO("SLT10", 10, VTelemetryParameters, slot[9], 6),
    AP_GROUPINFO("SLT11", 11, VTelemetryParameters, slot[10], 7),
    AP_GROUPINFO("SLT12", 12, VTelemetryParameters, slot[11], 90),
    AP_GROUPINFO("SLT13", 13, VTelemetryParameters, slot[12], 91),
    AP_GROUPINFO("SLT14", 14, VTelemetryParameters, slot[13], 2),
    AP_GROUPINFO("SLT15", 15, VTelemetryParameters, slot[14], 101),
    // @Param: SLT16
    // @DisplayName: HoTT EAM cell 1 source ID
    // @Description: HoTT EAM cell fields start here. They default to zero because measurement point voltages are not necessarily individual cell voltages. Numeric sources accepted regardless of physical meaning.
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("SLT16", 16, VTelemetryParameters, slot[15], 0),
    AP_GROUPINFO("SLT17", 17, VTelemetryParameters, slot[16], 0),
    AP_GROUPINFO("SLT18", 18, VTelemetryParameters, slot[17], 0),
    AP_GROUPINFO("SLT19", 19, VTelemetryParameters, slot[18], 0),
    AP_GROUPINFO("SLT20", 20, VTelemetryParameters, slot[19], 0),
    AP_GROUPINFO("SLT21", 21, VTelemetryParameters, slot[20], 0),
    AP_GROUPINFO("SLT22", 22, VTelemetryParameters, slot[21], 0),
    AP_GROUPINFO("SLT23", 23, VTelemetryParameters, slot[22], 0),
    AP_GROUPINFO("SLT24", 24, VTelemetryParameters, slot[23], 0),
    AP_GROUPINFO("SLT25", 25, VTelemetryParameters, slot[24], 0),
    AP_GROUPINFO("SLT26", 26, VTelemetryParameters, slot[25], 0),
    AP_GROUPINFO("SLT27", 27, VTelemetryParameters, slot[26], 0),
    AP_GROUPINFO("SLT28", 28, VTelemetryParameters, slot[27], 0),
    AP_GROUPINFO("SLT29", 29, VTelemetryParameters, slot[28], 0),
    AP_GROUPINFO("SLT30", 30, VTelemetryParameters, slot[29], 100),
    AP_GROUPINFO("SLT31", 31, VTelemetryParameters, slot[30], 101),
    AP_GROUPINFO("SLT32", 32, VTelemetryParameters, slot[31], 0),
    AP_GROUPINFO("SLT33", 33, VTelemetryParameters, slot[32], 0),
    AP_GROUPINFO("SLT34", 34, VTelemetryParameters, slot[33], 8),
    AP_GROUPINFO("SLT35", 35, VTelemetryParameters, slot[34], 0),
    AP_GROUPINFO("SLT36", 36, VTelemetryParameters, slot[35], 100),
    AP_GROUPINFO("SLT37", 37, VTelemetryParameters, slot[36], 0),
    AP_GROUPINFO("SLT38", 38, VTelemetryParameters, slot[37], 9),
    AP_GROUPINFO("SLT39", 39, VTelemetryParameters, slot[38], 0),
    AP_GROUPINFO("SLT40", 40, VTelemetryParameters, slot[39], 90),
    AP_GROUPINFO("SLT41", 41, VTelemetryParameters, slot[40], 0),
    AP_GROUPINFO("SLT42", 42, VTelemetryParameters, slot[41], 0),
    AP_GROUPINFO("SLT43", 43, VTelemetryParameters, slot[42], 2),
    // @Param: SLT44
    // @DisplayName: HoTT GPS direction source ID
    // @Description: First of HoTT GPS fixed fields, slots 44 to 58. GPS latitude/longitude slots 46 and 47 require coordinate sources.
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("SLT44", 44, VTelemetryParameters, slot[43], 3),
    AP_GROUPINFO("SLT45", 45, VTelemetryParameters, slot[44], 2),
    AP_GROUPINFO("SLT46", 46, VTelemetryParameters, slot[45], 6),
    AP_GROUPINFO("SLT47", 47, VTelemetryParameters, slot[46], 7),
    AP_GROUPINFO("SLT48", 48, VTelemetryParameters, slot[47], 0),
    AP_GROUPINFO("SLT49", 49, VTelemetryParameters, slot[48], 8),
    AP_GROUPINFO("SLT50", 50, VTelemetryParameters, slot[49], 9),
    AP_GROUPINFO("SLT51", 51, VTelemetryParameters, slot[50], 0),
    AP_GROUPINFO("SLT52", 52, VTelemetryParameters, slot[51], 4),
    AP_GROUPINFO("SLT53", 53, VTelemetryParameters, slot[52], 5),
    AP_GROUPINFO("SLT54", 54, VTelemetryParameters, slot[53], 0),
    AP_GROUPINFO("SLT55", 55, VTelemetryParameters, slot[54], 13),
    AP_GROUPINFO("SLT56", 56, VTelemetryParameters, slot[55], 15),
    AP_GROUPINFO("SLT57", 57, VTelemetryParameters, slot[56], 14),
    AP_GROUPINFO("SLT58", 58, VTelemetryParameters, slot[57], 16),
    // @Param: SLT59
    // @DisplayName: HoTT VARIO altitude source ID
    // @Description: First of HoTT VARIO numeric fields, slots 59 to 65.
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("SLT59", 59, VTelemetryParameters, slot[58], 8),
    AP_GROUPINFO("SLT60", 60, VTelemetryParameters, slot[59], 0),
    AP_GROUPINFO("SLT61", 61, VTelemetryParameters, slot[60], 0),
    // ArduPilot AP_Param group indexes are 6-bit. Do not use direct 64/65
    // group indexes: they fail check_var_info(). An empty-name extension
    // retains the external parameter names VTEL_SLT62..VTEL_SLT65.
    AP_SUBGROUPEXTENSION("", 62, VTelemetryParameters, var_info2),
    AP_GROUPEND
};

const AP_Param::GroupInfo VTelemetryParameters::var_info2[] = {
    AP_GROUPINFO("SLT62", 1, VTelemetryParameters, slot[61], 9),
    AP_GROUPINFO("SLT63", 2, VTelemetryParameters, slot[62], 0),
    AP_GROUPINFO("SLT64", 3, VTelemetryParameters, slot[63], 0),
    AP_GROUPINFO("SLT65", 4, VTelemetryParameters, slot[64], 12),
    AP_GROUPEND
};

bool VTelemetryParameters::changed(int16_t (&ids)[VTelemetryCore::slot_count])
{
    bool change = !_seen;
    for (uint8_t i = 0; i < VTelemetryCore::slot_count; i++) {
        ids[i] = slot[i].get();
        if (ids[i] != _previous[i]) {
            change = true;
        }
    }
    if (change) {
        for (uint8_t i = 0; i < VTelemetryCore::slot_count; i++) {
            _previous[i] = ids[i];
        }
        _seen = true;
    }
    return change;
}

} // namespace Vektor2
