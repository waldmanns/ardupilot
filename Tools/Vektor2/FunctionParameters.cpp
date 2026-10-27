#include "FunctionParameters.h"

namespace Vektor2 {

const AP_Param::GroupInfo FunctionSlotParameters::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Function slot enable
    // @Description: Enables evaluation of this slot. A disabled assigned slot contributes no request; selecting TYPE=0 removes its assignment. Toggle state is not saved.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, FunctionSlotParameters, enable, 0),
    // @Param: TYPE
    // @DisplayName: Function action
    // @Description: Built-in action requested by this slot. Slots selecting the same action combine with OR, in addition to the action's master enable. Input arrives through RTn_DST=201..210.
    // @Values: 0:Unused,1:VRS,2:Auto heading,3:Bow thruster enable,4:Stern thruster enable
    // @User: Standard
    AP_GROUPINFO("TYPE", 2, FunctionSlotParameters, type, 0),
    // @Param: MODE
    // @DisplayName: Function trigger mode
    // @Description: Level requests the action while LOW<=input<=HIGH. Toggle changes the request once per entry, after a valid outside sample. Loss, disable, configuration or input-route changes clear the latch. Requests take effect on the next control cycle.
    // @Values: 0:Level,1:Toggle on entry
    // @User: Standard
    AP_GROUPINFO("MODE", 3, FunctionSlotParameters, mode, 0),
    // @Param: LOW
    // @DisplayName: Function activation lower bound
    // @Description: Inclusive activation interval lower bound. Must not exceed HIGH. Invalid settings inhibit the slot rather than being clamped.
    // @Units: us
    // @Range: 800 2200
    // @User: Standard
    AP_GROUPINFO("LOW", 4, FunctionSlotParameters, low, 1700),
    // @Param: HIGH
    // @DisplayName: Function activation upper bound
    // @Description: Inclusive activation interval upper bound. Must be at least LOW. Function routes preserve 800..2200 us without the normal actuator-route clamp.
    // @Units: us
    // @Range: 800 2200
    // @User: Standard
    AP_GROUPINFO("HIGH", 5, FunctionSlotParameters, high, 2000),
    AP_GROUPEND
};

#define FUNCTION_SLOT(n) AP_SUBGROUPINFO(slots[(n)-1], #n "_", n, FunctionParameters, FunctionSlotParameters)
const AP_Param::GroupInfo FunctionParameters::var_info[] = {
    FUNCTION_SLOT(1),
    FUNCTION_SLOT(2),
    FUNCTION_SLOT(3),
    FUNCTION_SLOT(4),
    FUNCTION_SLOT(5),
    FUNCTION_SLOT(6),
    FUNCTION_SLOT(7),
    FUNCTION_SLOT(8),
    FUNCTION_SLOT(9),
    FUNCTION_SLOT(10),
    // @Param: _TEL_HZ
    // @DisplayName: Function telemetry rate
    // @Description: Rate for VFN slot input/status/error and effective master-gated action enables. Independent of control and existing controller telemetry. Zero disables the stream. No stream is sent until a function is configured.
    // @Units: Hz
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("_TEL_HZ", 11, FunctionParameters, tel_hz, 1),
    AP_GROUPEND
};
#undef FUNCTION_SLOT

FunctionSettings FunctionSlotParameters::values() const
{
    FunctionSettings s;
    s.enable = enable.get();
    s.type = type.get();
    s.mode = mode.get();
    s.low = low.get();
    s.high = high.get();
    return s;
}

void FunctionParameters::values(FunctionSettings (&settings)[Functions::count]) const
{
    for (uint8_t n = 0; n < Functions::count; n++) {
        settings[n] = slots[n].values();
    }
}

} // namespace Vektor2
