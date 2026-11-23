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
    // @Description: Built-in action. 7=NavMode REQUIRES a three-position RC switch: low Free, middle Heading, high DP. One level-only slot and routed input; +/-50us around LOW/HIGH holds the previous mode (no new action). Old 5/6 actions are retired and fail off. Input via RTn_DST=201..210.
    // @Values: 0:Unused,1:VRS,2:Auto heading enable,3:Bow thruster enable,4:Stern thruster enable,7:NavMode switch
    // @User: Standard
    AP_GROUPINFO("TYPE", 2, FunctionSlotParameters, type, 0),
    // @Param: MODE
    // @DisplayName: Function trigger mode
    // @Description: Standard actions 0=Level (inclusive interval), 1=Toggle. NavMode type 7 requires 0=Level and a THREE-position switch, not a two-position switch. LOW/HIGH define transition centers; in +/-50us gaps no mode change. RC loss and errors select Free.
    // @Values: 0:Level,1:Toggle on entry
    // @User: Standard
    AP_GROUPINFO("MODE", 3, FunctionSlotParameters, mode, 0),
    // @Param: LOW
    // @DisplayName: Function activation lower bound
    // @Description: Normal actions: inclusive lower activation bound. NavMode three-position switch: Free below LOW-50us, no change from LOW-50 through LOW+49, Heading from LOW+50. Example LOW=1300 for 1000/1500/2000us switch.
    // @Units: us
    // @Range: 800 2200
    // @User: Standard
    AP_GROUPINFO("LOW", 4, FunctionSlotParameters, low, 1700),
    // @Param: HIGH
    // @DisplayName: Function activation upper bound
    // @Description: Normal actions: inclusive upper activation bound. NavMode three-position switch: Heading below HIGH-50us, no change from HIGH-50 through HIGH+49, DP from HIGH+50. Example HIGH=1700 for 1000/1500/2000us switch.
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
