#pragma once

#include "Functions.h"
#include <AP_Param/AP_Param.h>

namespace Vektor2 {

class FunctionSlotParameters {
public:
    FunctionSlotParameters() { AP_Param::setup_object_defaults(this, var_info); }
    static const AP_Param::GroupInfo var_info[];
    FunctionSettings values() const;

private:
    AP_Int8 enable;
    AP_Int8 type;
    AP_Int8 mode;
    AP_Int16 low;
    AP_Int16 high;
};

class FunctionParameters {
public:
    FunctionParameters() { AP_Param::setup_object_defaults(this, var_info); }
    static const AP_Param::GroupInfo var_info[];
    void values(FunctionSettings (&settings)[Functions::count]) const;
    int16_t telemetry_hz() const { return tel_hz.get(); }

private:
    FunctionSlotParameters slots[Functions::count];
    AP_Int16 tel_hz;
};

} // namespace Vektor2
