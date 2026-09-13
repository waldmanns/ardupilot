#pragma once

#include "Config.h"
#include "Vektor_Capability.h"
#include "Vektor_Parameters.h"
#include "Vektor_Runtime.h"
#include "Vektor_SerialProtocol.h"

#include <stdint.h>

namespace Vektor {

class App {
public:
    static constexpr uint16_t k_format_version = 1;

    void setup();
    void loop();

    Parameters g;
    static const AP_Param::Info var_info[];

private:
    void load_parameters();

    AP_Param param_loader{var_info};
    const BoardCapability *_active_capability;
    RuntimeState _runtime;
    VspComponent _vsp;
    SerialProtocol _serial_protocol;
};

extern App vektor;

} // namespace Vektor

void setup();
void loop();
