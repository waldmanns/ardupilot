#pragma once

#include "Config.h"
#include "Vektor_AssignmentMatrix.h"
#include "Vektor_Capability.h"
#include "Vektor_Parameters.h"
#include "Vektor_PwmInput.h"
#include "Vektor_PwmOutput.h"
#include "Vektor_Rcin.h"
#include "Vektor_Runtime.h"
#include "Vektor_SerialProtocol.h"

#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

#include <stdint.h>

namespace Vektor {

class VektorRCChannel : public RC_Channel {};

class VektorRCChannels : public RC_Channels {
public:
    RC_Channel *channel(uint8_t channel_number) override
    {
        return channel_number < NUM_RC_CHANNELS ?
               &obj_channels[channel_number] : nullptr;
    }

    VektorRCChannel obj_channels[NUM_RC_CHANNELS];

private:
    int8_t flight_mode_channel_number() const override { return -1; }
};

class App {
public:
    static constexpr uint16_t k_format_version = 2;

    void setup();
    void loop();

    Parameters g;
    VektorRCChannels rc_channels;
    AssignmentMatrix assignments;
    PwmInput pwm_input;
    PwmOutput pwm_output;
    static const AP_Param::Info var_info[];

private:
    void load_parameters();
    void setup_rcin_uart();
    void update_rcin(uint64_t now_us);
    void apply_vsp_inputs();
    void apply_pwm_outputs();
    SignalSample<float> assigned_float(uint32_t destination_id) const;

    AP_Param param_loader{var_info};
    // The ChibiOS RCOutput driver consults the SRV channel registry during
    // initialization even though Vektor owns PWM calibration itself.
    SRV_Channels _servo_channels;
    const BoardCapability *_active_capability;
    RuntimeState _runtime;
    RcinSource _rcin;
    VspComponent _vsp;
    SignalSample<float> _pwm_commands[PwmOutput::max_channels] {};
    SerialProtocol _serial_protocol;
};

extern App vektor;

} // namespace Vektor

void setup();
void loop();
