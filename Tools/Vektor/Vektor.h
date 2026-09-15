#pragma once

#include "Config.h"
#include "Vektor_AssignmentMatrix.h"
#include "Vektor_Attitude.h"
#include "Vektor_Capability.h"
#include "Vektor_Parameters.h"
#include "Vektor_PwmInput.h"
#include "Vektor_PwmOutput.h"
#include "Vektor_Rcin.h"
#include "Vektor_Runtime.h"
#include "Vektor_SerialProtocol.h"

#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

#if VEKTOR_ATTITUDE_ENABLED
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#if HAL_LOGGING_ENABLED
#include <AP_Logger/AP_Logger.h>
#endif
#endif

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
    static constexpr uint16_t k_format_version = 3;

    void setup();
    void loop();

    Parameters g;
    VektorRCChannels rc_channels;
    AssignmentMatrix assignments;
    PwmInput pwm_input;
    PwmOutput pwm_output;
#if VEKTOR_ATTITUDE_ENABLED
    AP_BoardConfig board_config;
    AP_InertialSensor ins;
    Compass compass;
    AP_GPS gps;
    AP_Baro barometer;
#if HAL_LOGGING_ENABLED
    AP_Logger logger;
#endif
    AP_AHRS ahrs;
#endif
    static const AP_Param::Info var_info[];

private:
    enum class SourceKind : uint8_t {
        NONE,
        RCIN,
        PWMIN,
        VSP_SERVO_A,
        VSP_SERVO_B,
    };

    struct CompiledSource {
        SourceKind kind = SourceKind::NONE;
        uint8_t channel = 0;

        CompiledSource() = default;
        constexpr CompiledSource(SourceKind source_kind, uint8_t source_channel) :
            kind(source_kind),
            channel(source_channel)
        {}
    };

    void load_parameters();
    void setup_rcin_uart();
    void setup_attitude();
    void update_attitude(uint64_t now_us);
    void update_rcin(uint64_t now_us);
    void apply_vsp_inputs();
    void apply_pwm_outputs();
    void refresh_compiled_routes();
    CompiledSource compile_source(uint32_t destination_id) const;
    SignalSample<float> sample_from(const CompiledSource &source) const;

    AP_Param param_loader{var_info};
    // The ChibiOS RCOutput driver consults the SRV channel registry during
    // initialization even though Vektor owns PWM calibration itself.
    SRV_Channels _servo_channels;
    const BoardCapability *_active_capability;
    SerialRoleManager _serial_roles;
    RuntimeState _runtime;
    AttitudeSource _attitude;
    RcinSource _rcin;
    VspComponent _vsp;
    SignalSample<float> _pwm_commands[PwmOutput::max_channels] {};
    CompiledSource _vsp_input_sources[2] {};
    CompiledSource _pwm_output_sources[PwmOutput::max_channels] {};
    uint32_t _compiled_assignment_revision = 0;
    bool _compiled_routes_valid = false;
    // USB recovery/configuration plus at most one role-assigned UART.
    SerialProtocol _serial_protocol[2];
#if VEKTOR_ATTITUDE_ENABLED
    uint64_t _last_compass_update_us = 0;
    bool _attitude_initialized = false;
#endif
};

extern App vektor;

} // namespace Vektor

void setup();
void loop();
