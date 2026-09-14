/*
   Vektor standalone ArduPilot application scaffold.

   This proves the Waf target, application entry point, board capability
   inventory, parameter store, and the first Vektor Serial Protocol surface.
 */

#include "Vektor.h"
#include "Vektor_SerialCatalog.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

#include <math.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace Vektor {

App vektor;

void App::setup()
{
    load_parameters();
    _active_capability = &default_capability_for_build();
    _runtime.init(default_service_rate_hz, AP_HAL::micros64());
    setup_attitude();
    _rcin.reset(uint32_t(g.rcin_timeout_ms.get()) * 1000U);
    setup_rcin_uart();
    pwm_output.reset();
    pwm_output.init(_active_capability->pwm_outputs, 0);
    pwm_input.reset();
    pwm_input.init();
    pwm_output.set_reserved_channel_mask(pwm_input.reserved_output_mask());
    assignments.configure_channel_limits(PwmInput::max_channels,
                                         _active_capability->pwm_outputs,
                                         pwm_input.reserved_output_mask());
    assignments.load_persistent();
    _vsp.reset();
    _serial_protocol.init(hal.serial(0),
                          *_active_capability,
                          g,
                          _runtime,
                          _attitude,
                          _vsp,
                          _rcin,
                          pwm_input,
                          pwm_output,
                          assignments);
}

void App::loop()
{
    const uint64_t now_us = AP_HAL::micros64();
    _runtime.begin_loop(now_us);
    update_attitude(now_us);
    update_rcin(now_us);
    pwm_input.update(now_us);
    apply_vsp_inputs();
    _vsp.update(now_us);
    apply_pwm_outputs();
    _serial_protocol.update();
    _runtime.end_loop(AP_HAL::micros64());
    hal.scheduler->delay(1000U / default_service_rate_hz);
}

void App::setup_attitude()
{
    _attitude.reset();
#if VEKTOR_ATTITUDE_ENABLED
    if (!capability_has(*_active_capability, CAP_ONBOARD_IMU)) {
        return;
    }

    board_config.init();
    ins.init(attitude_update_rate_hz);
    compass.init();
    ahrs.set_ekf_type(AP_AHRS::EKFType::DCM);
    ahrs.init();
    ahrs.set_fly_forward(false);
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::GROUND);
    ahrs.reset();
    _last_compass_update_us = 0;
    _attitude_initialized = true;
#endif
}

void App::update_attitude(uint64_t now_us)
{
#if VEKTOR_ATTITUDE_ENABLED
    if (!_attitude_initialized) {
        return;
    }

    ins.update();
    if (_last_compass_update_us == 0 ||
        now_us - _last_compass_update_us >= 100000U) {
        (void)compass.read();
        _last_compass_update_us = now_us;
    }
    ahrs.update(true);

    ::Quaternion quaternion;
    ahrs.get_quat_body_to_ned(quaternion);
    const Vector3f &gyro = ahrs.get_gyro();
    const AttitudeValue value {
        ahrs.get_roll_deg(),
        ahrs.get_pitch_deg(),
        ahrs.get_yaw_deg(),
        quaternion.q1,
        quaternion.q2,
        quaternion.q3,
        quaternion.q4,
        gyro.x,
        gyro.y,
        gyro.z,
    };
    _attitude.ingest(value, now_us, ahrs.healthy());
#else
    (void)now_us;
#endif
}

void App::setup_rcin_uart()
{
#if AP_RCPROTOCOL_ENABLED
    const int16_t port = g.rcin_port.get();
    if (port <= 0 || port > rcin_port_max || AP::RC().has_uart()) {
        return;
    }
    AP_HAL::UARTDriver *uart = hal.serial(uint8_t(port));
    if (uart != nullptr) {
        AP::RC().add_uart(uart);
    }
#endif
}

void App::update_rcin(uint64_t now_us)
{
    int16_t timeout_ms = g.rcin_timeout_ms.get();
    if (timeout_ms < rcin_timeout_ms_min ||
        timeout_ms > rcin_timeout_ms_max) {
        timeout_ms = default_rcin_timeout_ms;
    }
    _rcin.set_timeout_us(uint32_t(timeout_ms) * 1000U);

#if AP_RCPROTOCOL_ENABLED
    AP::RC().update();
#endif

    if (hal.rcin != nullptr && hal.rcin->new_input()) {
        uint16_t pwm[RcinSource::max_channels] {};
        float normalized[RcinSource::max_channels] {};
        const uint8_t available = hal.rcin->num_channels();
        const uint8_t count = available > RcinSource::max_channels ?
                              RcinSource::max_channels : available;
        if (count != 0) {
            hal.rcin->read(pwm, count);
        }
        for (uint8_t i = 0; i < count; i++) {
            RC_Channel *channel = rc_channels.channel(i);
            if (channel == nullptr ||
                pwm[i] < RC_INPUT_MIN_PULSEWIDTH ||
                pwm[i] > RC_INPUT_MAX_PULSEWIDTH) {
                normalized[i] = NAN;
                continue;
            }
            channel->set_radio_in(pwm[i]);
            normalized[i] = channel->norm_input_dz();
        }
        bool failsafe = false;
#if AP_RCPROTOCOL_ENABLED
        failsafe = AP::RC().failsafe_active();
#endif
        _rcin.ingest_normalized(normalized,
                                pwm,
                                count,
                                failsafe,
                                now_us);
    }
    _rcin.update(now_us);
}

SignalSample<float> App::assigned_float(uint32_t destination_id) const
{
    const AssignmentMatrix::Entry *route =
        assignments.to_destination(destination_id);
    if (route == nullptr) {
        return {};
    }

    const FieldDescriptor *source =
        schema_registry().field_by_id(route->source_output_id);
    uint8_t channel_index = 0;
    if (source == nullptr) {
        return {};
    }

    const SignalSample<float> *sample = nullptr;
    if (rcin_channel_for_slot(source->slot, channel_index)) {
        sample = _rcin.channel(channel_index);
    } else if (pwmin_channel_for_slot(source->slot, channel_index)) {
        sample = pwm_input.channel(channel_index);
    } else if (source->slot == FieldSlot::VSP_SERVO_A) {
        sample = &_vsp.servo_a();
    } else if (source->slot == FieldSlot::VSP_SERVO_B) {
        sample = &_vsp.servo_b();
    }
    return sample == nullptr ? SignalSample<float> {} : *sample;
}

void App::apply_vsp_inputs()
{
    const uint32_t x_id =
        SerialCatalog::Input::VSP_X.id;
    const uint32_t y_id =
        SerialCatalog::Input::VSP_Y.id;
    _vsp.set_inputs(assigned_float(x_id), assigned_float(y_id));
}

void App::apply_pwm_outputs()
{
    for (uint8_t channel = 0; channel < PwmOutput::max_channels; channel++) {
        _pwm_commands[channel] = {};
    }
    uint32_t assigned_mask = 0;
    const uint8_t count = MIN(_active_capability->pwm_outputs,
                              PwmOutput::max_channels);
    for (uint8_t channel = 0; channel < count; channel++) {
        const uint32_t destination_id =
            SerialCatalog::Input::PWMOUT_CHANNELS[channel]->id;
        if (assignments.to_destination(destination_id) != nullptr) {
            assigned_mask |= 1U << channel;
            _pwm_commands[channel] = assigned_float(destination_id);
        }
    }
    pwm_output.update(_pwm_commands, count, assigned_mask);
}

} // namespace Vektor

void setup()
{
    Vektor::vektor.setup();
}

void loop()
{
    Vektor::vektor.loop();
}

AP_HAL_MAIN();
