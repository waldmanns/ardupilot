#include "RpmInputs.h"
#include "Logic.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

namespace Vektor2 {

#ifdef HAL_VEKTOR2_RPM_PINS
static constexpr uint8_t pins[] = { HAL_VEKTOR2_RPM_PINS };
static constexpr uint32_t min_period_us = 1000;
static constexpr uint32_t max_period_us = 5000000; // 12 RPM with one magnet
static constexpr uint32_t min_stop_timeout_us = 250000;

void RpmInputs::Input::on_falling_edge(uint8_t, bool, uint32_t timestamp_us)
{
    const uint32_t interval_us = timestamp_us - last_edge_us;
    if (have_edge && interval_us < min_period_us) {
        return;
    }
    period_us = have_edge && interval_us <= max_period_us ? interval_us : 0;
    last_edge_us = timestamp_us;
    have_edge = true;
}
#endif

void RpmInputs::init()
{
#ifdef HAL_VEKTOR2_RPM_PINS
    static_assert(ARRAY_SIZE(pins) == ARRAY_SIZE(_inputs), "Expected two RPM input pins");
    for (uint8_t i = 0; i < ARRAY_SIZE(pins); i++) {
        auto* input = &_inputs[i];
        input->attached = hal.gpio->attach_interrupt(pins[i],
            FUNCTOR_BIND(input, &Input::on_falling_edge, void, uint8_t, bool, uint32_t),
            AP_HAL::GPIO::INTERRUPT_FALLING);
        if (!input->attached) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPM%u input unavailable on GPIO %u", unsigned(i + 1), unsigned(pins[i]));
        }
    }
#endif
}

void RpmInputs::update()
{
#ifdef HAL_VEKTOR2_RPM_PINS
    uint32_t periods_us[ARRAY_SIZE(_inputs)];
    // Take both snapshots atomically with respect to the GPIO callbacks.
    void* irq_state = hal.scheduler->disable_interrupts_save();
    const uint32_t now_us = AP_HAL::micros();
    for (uint8_t i = 0; i < ARRAY_SIZE(_inputs); i++) {
        auto& input = _inputs[i];
        // Allow two missing revolutions, with a minimum timeout at high speed.
        const uint32_t timeout_us = input.period_us == 0 ? max_period_us :
                                    MAX(min_stop_timeout_us, 2U * input.period_us);
        if (input.have_edge && now_us - input.last_edge_us > timeout_us) {
            input.have_edge = false;
            input.period_us = 0;
        }
        periods_us[i] = input.period_us;
    }
    hal.scheduler->restore_interrupts(irq_state);

    for (uint8_t i = 0; i < ARRAY_SIZE(_inputs); i++) {
        _inputs[i].rpm = periods_us[i] == 0 ? 0.0f : 60000000.0f / periods_us[i];
    }
#endif
}

void RpmInputs::send_telemetry(uint32_t now_ms, int16_t telemetry_hz, const Logic& logic)
{
    if (!ready(now_ms, telemetry_hz)) {
        return;
    }
#ifdef HAL_VEKTOR2_RPM_PINS
    const bool magnet1 = hal.gpio->read(pins[0]) == 0;
    const bool magnet2 = hal.gpio->read(pins[1]) == 0;
    send_float("VRPM1_RPM", rpm(0));
    send_float("VRPM2_RPM", rpm(1));
    send_float("VRPM1_IRQ", magnet1);
    send_float("VRPM2_IRQ", magnet2);
#endif
    if (logic.vsp_selected()) {
        send_float("VRPM1_STAT", logic.vsp_rpm_status(0));
        send_float("VRPM2_STAT", logic.vsp_rpm_status(1));
    }
}

float RpmInputs::rpm(uint8_t channel) const
{
    return channel < ARRAY_SIZE(_inputs) ? _inputs[channel].rpm : 0.0f;
}

bool RpmInputs::available(uint8_t channel) const
{
    return channel < ARRAY_SIZE(_inputs) && _inputs[channel].attached;
}

} // namespace Vektor2
