#include "Telemetry.h"

#include "Logic.h"
#include "VSP1.h"
#include "VSP2.h"

#include <GCS_MAVLink/GCS.h>

namespace Vektor2 {

bool NamedFloatTelemetry::ready(uint32_t now_ms, int16_t rate_hz)
{
    if (rate_hz <= 0) {
        return false;
    }

    // Bound unexpected parameter values so telemetry cannot saturate the link.
    const uint16_t bounded_hz = rate_hz > 50 ? 50 : uint16_t(rate_hz);
    const uint32_t period_ms = 1000U / bounded_hz;
    if (_sent_once && now_ms - _last_send_ms < period_ms) {
        return false;
    }
    _last_send_ms = now_ms;
    _sent_once = true;

    return true;
}

void VspTelemetry::update(uint32_t now_ms, int16_t rate_hz, const Logic& logic)
{
    if (!ready(now_ms, rate_hz)) {
        return;
    }

    const uint16_t vsp1_outputs[] = {
        logic.output(ComponentId::VSP1, 0),
        logic.output(ComponentId::VSP1, 1),
    };
    const uint16_t vsp2_outputs[] = {
        logic.output(ComponentId::VSP2, 0),
        logic.output(ComponentId::VSP2, 1),
    };
    Components::vsp1_emit_telemetry(vsp1_outputs, *this);
    Components::vsp2_emit_telemetry(vsp2_outputs, *this);
    logic.vrs.send_telemetry(*this); // no additional messages when VRS is disabled
}

void ThrusterTelemetry::update(uint32_t now_ms, int16_t rate_hz, const Logic& logic)
{
    if (!ready(now_ms, rate_hz)) {
        return;
    }

    send_float("THRBOW_PWM", float(logic.output(ComponentId::ThrusterBow, 0)));
    send_float("THRSTN_PWM", float(logic.output(ComponentId::ThrusterStern, 0)));
}

void NamedFloatTelemetry::send_float(const char* name, float value) const
{
#if HAL_GCS_ENABLED
    // GCS checks each active link's payload space and skips a full link.
    gcs().send_named_float(name, value);
#else
    (void)name;
    (void)value;
#endif
}

} // namespace Vektor2
