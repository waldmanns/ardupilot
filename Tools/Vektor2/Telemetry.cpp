#include "Telemetry.h"

#include "Logic.h"
#include "VSP1.h"
#include "VSP2.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

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
    send_float("THRBOW_ST", uint8_t(logic.thruster(0).state()));
    send_float("THRSTN_ST", uint8_t(logic.thruster(1).state()));
    send_float("THRBOW_EN", logic.thruster(0).enabled());
    send_float("THRSTN_EN", logic.thruster(1).enabled());
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

void NamedFloatTelemetry::send_string(const char* name, const char* value) const
{
#if HAL_GCS_ENABLED
    gcs().send_named_string(name, value);
#else
    (void)name;
    (void)value;
#endif
}

void FunctionTelemetry::update(uint32_t now_ms, const Logic& logic)
{
    const bool configured = logic.vfn.configured();
    const int16_t rate = logic.vfn_params.telemetry_hz();
    if (rate <= 0) {
        for (auto& pending : _pending) {
            pending = 0;
        }
        return;
    }
    uint8_t channels = 0;
#if HAL_GCS_ENABLED
    static_assert(MAVLINK_COMM_NUM_BUFFERS <= 8, "Function telemetry supports eight channels");
    for (uint8_t n = 0; n < gcs().num_gcs(); n++) {
        const auto* link = gcs().chan(n);
        if (link == nullptr || !link->is_active() || link->is_private()) {
            continue;
        }
#if HAL_HIGH_LATENCY2_ENABLED
        if (link->is_high_latency_link) {
            continue;
        }
#endif
        channels |= 1U << uint8_t(link->get_chan());
    }
#endif
    if ((configured || _was_configured) && ready(now_ms, rate > 10 ? 10 : rate)) {
        // Finish with a clearing snapshot when the last assignment is removed.
        _was_configured = configured;
        _snapshot_ms = now_ms;
        for (uint8_t n = 0; n < Functions::count; n++) {
            _status[n] = logic.vfn.status(n);
        }
        // These are the gates used for this cycle, not the pending requests.
        _roll_enabled = logic.roll_enabled();
        _heading_enabled = logic.heading_enabled();
        for (uint8_t n = 0; n < 8; n++) {
            _pending[n] = (channels & (1U << n)) ? UINT32_MAX : 0;
        }
    }
#if HAL_GCS_ENABLED
    static_assert(Functions::count * 3 + 2 == 32, "One bit per function telemetry field");
    for (uint8_t n = 0; n < gcs().num_gcs(); n++) {
        auto* link = gcs().chan(n);
        if (link == nullptr) {
            continue;
        }
        const auto channel = link->get_chan();
        auto& pending = _pending[uint8_t(channel)];
        if (!(channels & (1U << uint8_t(channel)))) {
            pending = 0;
        }
        WITH_SEMAPHORE(comm_chan_lock(channel));
        for (uint8_t field = 0; field < 32 && pending; field++) {
            const uint32_t bit = uint32_t(1) << field;
            if (!(pending & bit)) {
                continue;
            }
            if (!HAVE_PAYLOAD_SPACE(channel, NAMED_VALUE_FLOAT)) {
                break; // Retry unsent fields next loop; no blocking control IO.
            }
            mavlink_named_value_float_t value{};
            value.time_boot_ms = _snapshot_ms;
            if (field < Functions::count * 3) {
                const uint8_t slot = field / 3;
                const auto& status = _status[slot];
                const uint8_t kind = field % 3;
                hal.util->snprintf(value.name, sizeof(value.name), "VFN%u_%s", unsigned(slot + 1),
                                   kind == 0 ? "IN" : (kind == 1 ? "ST" : "ERR"));
                value.value = kind == 0 ? ((status.flags & Functions::Valid) ? float(status.pulse_us) : NAN) :
                              (kind == 1 ? float(status.flags) : float(status.errors));
            } else {
                const bool roll = field == Functions::count * 3;
                strncpy(value.name, roll ? "VFN_VRS" : "VFN_HDG", sizeof(value.name));
                value.value = roll ? float(_roll_enabled) : float(_heading_enabled);
            }
            mavlink_msg_named_value_float_send_struct(channel, &value);
            pending &= ~bit;
        }
    }
#endif
}

void HeadingTelemetry::update(uint32_t now_ms, const Logic& logic)
{
    if (!logic.vhdg.controller().enabled()) {
        _status_sent = false;
        _pending_state = _pending_inhibit = _pending_reason = 0;
        reset();
        return;
    }
    const int16_t rate_hz = logic.vhdg_params.telemetry_hz();
    const uint32_t status_period_ms = rate_hz < 10 ? 1000U : 500U;
    uint8_t channels = 0;
#if HAL_GCS_ENABLED
    static_assert(MAVLINK_COMM_NUM_BUFFERS <= 8, "Status channel masks are eight bits");
    for (uint8_t n = 0; n < gcs().num_gcs(); n++) {
        const auto* link = gcs().chan(n);
        if (link == nullptr || !link->is_active() || link->is_private()) {
            continue;
        }
#if HAL_HIGH_LATENCY2_ENABLED
        if (link->is_high_latency_link) {
            continue;
        }
#endif
        channels |= 1U << uint8_t(link->get_chan());
    }
#endif
    // Capture one coherent status snapshot at the requested slow cadence.
    // Retry unsent fields per link between ticks instead of dropping a status
    // permanently when its fixed phase coincides with another telemetry burst.
    if (!_status_sent || now_ms - _last_status_ms >= status_period_ms) {
        _last_status_ms = now_ms;
        _status_sent = true;
        const auto& controller = logic.vhdg.controller();
        _state = uint8_t(controller.state());
        _inhibit = controller.inhibited();
        _reason = controller.inhibit_reason();
        _pending_state = _pending_inhibit = _pending_reason = channels;
#if HAL_LOGGING_ENABLED
        if (AP::logger().should_log(1U << 3)) {
            AP::logger().WriteStreaming("VHST", "TimeUS,State,Inhib,Reason", "QBHZ",
                                       AP_HAL::micros64(), _state, _inhibit, _reason);
        }
#endif
    }
    _pending_state &= channels;
    _pending_inhibit &= channels;
    _pending_reason &= channels;
#if HAL_GCS_ENABLED
    for (uint8_t n = 0; n < gcs().num_gcs(); n++) {
        auto* link = gcs().chan(n);
        if (link == nullptr) {
            continue;
        }
        const auto channel = link->get_chan();
        const uint8_t mask = 1U << uint8_t(channel);
        if (!((_pending_state | _pending_inhibit | _pending_reason) & mask)) {
            continue;
        }
        WITH_SEMAPHORE(comm_chan_lock(channel));
        mavlink_named_value_float_t value{};
        value.time_boot_ms = _last_status_ms;
        if ((_pending_state & mask) && HAVE_PAYLOAD_SPACE(channel, NAMED_VALUE_FLOAT)) {
            memcpy(value.name, "HDG_STATE", 9);
            value.value = _state;
            mavlink_msg_named_value_float_send_struct(channel, &value);
            _pending_state &= ~mask;
        }
        if ((_pending_inhibit & mask) && HAVE_PAYLOAD_SPACE(channel, NAMED_VALUE_FLOAT)) {
            memcpy(value.name, "HDG_INHIB", 9);
            value.value = _inhibit;
            mavlink_msg_named_value_float_send_struct(channel, &value);
            _pending_inhibit &= ~mask;
        }
        if ((_pending_reason & mask) && HAVE_PAYLOAD_SPACE(channel, NAMED_VALUE_STRING)) {
            mavlink_named_value_string_t reason{};
            reason.time_boot_ms = _last_status_ms;
            memcpy(reason.name, "HDG_REASON", 10);
            strncpy(reason.value, _reason, sizeof(reason.value) - 1);
            mavlink_msg_named_value_string_send_struct(channel, &reason);
            _pending_reason &= ~mask;
        }
    }
#endif
    if (ready(now_ms, rate_hz)) {
        logic.vhdg.send_telemetry(*this);
    }
}

} // namespace Vektor2
