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
        reset();
        return false;
    }

    // Bound unexpected parameter values so telemetry cannot saturate the link.
    const uint16_t bounded_hz = rate_hz > 50 ? 50 : uint16_t(rate_hz);
    // Ceiling prevents rates such as 30 Hz from being sent faster than requested.
    const uint32_t period_ms = (1000U + bounded_hz - 1U) / bounded_hz;
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
    // Decode the final component outputs, including heading/VRS corrections.
    // Invalid estimates are NaN, never plausible zero thrust.
    const VspMapper::Config configs[] = {logic.vsp1_params.mapper_config(),
                                         logic.vsp2_params.mapper_config()};
    const uint16_t* outputs[] = {vsp1_outputs, vsp2_outputs};
    const char* direction_keys[] = {"VSP1_TDIR", "VSP2_TDIR"};
    const char* intensity_keys[] = {"VSP1_TMAG", "VSP2_TMAG"};
    for (uint8_t n = 0; n < 2; n++) {
        const auto estimate = VspMapper::estimate(configs[n], outputs[n][0], outputs[n][1]);
        // This is rod excursion, not RPM-dependent force or control readiness.
        // RPM may be routed directly to a motor instead of through the VSP.
        const ComponentId id = ComponentId(n);
        const bool valid = logic.input_valid(id, 0) && logic.input_valid(id, 1) && estimate.valid;
        send_float(direction_keys[n], valid ? estimate.direction_deg : NAN);
        send_float(intensity_keys[n], valid ? estimate.intensity : NAN);
        send_float(n == 0 ? "VSP1_VST" : "VSP2_VST", uint8_t(logic.vector_state(n)));
        send_float(n == 0 ? "VSP1_AUTH" : "VSP2_AUTH", logic.vsp_authority(n));
    }
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
        for (uint8_t n = 0; n < 8; n++) {
            _pending[n] = (channels & (1U << n)) ? ((1UL << 31) - 1) : 0;
        }
    }
#if HAL_GCS_ENABLED
    static_assert(Functions::count * 3 + 1 == 31, "One bit per function telemetry field");
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
        for (uint8_t field = 0; field < Functions::count * 3 + 1 && pending; field++) {
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
                strncpy(value.name, "VFN_VRS", sizeof(value.name));
                value.value = float(_roll_enabled);
            }
            mavlink_msg_named_value_float_send_struct(channel, &value);
            pending &= ~bit;
        }
    }
#endif
}

void DPTelemetry::update(uint32_t now_ms, int16_t rate_hz,
                         const NavigationDP::Output& dp,
                         const NavigationHeading::Output& heading)
{
    if (!ready(now_ms, rate_hz)) { return; }

    // NAMED_VALUE_FLOAT uses 10-byte names and cannot encode a full E7
    // coordinate accurately as one float. Exact target / actual E7 positions
    // are instead available through VTEL sources 240..243; existing standard
    // MAVLink GLOBAL_POSITION_INT reports current EKF position.
    const float no = NAN;
    // A MAVLink float cannot retain 1e-7 degree precision at ordinary
    // latitude magnitudes. Split target coordinate E7 into an integer degree
    // part and a signed E7 remainder (each exactly representable as float).
    // Reconstruct: lat_e7 = DP_TLAT_I * 1e7 + DP_TLAT_F, likewise longitude.
    send_float("DP_TLAT_I", dp.target_valid ? float(dp.target.lat_e7 / 10000000) : no);
    send_float("DP_TLAT_F", dp.target_valid ? float(dp.target.lat_e7 % 10000000) : no);
    send_float("DP_TLON_I", dp.target_valid ? float(dp.target.lon_e7 / 10000000) : no);
    send_float("DP_TLON_F", dp.target_valid ? float(dp.target.lon_e7 % 10000000) : no);
    send_float("DP_DIST", dp.target_valid && dp.actual_valid ? dp.distance_m : no);
    send_float("DP_BRG", dp.target_valid && dp.actual_valid ? dp.bearing_deg : no);
    send_float("DP_ERR_N", dp.target_valid && dp.actual_valid ? dp.error_n_m : no);
    send_float("DP_ERR_E", dp.target_valid && dp.actual_valid ? dp.error_e_m : no);
    send_float("DP_VN", dp.valid ? dp.velocity_n_ms : no);
    send_float("DP_VE", dp.valid ? dp.velocity_e_ms : no);
    send_float("DP_CMD_X", dp.valid ? dp.surge : no);
    send_float("DP_CMD_Y", dp.valid ? dp.sway : no);
    send_float("DP_CMD_N", dp.valid ? dp.demand_n : no);
    send_float("DP_CMD_E", dp.valid ? dp.demand_e : no);
    send_float("DP_CMD_MAG", dp.valid ? dp.demand_magnitude : no);
    send_float("DP_CMD_BRG", dp.valid ? dp.demand_bearing_deg : no);
    send_float("HDG_TGT", heading.target_valid ? heading.target_deg : no);
    send_float("HDG_ACT", heading.actual_valid ? heading.actual_deg : no);
    send_float("HDG_ERR", heading.target_valid && heading.actual_valid ? heading.error_deg : no);
    send_float("HDG_CMD", heading.valid ? heading.yaw_demand : no);
    // Explicit calculation-valid indicators for board-only DP experiments:
    // neither signal implies propulsion ownership or an ACTIVE NAV_STATE.
    send_float("DP_CALC", dp.valid ? 1.0f : 0.0f);
    send_float("HDG_CALC", heading.valid ? 1.0f : 0.0f);
}

void NavigationTelemetry::update(uint32_t now_ms, const NavigationSupervisor::Status& status)
{
    const bool changed = !_sent || _previous.state != status.state ||
                         _previous.reason_bits != status.reason_bits;
    // 2 Hz periodic refresh and bounded state-change notification. Both values
    // are sampled from one control-cycle status, never independent controllers.
    if (_sent && now_ms - _last_sent_ms < (changed ? 100U : 500U)) {
        return;
    }
    _sent = true;
    _last_sent_ms = now_ms;
    _previous = status;
    send_float("NAV_STATE", float(uint8_t(status.state)));
    send_float("NAV_RBITS", float(status.reason_bits));
}

} // namespace Vektor2
