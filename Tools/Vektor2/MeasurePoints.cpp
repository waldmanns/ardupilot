#include "MeasurePoints.h"

#include <GCS_MAVLink/GCS.h>
#include <cmath>
#include <cstring>

extern const AP_HAL::HAL& hal;

namespace Vektor2 {

const AP_Param::GroupInfo MeasurePoints::var_info[] = {
    // @Param: TEL_HZ
    // @DisplayName: Measurement point telemetry rate
    // @Description: Rate per ADC measurement point in NAMED_VALUE_FLOAT messages, in volts. Zero disables measurement telemetry. Requires board-defined measurement points. Values above 50 are limited to 50 Hz.
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TEL_HZ", 1, MeasurePoints, _tel_hz, 0),
    AP_GROUPEND
};

MeasurePoints::MeasurePoints()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#ifdef HAL_VEKTOR2_MEASURE_POINTS
static const struct {
    const char* name;
    uint8_t pin;
    float multiplier;
} points[] = { HAL_VEKTOR2_MEASURE_POINTS };
#endif

void MeasurePoints::init()
{
#ifdef HAL_VEKTOR2_MEASURE_POINTS
    static_assert(ARRAY_SIZE(points) == point_count, "Expected four measurement points");
    for (uint8_t i = 0; i < point_count; i++) {
        _sources[i] = hal.analogin->channel(points[i].pin);
    }
#endif
}

void MeasurePoints::update(uint32_t now_ms)
{
#if defined(HAL_VEKTOR2_MEASURE_POINTS) && HAL_GCS_ENABLED
    const int16_t rate_hz = _tel_hz.get();
    if (rate_hz <= 0) {
        memset(_pending, 0, sizeof(_pending));
        _sent_once = false;
        return;
    }

    static_assert(MAVLINK_COMM_NUM_BUFFERS <= ARRAY_SIZE(_pending), "One mask per MAVLink channel");
    uint8_t channels = 0;
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

    const uint16_t bounded_hz = rate_hz > 50 ? 50 : uint16_t(rate_hz);
    // Round up so the actual rate never exceeds the requested rate.
    const uint32_t period_ms = (1000U + bounded_hz - 1U) / bounded_hz;
    if (!_sent_once || now_ms - _snapshot_ms >= period_ms) {
        _snapshot_ms = now_ms;
        _sent_once = true;
        for (uint8_t i = 0; i < point_count; i++) {
            _values[i] = _sources[i] == nullptr ? NAN :
                         _sources[i]->voltage_average() * points[i].multiplier;
        }
        for (uint8_t i = 0; i < ARRAY_SIZE(_pending); i++) {
            _pending[i] = (channels & (1U << i)) ? (1U << point_count) - 1U : 0;
        }
    }

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
        for (uint8_t i = 0; i < point_count && pending; i++) {
            const uint8_t bit = 1U << i;
            if (!(pending & bit)) {
                continue;
            }
            if (!HAVE_PAYLOAD_SPACE(channel, NAMED_VALUE_FLOAT)) {
                break; // Retry remaining points next loop without blocking PWM.
            }
            mavlink_named_value_float_t value{};
            value.time_boot_ms = _snapshot_ms;
            static_assert(sizeof(value.name) >= sizeof("MP_V_VCAN") - 1, "MAVLink key fits");
            strncpy(value.name, points[i].name, sizeof(value.name));
            value.value = _values[i];
            mavlink_msg_named_value_float_send_struct(channel, &value);
            pending &= ~bit;
        }
    }
#else
    (void)now_ms;
#endif
}

} // namespace Vektor2
