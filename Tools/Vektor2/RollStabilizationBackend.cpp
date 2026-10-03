#include "RollStabilizationBackend.h"
#include "Telemetry.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <RC_Channel/RC_Channel.h>

namespace Vektor2 {

void RollStabilizationBackend::update(const RollStabilizationParameters& params,
                                     const RollDriveParameters& first,
                                     const RollDriveParameters& second)
{
    // Disabled fast path: no AHRS/gyro reads, clock reads, parameter snapshots,
    // filters, trigonometry or added telemetry. Clear state only on transition.
    if (!params.enabled()) {
        if (_controller.enabled()) {
            const RollDriveSettings unused[2] {};
            _controller.update(0, RollSettings{}, unused, RollSample{});
        }
        return;
    }

    const auto& ahrs = AP::ahrs();
    const auto& ins = AP::ins();
    nav_filter_status status{};
    RollSample sample;
    sample.roll_deg = ahrs.get_roll_deg();
    sample.rate_dps = degrees(ahrs.get_gyro().x);
    // Only attitude is needed: do not require a GPS position or absolute yaw.
    // INS is updated immediately before routing; check both health and age so
    // a stale sensor snapshot cannot produce a fresh correction after a stall.
    sample.attitude_valid = ahrs.get_filter_status(status) && status.flags.attitude &&
                            ins.healthy() &&
                            uint32_t(AP_HAL::micros() - ins.get_last_update_usec()) <= 100000U;
    // Routing can chain components whose old limiter maps missing (zero) input
    // to a nonzero output. Gate on the original RC/override source as well as
    // checking each driver's three routed inputs before applying a correction.
    sample.control_valid = rc().has_valid_input();
    const RollDriveSettings drives[2] {first.values(), second.values()};
    _controller.update(AP_HAL::millis(), params.values(), drives, sample);
}

void RollStabilizationBackend::send_telemetry(const VspTelemetry& telemetry) const
{
    if (!_controller.enabled()) {
        return;
    }
    telemetry.send_float("RS_ROLL", _controller.roll());
    telemetry.send_float("RS_RATE", _controller.rate());
    telemetry.send_float("RS_REQ", _controller.demand());
    telemetry.send_float("RS_STATE", float(_controller.state()));
    telemetry.send_float("RS_INHIB", float(_controller.inhibited()));
    // Flags and state have small exact integer values even in NAMED_VALUE_FLOAT.
    // Keeping the existing transport also makes them visible in named-float GCS
    // plots. The cadence is VSP_TEL_HZ; no sends occur in the control algorithm.
    const auto& first = _controller.status(0);
    const auto& second = _controller.status(1);
    telemetry.send_float("V1_RS_CMD", first.command);
    telemetry.send_float("V2_RS_CMD", second.command);
    telemetry.send_float("V1_RS_HRM", first.headroom);
    telemetry.send_float("V2_RS_HRM", second.headroom);
    telemetry.send_float("V1_RS_DUT", first.duty);
    telemetry.send_float("V2_RS_DUT", second.duty);
    telemetry.send_float("V1_RS_LIM", float(first.limited));
    telemetry.send_float("V2_RS_LIM", float(second.limited));
}

} // namespace Vektor2
