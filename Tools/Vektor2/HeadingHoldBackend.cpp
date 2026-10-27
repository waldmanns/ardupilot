#include "HeadingHoldBackend.h"
#include "Telemetry.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

namespace Vektor2 {
void HeadingHoldBackend::update(const HeadingHoldParameters& params, const HeadingDrive (&drives)[2],
                                bool controls_valid, bool routes_valid, uint32_t route_revision,
                                bool permitted, const HeadingAuxiliary& auxiliary)
{
    if (!params.enabled() || !permitted) {
        if (_controller.enabled()) {
            _controller.update(0, HeadingSettings{}, HeadingSample{}, drives);
        }
        return;
    }

    auto& ahrs = AP::ahrs();
    const auto& compass = AP::compass();
    const auto& ins = AP::ins();
    const uint32_t now_ms = AP_HAL::millis();
    float yaw_reset_angle;
    const uint32_t yaw_reset_ms = ahrs.getLastYawResetAngle(yaw_reset_angle);
    HeadingSample sample;
    sample.auxiliary = auxiliary;
    sample.heading_deg = ahrs.get_yaw_deg();
    sample.rate_dps = degrees(ahrs.get_yaw_rate_earth());
    sample.compass_valid = compass.available() && compass.get_num_enabled() > 0 &&
                           compass.healthy() && compass.use_for_yaw() && ahrs.use_compass() &&
                           now_ms - compass.last_update_ms() <= 300;
#if COMPASS_CAL_ENABLED
    sample.compass_valid &= !compass.is_calibrating() && !compass.compass_cal_requires_reboot();
#endif
    nav_filter_status status{};
    sample.attitude_valid = ahrs.get_filter_status(status) && status.flags.attitude &&
                            status.flags.initalized && ins.healthy() &&
                            uint32_t(AP_HAL::micros() - ins.get_last_update_usec()) <= 100000U;
    sample.control_valid = controls_valid;
    sample.routes_valid = routes_valid;
    sample.yaw_reset = _controller.enabled() && yaw_reset_ms != _yaw_reset_ms;
    sample.routes_changed = route_revision != _route_revision;
    _yaw_reset_ms = yaw_reset_ms;
    _route_revision = route_revision;
    _controller.update(now_ms, params.values(), sample, drives);
}

void HeadingHoldBackend::send_telemetry(const HeadingTelemetry& telemetry) const
{
    if (!_controller.enabled()) {
        return;
    }
    const bool holding = _controller.state() == HeadingHold::State::Hold;
    telemetry.send_float("HDG_NOW", _controller.heading());
    telemetry.send_float("HDG_TGT", holding ? _controller.target() : NAN);
    telemetry.send_float("HDG_ERR", holding ? _controller.error() : NAN);
    telemetry.send_float("HDG_RATE", _controller.rate());
    telemetry.send_float("HDG_REQ", _controller.demand());
    telemetry.send_float("HDG_I", _controller.integral());
    telemetry.send_float("HDG_WAIT", _controller.wait_ms() * 0.001f);
    telemetry.send_float("HDG_SCALE", _controller.travel_scale());
    telemetry.send_float("V1_HDG_US", float(_controller.correction_us(0)));
    telemetry.send_float("V2_HDG_US", float(_controller.correction_us(1)));
}

} // namespace Vektor2
