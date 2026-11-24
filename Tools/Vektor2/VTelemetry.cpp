#include "VTelemetry.h"

#include "Vektor2.h"
#include "VTelemetryPropulsion.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <cmath>

extern const AP_HAL::HAL& hal;

namespace Vektor2 {

using namespace VTelemetryCore;

bool VTelemetry::read_source(uint16_t id, Sample& s, void* context)
{
    auto& application = *static_cast<App*>(context);
    const bool gps_fix2d = application.gps.status(0) >= AP_GPS::GPS_OK_FIX_2D;
    const bool gps_fix3d = application.gps.status(0) >= AP_GPS::GPS_OK_FIX_3D;
    switch (id) {
    case 1: if (!gps_fix2d) { return false; } s.number = application.gps.ground_speed(); return true;
    case 2: if (!gps_fix2d) { return false; } s.number = application.gps.ground_speed() * 3.6f; return true;
    case 3: if (!gps_fix2d) { return false; } s.number = application.gps.ground_course(); return true;
    case 4: s.number = application.gps.num_sats(); return true;
    case 5: s.number = uint8_t(application.gps.status(0)); return true;
    case 6: if (!gps_fix2d) { return false; } s.coordinate_e7 = application.gps.location().lat; return true;
    case 7: if (!gps_fix2d) { return false; } s.coordinate_e7 = application.gps.location().lng; return true;
    case 8: if (!gps_fix3d) { return false; } s.number = application.gps.location().alt * 0.01f; return true;
    case 9: {
        Vector3f velocity;
        if (!application.ahrs.get_velocity_NED(velocity)) { return false; }
        s.number = -velocity.z; // NED is positive down; telemetry climb is positive up
        return true;
    }
    case 10: s.number = application.ahrs.get_roll_deg(); return true;
    case 11: s.number = application.ahrs.get_pitch_deg(); return true;
    case 12: s.number = application.ahrs.get_yaw_deg(); return true;
    case 13:
    case 14: {
        Vector3f velocity;
        if (!application.ahrs.get_velocity_NED(velocity)) { return false; }
        s.number = id == 13 ? velocity.x : velocity.y;
        return true;
    }
    case 15: return application.gps.speed_accuracy(s.number);
    case 16: return application.gps.horizontal_accuracy(s.number);
    case 18:
        s.number = float(uint16_t(uint8_t(application.gps.status(0))) |
                         (uint16_t(application.gps.num_sats()) << 8));
        return true;
    default: break;
    }
    if (id >= 270 && id <= 296) {
        return read_propulsion_source(id, s, application.logic);
    }
    if (id >= 20 && id <= 37) {
        const uint8_t channel = uint8_t(id - 20);
        if (!application.rcin.valid() || channel >= application.rcin.channel_count()) { return false; }
        s.number = application.rcin.channel_us(channel);
        return s.number > 0;
    }
    if (id >= 40 && id <= 45) {
        const uint8_t drive = uint8_t((id - 40) / 3);
        const uint8_t port = uint8_t((id - 40) % 3);
        const ComponentId component = drive == 0 ? ComponentId::VSP1 : ComponentId::VSP2;
        if (!application.logic.output_valid(component)) { return false; }
        s.number = application.logic.output(component, port);
        return true;
    }
    if (id == 50 || id == 51) {
        const ComponentId component = id == 50 ? ComponentId::ThrusterBow : ComponentId::ThrusterStern;
        if (!application.logic.output_valid(component)) { return false; }
        s.number = application.logic.output(component, 0);
        return true;
    }
    if (id >= 60 && id <= 63) {
        const uint8_t drive = uint8_t((id - 60) / 2);
        const ComponentId component = drive == 0 ? ComponentId::VSP1 : ComponentId::VSP2;
        if (!application.logic.input_valid(component, 0) || !application.logic.input_valid(component, 1) ||
            !application.logic.output_valid(component)) { return false; }
        const auto config = drive == 0 ? application.logic.vsp1_params.mapper_config() :
                                         application.logic.vsp2_params.mapper_config();
        const auto estimate = VspMapper::estimate(config, application.logic.output(component, 0),
                                                  application.logic.output(component, 1));
        if (!estimate.valid) { return false; }
        s.number = ((id - 60) % 2) == 0 ? estimate.direction_deg : estimate.intensity;
        return true;
    }
    if (id >= 64 && id <= 67 && !application.logic.vsp_selected()) { return false; }
    if (id >= 64 && id <= 65) { s.number = uint8_t(application.logic.vector_state(id - 64)); return true; }
    if (id == 262) { if (!application.logic.vsp_selected()) return false; s.number=application.logic.vsp_alloc_status(); return true; }
    if (id >= 66 && id <= 67) { s.number = application.logic.vsp_authority(id - 66); return true; }
    if (id >= 80 && id <= 82) {
        if (!application.logic.vsp_selected()) { return false; }
        const auto* vrs = application.logic.roll_controller();
        if (id == 80) { s.number = vrs ? uint8_t(vrs->state()) : 0; return true; }
        if (!vrs) { return false; }
        s.number = id == 81 ? vrs->demand() : float(vrs->inhibited());
        return true;
    }
    if (id >= 90 && id <= 91) {
#ifdef HAL_VEKTOR2_RPM_PINS
        s.number = application.rpm_inputs.rpm(id - 90);
        return true;
#else
        return false;
#endif
    }
    if (id >= 100 && id <= 103) {
        return application.measure_points.read_voltage(uint8_t(id - 100), s.number);
    }
    if (id >= 110 && id <= 139) {
        const uint8_t index = uint8_t((id - 110) % Functions::count);
        const auto& status = application.logic.vfn.status(index);
        if (id < 120) { s.number = status.flags; }
        else if (id < 130) { s.number = status.errors; }
        else { s.number = status.pulse_us; }
        return true;
    }
    if (id == 140 || id == 141) {
        const auto& nav = application.nav_supervisor.status();
        s.number = id == 140 ? float(uint8_t(nav.state)) : float(nav.reason_bits);
        return true;
    }
    if (id == 260 || id == 261) {
        s.number = (id == 260 ? application.nav_dp.output().valid :
                    application.nav_heading.output().valid) ? 1.0f : 0.0f;
        return true;
    }
    if (id >= 240 && id <= 259) {
        const auto& dp = application.nav_dp.output();
        const auto& hdg = application.nav_heading.output();
        if (id >= 240 && id <= 243) {
            const bool target = id < 242;
            if (target ? !dp.target_valid : !dp.actual_valid) { return false; }
            const auto& point = target ? dp.target : dp.actual;
            s.coordinate_e7 = (id % 2 == 0) ? point.lat_e7 : point.lon_e7;
            return true;
        }
        if (id == 246 || id == 247 || id == 252 || id == 257) {
            if (id == 246) { if (!hdg.target_valid) { return false; } s.number = hdg.target_deg; }
            if (id == 247) { if (!hdg.actual_valid) { return false; } s.number = hdg.actual_deg; }
            if (id == 252) { if (!hdg.valid) { return false; } s.number = hdg.yaw_demand; }
            if (id == 257) {
                if (!hdg.target_valid || !hdg.actual_valid) { return false; }
                s.number = hdg.error_deg;
            }
            return true;
        }
        if (id >= 244 && id <= 249) {
            if (!dp.target_valid || !dp.actual_valid) { return false; }
            switch (id) {
            case 244: s.number = dp.distance_m; break;
            case 245: s.number = dp.bearing_deg; break;
            case 248: s.number = dp.error_n_m; break;
            case 249: s.number = dp.error_e_m; break;
            default: return false;
            }
            return true;
        }
        if (!dp.valid) { return false; }
        switch (id) {
        case 250: s.number = dp.surge; break;
        case 251: s.number = dp.sway; break;
        case 253: s.number = dp.velocity_n_ms; break;
        case 254: s.number = dp.velocity_e_ms; break;
        case 255: s.number = dp.demand_magnitude; break;
        case 256: s.number = dp.demand_bearing_deg; break;
        case 258: s.number = dp.demand_n; break;
        case 259: s.number = dp.demand_e; break;
        default: return false;
        }
        return true;
    }
    if (id >= 150 && id <= 185) {
        const uint8_t lane = uint8_t(id < 170 ? id - 150 : id - 170);
        if (lane >= Sequencer::lane_count) { return false; }
        const auto& status = application.sequencer.engine.status(lane);
        if (id < 170) { s.number = uint8_t(status.state); return true; }
        if (!status.valid) { return false; }
        s.number = status.pulse;
        return true;
    }
    if (id >= 200 && id <= 231) {
        const uint8_t channel = uint8_t(id - 200);
        if (!application.pwm.enabled(channel)) { return false; }
        s.number = application.pwm.value_us(channel);
        return true;
    }
    return false;
}

void VTelemetry::init(App& application, VTelemetryParameters& parameters, AP_SerialManager& serial)
{
    // The application has already loaded parameters and initialized serial
    // roles, sensors and routing before these threads are allowed to start.
    update(application, parameters, AP_HAL::millis());
    _ibus = serial.find_serial(AP_SerialManager::SerialProtocol_IBUS_Telem, 0);
    _hott = serial.find_serial(AP_SerialManager::SerialProtocol_Hott, 0);
    if (_ibus != nullptr) {
        // Respect the board/user UART options (inversion, pin swap, etc.).
        // This protocol uses the receiver's one-wire sensor socket.
        const uint16_t options = _ibus->get_options() | AP_HAL::UARTDriver::OPTION_HDPLEX;
        if (!_ibus->set_options(options)) {
            _ibus = nullptr;
#if HAL_GCS_ENABLED
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTEL iBUS half-duplex unavailable");
#endif
        } else if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&VTelemetry::ibus_task, void),
                                                 "VTEL_iBUS", 1024,
                                                 AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
            _ibus = nullptr;
#if HAL_GCS_ENABLED
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTEL iBUS task unavailable");
#endif
        }
    }
    if (_hott != nullptr) {
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&VTelemetry::hott_task, void),
                                          "VTEL_HoTT", 1024,
                                          AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
            _hott = nullptr;
#if HAL_GCS_ENABLED
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTEL HoTT task unavailable");
#endif
        }
    }
}

void VTelemetry::update(App& application, VTelemetryParameters& parameters, uint32_t now_ms)
{
    if (_initialized && now_ms - _last_update_ms < 100) { return; }
    _initialized = true;
    _last_update_ms = now_ms;
    int16_t source_ids[slot_count] {};
    const bool changed = parameters.changed(source_ids);
    uint8_t invalid = 0;
    uint8_t first_bad = 0;
    {
        WITH_SEMAPHORE(_sem);
        if (changed) {
            invalid = _router.configure(source_ids);
            if (invalid != 0) {
                for (uint8_t i = 1; i <= slot_count; i++) {
                    if (source_ids[i-1] != 0 && _router.assigned(i) == nullptr) {
                        first_bad = i;
                        break;
                    }
                }
            }
        }
        _router.sample(read_source, &application);
    }
#if HAL_GCS_ENABLED
    if (changed && invalid != 0) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "VTEL SLT%u invalid (%u total), ignored",
                        unsigned(first_bad), unsigned(invalid));
    }
#else
    (void)invalid;
    (void)first_bad;
#endif
}

void VTelemetry::ibus_task()
{
    _ibus->begin(115200);
    while (true) {
        hal.scheduler->delay(1);
        const uint32_t bytes = _ibus->available();
        if (bytes == 0) { continue; }
        if (bytes != 4) {
            // Do not answer malformed or partial receiver polls.
            if (bytes > 4) { _ibus->discard_input(); }
            continue;
        }
        uint8_t request[4];
        if (_ibus->read(request, sizeof(request)) != sizeof(request)) { continue; }
        uint8_t reply[8] {};
        uint8_t count;
        {
            WITH_SEMAPHORE(_sem);
            count = _router.ibus_reply(request, reply);
        }
        if (count != 0 && _ibus->txspace() >= count) {
            _ibus->write(reply, count);
        }
    }
}

void VTelemetry::hott_task()
{
    _hott->begin(19200, 10, 10);
    _hott->set_unbuffered_writes(true);
    while (true) {
        hal.scheduler->delay_microseconds(1500);
        const uint32_t bytes = _hott->available();
        if (bytes < 2) { continue; }
        if (bytes != 2) { _hott->discard_input(); continue; }
        const int16_t protocol = _hott->read();
        const int16_t sensor = _hott->read();
        if (protocol != 0x80 || sensor < 0) { continue; }
        uint8_t frame[44];
        bool enabled;
        {
            WITH_SEMAPHORE(_sem);
            enabled = _router.hott_frame(uint8_t(sensor), frame);
        }
        if (!enabled) { continue; }
        // Binary HoTT requests require a 4 ms initial silence and 1.2 ms
        // spacing between bytes. Use a separate task, never App::loop().
        hal.scheduler->delay_microseconds(4000);
        uint8_t checksum = 0;
        bool sent = true;
        for (uint8_t i = 0; i < sizeof(frame); i++) {
            // Bounded retry: do not stall this worker on a disconnected UART.
            bool wrote = false;
            for (uint8_t retry = 0; retry < 10; retry++) {
                if (_hott->write(frame[i]) == 1) { wrote = true; break; }
                hal.scheduler->delay_microseconds(100);
            }
            if (!wrote) { sent = false; break; }
            checksum += frame[i];
            hal.scheduler->delay_microseconds(1200);
        }
        if (sent) { _hott->write(checksum); }
        hal.scheduler->delay_microseconds(2400);
        _hott->discard_input();
    }
}

} // namespace Vektor2
