#pragma once

#include "LibVSP.h"
#include <stdint.h>

namespace Vektor2 {

// This file deliberately depends on neither AHRS nor AP_Param. The controller
// consumes one timestamped sensor/configuration snapshot per routing cycle;
// the same code can therefore be exercised on the host without flight hardware.
struct RollSettings {
    bool enabled = false;
    float p = 0.0f;             // normalized ring displacement / degree
    float d = 0.0f;             // normalized ring displacement / (degree/s)
    float trim_deg = 0.0f;
    float filter_hz = 2.0f;
    float deadband = 0.02f;     // demand threshold; release at half this value
    uint16_t on_ms = 250;
    uint16_t off_ms = 250;
    float slew = 1.0f;          // ring radii / second, applied only to correction
    float maneuver_limit = 0.8f;
};

struct RollDriveSettings {
    int16_t angle_deg = 0;      // POST-limiter axes: +X=0, +Y=90 degrees
    int8_t sign = 0;            // roll moment from this ray: -1, +1, or 0=unused
    float maximum = 0.2f;       // maximum added displacement / ring radius
};

struct RollSample {
    float roll_deg = 0.0f;
    float rate_dps = 0.0f;      // bias-corrected BODY X gyro rate, not d(roll)/dt
    bool attitude_valid = false;
    bool control_valid = false;
};

class RollStabilization {
public:
    enum class State : uint8_t { Disabled, Waiting, Ready, Burst, Recovery };
    enum Inhibit : uint16_t {
        Attitude = 1U << 0,
        Configuration = 1U << 1,
        Input = 1U << 2,
        Timing = 1U << 3,
        Deadband = 1U << 4,
        Recovery = 1U << 5,
        Maneuver = 1U << 6,
        Travel = 1U << 7,
        Baseline = 1U << 8,
        NoDrive = 1U << 9,
    };
    struct DriveStatus {
        float command = 0.0f;   // projection of the quantized applied correction
        float headroom = 0.0f;  // physical travel on the configured ray (radii)
        float duty = 0.0f;      // active fraction of the last completed 1 s window
        uint32_t bursts = 0;
        uint16_t limited = 0;
    };

    // update() runs once before either driver. Both drivers see the same sensor
    // snapshot, demand and global recovery timer, regardless of routing order.
    void update(uint32_t now_ms, const RollSettings& settings,
                const RollDriveSettings (&drives)[2], const RollSample& sample);

    // The baseline is the EXISTING mapCoordinates() result. No correction means
    // returning that integer pair verbatim, including on every disabled call.
    Coordinates apply(uint8_t drive, Coordinates baseline, Limiter limits,
                      bool inputs_valid);

    bool enabled() const { return _settings.enabled; }
    State state() const { return _state; }
    float roll() const { return _roll; }
    float rate() const { return _rate; }
    float demand() const { return _demand; }
    uint16_t inhibited() const { return _inhibited; }
    const DriveStatus& status(uint8_t drive) const { return _status[drive]; }

private:
    static bool valid(const RollSettings& settings, const RollDriveSettings (&drives)[2]);
    bool changed(const RollSettings& settings, const RollDriveSettings (&drives)[2]) const;
    void stop();
    void account_time(uint32_t elapsed_ms);

    RollSettings _settings;
    RollDriveSettings _drives[2];
    DriveStatus _status[2];
    State _state = State::Disabled;
    uint16_t _inhibited = 0;
    uint32_t _now_ms = 0;
    uint32_t _burst_ms = 0;
    uint32_t _recovery_ms = 0;
    uint32_t _window_ms = 0;
    uint32_t _active_ms[2] {};
    float _dt = 0.0f;
    float _roll = 0.0f;
    float _rate = 0.0f;
    float _demand = 0.0f;
    float _amplitude = 0.0f;
    int8_t _selected = -1;
    int8_t _burst_drive = -1;
    bool _filter_ready = false;
    bool _demand_active = false;
    bool _applied[2] {};
};

} // namespace Vektor2
