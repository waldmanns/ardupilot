#pragma once

#include "MainPropulsion.h"
#include <stdint.h>

namespace Vektor2 {
namespace Components {

// Body frame: positive X ahead, positive Y starboard; angles clockwise.
// Mechanical angles are unwrapped position-servo coordinates, not headings.
struct AzipodSettings {
    int16_t angle_min = -90, angle_max = 90, offset = 0;
    int8_t reversible = 0, reverse_opt = 1, unreachable = 0;
    int16_t deadband = 25, rate = 90, hysteresis = 15, reverse_delay = 500;
    int16_t align = 5;
    float motor_slew = 1.0f;
    bool valid() const;
};

struct AzipodDemand { float angle; float magnitude; };
struct AzipodSolution { float angle; float thrust; bool reachable; };
AzipodDemand azipod_map(uint16_t x, uint16_t y, int16_t deadband);
AzipodSolution azipod_solve(const AzipodSettings& s, AzipodDemand demand,
                            float current, int8_t preferred_direction);

class AzipodControl {
public:
    enum Input : uint8_t { X_IN = 0, Y_IN = 1 };
    enum Output : uint8_t { AZ_OUT = 0, MOT_OUT = 1 };
    static constexpr uint8_t input_count = 2, output_count = 2;

    // Calibration latched at boot. Logic selects installed drives. No HAL dependency.
    void configure(const AzipodSettings& s);
    void run(uint32_t now, uint32_t revision, const uint16_t* inputs,
             const bool* valid, bool routes_ready, bool rc_valid, uint16_t* outputs);
    void calibrate(bool selected, uint16_t steering, int8_t motor_percent, bool routes_ready, uint16_t* outputs);
    bool calibration_valid() const { return _settings.valid(); }
    bool reversible() const { return _settings.reversible == 1; }
    // Per-cycle diagnostic flags, independent of the state enumeration.
    enum Reason : uint16_t {
        CalibrationInvalid = 1U << 0, OutputRouteMissing = 1U << 1,
        OutputRejected = 1U << 2, RcInvalid = 1U << 3,
        InputInvalid = 1U << 4, StartupCentering = 1U << 5,
        NeutralRequired = 1U << 6, AlignmentPending = 1U << 7,
        ReversalDelay = 1U << 8, DirectionUnreachable = 1U << 9,
        DirectionProjected = 1U << 10
    };
    uint16_t reasons() const { return _reasons; }
    void output_failed() { _reasons |= OutputRejected; _running = _armed = _calibrating = false; }
    uint16_t motor_stop() const { return _settings.reversible == 1 ? 1500 : 1000; }
    // Read-only commissioning telemetry. Angles are commands, not feedback.
    enum class State : uint8_t { Blocked = 0, Homing = 1, WaitNeutral = 2, Ready = 3, Calibration = 4 };
    State state() const { return _calibrating ? State::Calibration : !_running ? State::Blocked : _homing ? State::Homing :
        !_armed ? State::WaitNeutral : State::Ready; }
    float commanded_angle() const { return _angle; }
    float target_angle() const { return _target; }
    float motor_command() const { return _motor; }

private:
    AzipodSettings _settings;
    float _angle = 0, _target = 0, _motor = 0;
    uint32_t _last = 0, _revision = 0, _home_start = 0, _stopped_since = 0;
    uint16_t _reasons = 0;
    int8_t _last_direction = 0, _preferred = 1;
    bool _running = false, _homing = true, _armed = false, _calibrating = false;
    void stop(uint32_t now);
    uint16_t servo_pwm() const;
    uint16_t motor_pwm() const;
};

struct AZI1 : AzipodControl {};
struct AZI2 : AzipodControl {};

} // namespace Components

struct AzipodBackend {
    Components::AZI1 port;
    Components::AZI2 starboard;
    AzipodLayout layout = AzipodLayout::Unspecified;
    Components::AzipodControl& drive(uint8_t n) { return n == 0 ?
        static_cast<Components::AzipodControl&>(port) :
        static_cast<Components::AzipodControl&>(starboard); }
    const Components::AzipodControl& drive(uint8_t n) const { return n == 0 ?
        static_cast<const Components::AzipodControl&>(port) :
        static_cast<const Components::AzipodControl&>(starboard); }
};
} // namespace Vektor2
