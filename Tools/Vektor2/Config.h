#pragma once

// Vektor2 is deliberately an application, not an ArduPilot vehicle.
// Keep this header small. Feature pruning should happen in the board/build
// configuration, not by growing another vehicle framework here.

#include <AP_AHRS/AP_AHRS_config.h>
#include <AP_InertialSensor/AP_InertialSensor_config.h>

#include <stdint.h>

#define VEKTOR2_FIRMWARE_NAME "Vektor2"

namespace Vektor2 {

static constexpr uint16_t firmware_major = 0;
static constexpr uint16_t firmware_minor = 5;
static constexpr uint16_t firmware_patch = 0;

// EKF/IMU service rate. 100 Hz is intentionally conservative for F405-class
// hardware and can be raised later after timing and RAM/flash are measured.
static constexpr uint16_t loop_rate_hz = 100;

// HEARTBEAT is the only application-pushed MAVLink message. All normal
// telemetry rates are owned by the standard ArduPilot MAVLink scheduler and
// MAV_CMD_SET_MESSAGE_INTERVAL / stream requests.
static constexpr uint16_t heartbeat_period_ms = 1000;

// Thin PWM wrapper limit. This is an application-side bound only; the board
// HAL remains authoritative about which channels physically exist.
static constexpr uint8_t max_pwm_channels = 32;
static constexpr uint16_t default_pwm_rate_hz = 50;

// Fixed-capacity signal routing table. Routes can connect RC inputs and logic
// component outputs to component inputs or PWM outputs. This is route capacity,
// not a claim that every board exposes 32 physical PWM channels.
static constexpr uint8_t max_routes = 32;
static constexpr uint16_t rc_input_timeout_ms = 500;

// Parameter-backed routes are checked at a low rate. Runtime routing still
// runs every estimator cycle.
static constexpr uint16_t route_parameter_poll_ms = 100;

} // namespace Vektor2
