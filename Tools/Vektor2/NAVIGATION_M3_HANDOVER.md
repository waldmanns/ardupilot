# Vektor2 — M3 vessel-agnostic DP diagnostics and reason-bit handover

**Date:** 23 November 2026  
**Input:** `Vektor2_Navigation_Propulsion_M2_HostAllocator.zip` and the agreed
navigation/propulsion baseline.  
**Safety status:** OBSERVATION ONLY. Automatic VSP/thruster outputs are not enabled.

## Delivered

1. `NavigationDP.h/.cpp`: pure C++11 horizontal DP core. Target and actual
   latitude/longitude (E7), N/E error and range/bearing, velocity feedback,
   bounded desired velocity, damped acceleration demand, normalized surge/sway,
   magnitude and direction. Uses yaw-only earth/body conversion. No vessel
   geometry, rudder, VSP, thruster, or ArduPilot vehicle-mode framework.
2. `NavigationHeading.h/.cpp`: the one yaw loop for Heading Hold and DP,
   with wrap-safe heading error and roll/pitch-corrected earth yaw rate
   from body gyros. No live output ownership.
3. `Vektor2.cpp`: capture/update the controllers from AHRS/EKF3 estimates,
   on valid transitions/explicit capture requests; clear targets after Free
   or yaw EKF reset; feed results into the existing supervisor. Controller
   output is used for diagnostics only.
4. `NavigationDP::Output::reason_bits` now emits the **existing** reason
   flags: `HEADING_INVALID`, `POSITION_INVALID`, `VELOCITY_INVALID`,
   `TARGET_INVALID`, `ESTIMATOR_RESET`, `CONFIG_INVALID`, `TIMING_FAULT`.
   Multiple conditions accumulate. A failed input zeroes demanded motion.
5. `NavigationSupervisor` merges these controller reason bits only in mode 2,
   preserving the **one public** `NAV_RBITS` and `NAV_STATE` interface.
   DP-specific parameter failure does not erroneously inhibit yaw/Heading Hold.
6. Top-level `DP_TEL_HZ` defaults to 0, range 0–50 Hz, and is capped at 50
   by `NamedFloatTelemetry`. Its MAVLink stream includes target, error,
   velocity, command-vector, and heading readings; unknown fields are NaN.
   Full E7 coordinates are available as VTEL sources 240–243, with DP
   diagnostics in 244–256, through existing receiver polling.
7. `NAVIGATION.md`, `VTEL.md`, README and focused host tests updated.

## Unified navigation reason mask

| Bit | Hex | Reason | DP controller emits it |
|---|---|---|---|
| 0 | 0x0001 | DISABLED | No: supervisor |
| 1 | 0x0002 | HEADING_INVALID | Yes |
| 2 | 0x0004 | POSITION_INVALID | Yes |
| 3 | 0x0008 | VELOCITY_INVALID | Yes |
| 4 | 0x0010 | RC_INVALID | No: supervisor |
| 5 | 0x0020 | PILOT_OVERRIDE | No: supervisor |
| 6 | 0x0040 | OUTPUT_NOT_READY | No: supervisor |
| 7 | 0x0080 | WAIT_NEUTRAL | No: supervisor |
| 8 | 0x0100 | TARGET_INVALID | Yes |
| 9 | 0x0200 | ESTIMATOR_RESET | Yes |
| 10 | 0x0400 | CONFIG_INVALID | Yes |
| 11 | 0x0800 | TIMING_FAULT | Yes |

`NAV_RBITS` retains its own existing MAVLink status timing (2 Hz plus bounded
change notification), and VTEL source 141. It is not duplicated as `DP_RBITS`.
`DP_TEL_HZ` only governs the new fast DP/heading diagnostic stream.

## Verification

The following independent host test runners PASS:
`test_navigation_dp.py`, `test_navigation_supervisor.py`,
`test_vtelemetry.py`, `test_vessel_thrust_allocator.py`,
`test_vsp_mapper.py`, `test_vsp_vector.py`,
`test_roll_stabilization.py`, `test_thrusters.py`,
`test_functions.py`, and `test_sequencer.py`.

`test_measure_points.py` could **not be run** because its required
`libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat` path is outside the
supplied application archive. Full ArduPilot firmware build, SITL and physical
actuator verification were **not performed**. Host tests do not establish
firmware or vessel readiness.

## Open commissioning issues

- Verify EKF `get_location()` and velocity refer to the same **hull control
  point** as target capture. GNSS antenna lever-arm treatment is unverified;
  rocking must not masquerade as drift.
- Establish real heading observability at zero speed. Compass-less GSF yaw
  may not give a reliable stationary heading without an additional source.
- Choose DP/heading gains, control rate, filters, antenna offsets, geometry,
  allocator authority and saturation priorities using appropriate testing.
- Decide assignable RC navigation-mode and manual ownership actions, Heading
  Hold translation input, pair-wide VSP override and independent bow override,
  re-engagement/neutral/failsafe rules.
- Integrate propulsion only after safe authority acknowledgment, bench tests,
  full firmware compilation and SITL. `HDG_ACTIVE` / `DP_ACTIVE` must never
  be announced without accepted live output ownership.

**No additional DP state machine or telemetry transport was introduced.**
