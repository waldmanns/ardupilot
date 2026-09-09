# MiniDP consistency review and corrections — 2026-09-09

The initial review covered commit `2fc3748301`, all MiniDP modules, parameter
storage, frame allocation, controller and authority logic, MAVLink, output
backends, tests, build scripts, and operator documentation. Its 82 existing
unit tests passed despite the defects below. The working-tree corrections
address all 18 reproduced findings.

| # | Finding | Correction and regression coverage |
| ---: | --- | --- |
| 1 | SET_MODE dereferenced an absent AP_Vehicle singleton | MiniDP handles its custom mode messages. Shared mode, home, mission-status and reboot paths guard the singleton. SITL exercises mode messages and reboot. |
| 2 | NaN motor-test throttle/timeout reached casts and caused SIGFPE | Finite checks precede arithmetic, casts and state changes; timeout is bounded before multiplication. Unit and armed SITL tests reject malformed percent/PWM requests while preserving the current test. |
| 3 | Huge finite yaw hung the controller's subtraction loop | Finite-checked `remainderf` wrapping at target ingress and in control. Unit and SITL tests exercise `1e20` yaw. |
| 4 | No-baro build retained a barometer EKF source | Barometer remains disabled. GPS height defaults/migration cover EKF3 source sets and EKF2. Fresh SITL reaches DP_READY without a BARO source. |
| 5 | Frame 902 pure yaw also produced sway | Force/moment allocation uses measured aft and bow arms. Tests check net translation and moment for unequal arms; dynamics tests exercise station keeping. |
| 6 | Frame 901 screw yaw reversed with longitudinal mounting position | Differential yaw uses lateral half-span. Legacy SCR_POS no longer changes it; SCR_YAW explicitly scales/disables it. Geometry tests cover signs and unequal tunnel arms. |
| 7 | Limited azimuth travel could reverse the requested surge force | Feasible forward/reverse candidates use projected force and residual saturation. Regression checks restricted reverse corrections retain the correct force direction. |
| 8 | Asymmetric reversed steering clipped part of servo travel | Physical angle is normalized before electrical reversal. Endpoint regression covers asymmetric travel. |
| 9 | Swapping motor functions applied stale channel calibration | Output configurations install atomically; logical slots are separate from physical channels, including high-numbered and unassigned outputs. Unit and SITL tests swap functions with unequal trims. |
| 10 | FRAME_TYPE changed live despite reboot metadata | Application latches the frame at boot. Invalid boot frames inhibit propulsion. SITL checks a pending change leaves Motor5 available until reboot and removes it after switching to 901. |
| 11 | Refreshing limiter configuration erased sub-deadband slew progress | Configuration changes clamp magnitude without deadbanding stored progress. High-rate repeated-configuration regression reaches the requested ramp. |
| 12 | Application could not recover from latched failsafe | Disarm, clear the original fault, then request MANUAL to clear both state machines atomically. SITL verifies clearing while armed is rejected and recovery stays disarmed. |
| 13 | Zero I gains retained integral thrust | Disabling each integral gain clears its accumulator. Unit test builds both integrals and verifies zero output after disabling them. |
| 14 | Identical target refreshes reset integral compensation | Equivalent position/yaw/origin/reset targets retain target identity. Unit and SITL tests verify repeated absolute targets preserve it. |
| 15 | BODY_NED positions were interpreted as body offsets | BODY_NED positions use absolute local NED; only BODY_OFFSET_NED rotates/translates. Unsupported velocity/yaw-rate combinations are rejected. SITL checks frame equivalence and mask rejection. |
| 16 | Rejected requests changed control ownership | Mode, target and motor-test requests validate candidate state before committing authority and mode together. SITL checks rejected targets/modes and malformed test replacement. |
| 17 | TEST ownership rejected valid replacement tests | Authorized TEST-to-TEST requests are permitted. Unit and SITL tests replace the active actuator test. |
| 18 | README described only frame 901 and four outputs | README now covers both frames, geometry, azimuth parameters, safety, coordinates, six telemetry slots, no-baro defaults and test commands. |

Additional corrections:

- Board setup now calls `init_safety()`. This fixes the ignored
  `BRD_SAFETY_DEFLT=0` setting that could leave arbitrary RCIN outputs blocked.
  Scaled RCIN also has initialized RC angle ranges. SITL checks raw/scaled RCIN
  on SERVO8 and RCIN on a former managed motor output.
- RC overrides work independently of a physical receiver. Per-channel validity
  prevents holes in sparse overrides from using stale receiver channels.
  Receiver timestamps are tracked separately from override timestamps.
- Centered manual inputs do not cancel hold; cached MAVLink manual input cannot
  undo a newer hold command. RC takeover still requires AUTH_RC_TAKE.
- Azimuth commands have travel-rate limiting, continuity preference at folding
  boundaries, and coordinated neutral thrust during steering. Boot, steering
  PWM loss and actuator tests require a full sweep-time settling interval.
- Neutral steering holds its previous command; explicit failsafe PWM remains
  authoritative. Minimum-effective-output shaping never creates thrust from zero.
- Integral growth respects controller and downstream saturation, slew limiting
  and allocation limitations. Mode changes reset limiter history so a DP-to-
  heading fallback cannot retain a translational slew command.
- Normal propulsion requires a complete frame output assignment; individual
  tests can exercise assigned actuators. Unsupported mission capabilities are
  no longer advertised. SITL fault-injection parameters are registered.
- MAVLink pre-arm checks use MiniDP's surface-vessel policy without changing
  arming state or calling the generic vehicle mission/barometer checks. The
  shared mission pre-arm check also guards the missing vehicle singleton.

## Validation

- SITL firmware build: passed.
- Nine C++ suites, **101 tests**: passed, including two planar vessel simulations
  using allocated forces, steering travel, drag, and steady external force/moment.
- Isolated MAVLink SITL regression: passed, including receiver loss with sparse
  overrides, pre-arm checks, failsafe recovery, saved calibration after reboot,
  and applying a pending frame change only after reboot.
- Hardware compile for the existing `revo-mini` target: passed (600,911 bytes
  flash used; 382,108 bytes free).

The planar model is a regression fixture, not an identified hull. It assumes
symmetric pod placement and calibrated normalized thrust; it cannot establish
real vessel gains, reverse efficiency, actuator delays or achievable accuracy.
Steering position is estimated, so physical stalls require actual feedback to
be detected. No physical servo, ESC, CAN output, or vessel was connected here.
The custom MiniDP-F405 definition still needs the board schematic; its directory
correctly remains a placeholder rather than guessing pin assignments.
