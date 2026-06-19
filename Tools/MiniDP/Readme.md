# MiniDP

MiniDP is a small ArduPilot vehicle application for dynamic positioning on a
surface vessel. It uses the normal ArduPilot HAL, EKF, GPS, compass, RC,
servo, logging, and MAVLink plumbing, but keeps the vehicle logic focused on
manual control, heading hold, DP hold, and simple thruster mixing.

This file is the operator/developer note for the MiniDP application. Keep it in
sync whenever MiniDP parameters, frame types, mode behavior, or output mapping
change.

## Operating Model

The main loop updates IMU, compass, GPS, AHRS/EKF state, RC input, MAVLink
input, authority arbitration, mode logic, controller output, axis limiting, and
servo output. Outputs are recomputed every loop and are driven through
ArduPilot `SERVOx_*` parameters.

MiniDP has these modes:

| Mode | Custom mode | Use |
| --- | ---: | --- |
| `MANUAL` | 0 | Manual surge, sway, and yaw from RC or MAVLink `MANUAL_CONTROL`. |
| `HEADING_HOLD` | 1 | Holds current yaw. Requires valid yaw. |
| `DP_HOLD` | 2 | Holds current yaw and local NE position. Requires healthy EKF, origin, position, velocity, and yaw. |
| `ACTUATOR_TEST` | 3 | Internal motor-test mode entered through `MAV_CMD_DO_MOTOR_TEST`. |
| `FAILSAFE` | 4 | Latched safe state after kill, unsafe state, or configured link loss. |

`HEADING_HOLD` and `DP_HOLD` are requested through MAVLink mode commands. RC or
MAVLink manual input can take the vessel back to `MANUAL` when manual authority
is enabled and accepted.

## DP Target Behavior

Entering `DP_HOLD` latches the current EKF local position and yaw as the target.
With `DP_RETARGET=1`, any request for `DP_HOLD` also relatches the current
position and yaw, even if MiniDP is already in `DP_HOLD`. This is useful for a
mode switch or ground-station button that means "hold here now".

If yaw is lost while holding, MiniDP enters `FAILSAFE`. If DP loses its origin,
EKF reset identity, position, velocity, or EKF health, it falls back to
`HEADING_HOLD` when yaw is still valid.

## Output Frame

`FRAME_TYPE=901` is the current default and only implemented frame. It is named
`OMNI_PLUS` and maps four actuators in clean ArduPilot motor order:

| Motor | Default output | Physical actuator | Mix contribution |
| --- | --- | --- | --- |
| `Motor1` | `SERVO1_FUNCTION=33` | Port propulsion screw | Surge + yaw |
| `Motor2` | `SERVO2_FUNCTION=34` | Starboard propulsion screw | Surge - yaw |
| `Motor3` | `SERVO3_FUNCTION=35` | Bow tunnel thruster | Sway + yaw |
| `Motor4` | `SERVO4_FUNCTION=36` | Stern tunnel thruster | Sway - yaw |

Motor outputs use the standard `SERVOx_MIN`, `SERVOx_TRIM`, `SERVOx_MAX`, and
`SERVOx_REVERSED` parameters. Mission Planner should show the motor channels as
normal servo outputs because they are ordinary ArduPilot `Motor1` through
`Motor4` functions.

Unused outputs may still use normal ArduPilot servo functions. For example,
`SERVO5_FUNCTION=51` passes through `RCIN1`, and `SERVO5_FUNCTION=140` uses the
scaled `RCIN1Scaled` function. MiniDP temporarily masks its managed motor
channels while allowing other valid `SERVOx_FUNCTION` outputs to run through the
standard SRV channel code.

## Controller Path

Manual and DP commands both pass through the same output chain:

1. Input or controller produces normalized axis commands: `surge`, `sway`,
   `yaw`, each in `-1..1`.
2. `AXIS_*` limits apply deadband, max output, and optional slew-rate limiting.
3. The frame mixer converts axes to per-motor demands.
4. If any motor demand exceeds `-1..1`, all motor demands are scaled together.
5. Demands are converted to PWM using each motor's `SERVOx_*` range.

The DP controller is intentionally simple:

| Axis | Control law |
| --- | --- |
| Yaw | `DP_YAW_P * yaw_error - DP_YAW_D * yaw_rate` |
| Surge | `DP_POS_P * body_forward_position_error - DP_VEL_D * body_forward_velocity` |
| Sway | `DP_POS_P * body_lateral_position_error - DP_VEL_D * body_lateral_velocity` |

All DP gains default to zero, so the vessel will not actively correct position
or heading until gains are set.

## Key Setup Steps

1. Flash the MiniDP firmware and connect with Mission Planner or another
   MAVLink ground station.
2. Confirm the firmware reports as MiniDP. The MAVLink vehicle type is still a
   surface boat for compatibility.
3. Check `FRAME_TYPE=901`.
4. Confirm `SERVO1_FUNCTION` through `SERVO4_FUNCTION` are `Motor1` through
   `Motor4`, or assign them to the physical output channels you need.
5. Check motor direction with `MAV_CMD_DO_MOTOR_TEST` after setting
   `AUTH_TEST=1`.
6. Set `MAN_ENABLE=1` only after the output mapping is safe.
7. Set MAVLink authority parameters for the control source you intend to use.
8. Tune manual limits first, then `AXIS_*`, then DP gains.

## MiniDP Parameters

### Identity And Common ArduPilot Groups

| Parameter | Default | Use |
| --- | ---: | --- |
| `FORMAT_VERSION` | `0` | Internal storage format marker. MiniDP resets parameters when the expected format changes. |
| `LOG_BITMASK` | `-1` | ArduPilot logging bitmask. |
| `BRD_*` | Board defaults | Board configuration. |
| `SERIAL*` | Board defaults | Serial port setup. |
| `INS*` | Board defaults | IMU setup. |
| `COMPASS_*` | Board defaults | Compass setup. |
| `GPS*` | Board defaults | GPS setup. |
| `LOG*` | Board defaults | Logger setup. |
| `NTF_*` | Board defaults | Notify/LED/buzzer setup. |
| `RC*` | Board defaults | RC input calibration and behavior. |
| `SERVO*` | Board defaults | Output function, min, trim, max, reverse, and related servo settings. |
| `MAV*` | Board defaults | GCS/MAVLink settings when GCS support is enabled. |
| `AHRS_*`, `EK2_*`, `EK3_*` | ArduPilot defaults | AHRS/EKF setup. |
| `BARO*` | Not registered by default | Barometer parameters only exist when `MINIDP_BARO_ENABLED=1` at compile time. |

### Authority And Failsafe

| Parameter | Default | Use |
| --- | ---: | --- |
| `AUTH_MAV_MAN` | `0` | Allow MAVLink `MANUAL_CONTROL` to own manual control. |
| `AUTH_MAV_TGT` | `0` | Allow MAVLink target modes, including `HEADING_HOLD` and `DP_HOLD`. |
| `AUTH_TEST` | `0` | Allow actuator/motor tests. |
| `AUTH_RC_TAKE` | `0` | Allow RC to take over while MAVLink owns control. |
| `AUTH_RC_FB` | `1` | Fall back to RC when MAVLink manual/target authority is lost and RC is healthy. |
| `AUTH_MAV_FB` | `0` | Let MAVLink manual take over when RC authority is lost. |
| `AUTH_TGT_LOSS` | `0` | Target-link loss action: `0` failsafe, `1` hold last target, `2` fall back to RC if healthy. |
| `AUTH_RC_TMO` | `0.5` | RC health timeout in seconds. Runtime minimum is 50 ms. |
| `AUTH_MAV_TMO` | `3.0` | MAVLink health timeout in seconds. Runtime minimum is 100 ms. |

### Frame And RC Input

| Parameter | Default | Use |
| --- | ---: | --- |
| `FRAME_TYPE` | `901` | Output/mixer frame. `901` is `OMNI_PLUS`. Unknown values fall back to `901`. |
| `IN_RC_SURGE` | `2` | 1-based RC channel for manual surge. `0` disables the axis. |
| `IN_RC_SWAY` | `1` | 1-based RC channel for manual sway. `0` disables the axis. |
| `IN_RC_YAW` | `4` | 1-based RC channel for manual yaw. `0` disables the axis. |
| `IN_RC_KILL` | `0` | 1-based RC kill channel. `0` disables RC kill. |
| `IN_KILL_PWM` | `1800` | Kill threshold PWM. A kill channel at or above this value forces `KILL`/failsafe handling. |

### Manual Control

| Parameter | Default | Use |
| --- | ---: | --- |
| `MAN_ENABLE` | `0` | Enables manual RC/MAVLink output when set to `1`. |
| `MAN_DZ` | `0.03` | Manual input deadband. |
| `MAN_SRG_LIM` | `0.5` | Manual surge limit. |
| `MAN_SWY_LIM` | `0.5` | Manual sway limit. |
| `MAN_YAW_LIM` | `0.5` | Manual yaw limit. |
| `MAN_MAV_TMO` | `0.5` | MAVLink manual-control timeout in seconds. Runtime clamp is 0.1 to 5.0 seconds. |

### Axis Limiter

| Parameter | Default | Use |
| --- | ---: | --- |
| `AXIS_DZ` | `0.0` | Deadband applied after manual/controller output. |
| `AXIS_SRG_MAX` | `1.0` | Final surge output limit. |
| `AXIS_SWY_MAX` | `1.0` | Final sway output limit. |
| `AXIS_YAW_MAX` | `1.0` | Final yaw output limit. |
| `AXIS_SRG_SLW` | `0.0` | Surge slew rate in normalized output per second. `0` disables slew limiting. |
| `AXIS_SWY_SLW` | `0.0` | Sway slew rate in normalized output per second. `0` disables slew limiting. |
| `AXIS_YAW_SLW` | `0.0` | Yaw slew rate in normalized output per second. `0` disables slew limiting. |

### DP Controller

| Parameter | Default | Use |
| --- | ---: | --- |
| `DP_YAW_P` | `0.0` | Heading-hold yaw proportional gain. |
| `DP_YAW_D` | `0.0` | Yaw-rate damping gain. |
| `DP_POS_P` | `0.0` | Position proportional gain for surge and sway. |
| `DP_VEL_D` | `0.0` | Velocity damping gain for surge and sway. |
| `DP_SRG_MAX` | `0.5` | DP controller surge output limit. |
| `DP_SWY_MAX` | `0.5` | DP controller sway output limit. |
| `DP_YAW_MAX` | `0.5` | DP controller yaw output limit. |
| `DP_RETARGET` | `1` | Relatch current yaw/position whenever `DP_HOLD` is requested. |

## Compile-Time Options

`MINIDP_BARO_ENABLED` defaults to `0` in `Config.h`. With the default build,
MiniDP keeps the shared ArduPilot barometer singleton present but does not
register `BARO*` parameters, initialize baro hardware, calibrate the barometer,
or update it in the main loop. Build with `MINIDP_BARO_ENABLED=1` only if a
future MiniDP configuration intentionally needs barometer data.

## Ground Station Notes

MiniDP sends standard ArduPilot MAVLink heartbeats using
`MAV_AUTOPILOT_ARDUPILOTMEGA` and `MAV_TYPE_SURFACE_BOAT`, with custom modes
for MiniDP. It also sends MiniDP firmware identity text with
`AUTOPILOT_VERSION`, so Mission Planner users can see that the connected
firmware is MiniDP.

Use Mission Planner's servo output view to verify `Motor1` through `Motor4`.
For passthrough outputs, assign an unused output to `RCIN1` through `RCIN16`
(`SERVOx_FUNCTION=51..66`) or `RCIN1Scaled` through `RCIN16Scaled`
(`SERVOx_FUNCTION=140..155`).

## Developer Checks

Typical local checks:

```sh
./waf configure --board revo-mini
./waf --targets bin/MiniDP
./waf configure --board sitl
./waf --targets tests/test_minidp_mode,tests/test_minidp_authority,tests/test_minidp_input,tests/test_minidp_axis_limiter,tests/test_minidp_controller,tests/test_minidp_output,tests/test_minidp_actuator_test,bin/MiniDP
build/sitl/tests/test_minidp_mode
build/sitl/tests/test_minidp_authority
build/sitl/tests/test_minidp_input
build/sitl/tests/test_minidp_axis_limiter
build/sitl/tests/test_minidp_controller
build/sitl/tests/test_minidp_output
build/sitl/tests/test_minidp_actuator_test
./waf configure --board revo-mini
```

When adding new MiniDP parameters or behavior, update this file in the same
change.
