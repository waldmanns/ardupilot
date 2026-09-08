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
input, arming state, authority arbitration, mode logic, controller output, axis
limiting, and servo output. Outputs are recomputed every loop and are driven
through ArduPilot `SERVOx_*` parameters.

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
is enabled and accepted. Centered inputs do not cancel target modes; takeover
requires stick deflection beyond the configured deadband and authority policy.

## Arming

MiniDP accepts `MAV_CMD_COMPONENT_ARM_DISARM`. Active motor outputs are allowed
only when MiniDP is armed, `ARMING_REQUIRE=0`, or an actuator is otherwise
explicitly permitted by the output state machine. A hardware safety switch in
the disarmed state still keeps managed motor outputs quiet.

`ARM_GPS_REQ` controls the GPS arming policy:

| Value | Meaning |
| ---: | --- |
| `0` | Do not require GPS to arm. Manual control can be bench-tested without GPS. |
| `1` | Require a 2D-or-better GPS fix and the configured GPS accuracy limits. |
| `2` | Require GPS accuracy plus DP-ready EKF state: yaw, origin, position, velocity, and EKF health. |

Force arm through MAVLink `param2=21196` skips GPS and DP-ready checks, but
does not bypass RC kill or failsafe rejection.

## DP Target Behavior

Entering `DP_HOLD` latches the current EKF local position and yaw as the target.
With `DP_RETARGET=1`, any request for `DP_HOLD` also relatches the current
position and yaw, even if MiniDP is already in `DP_HOLD`. This is useful for a
mode switch or ground-station button that means "hold here now".

If yaw is lost while holding, MiniDP enters `FAILSAFE`. If DP loses its origin,
EKF reset identity, position, velocity, or EKF health, it falls back to
`HEADING_HOLD` when yaw is still valid. DP entry and continued DP hold also
respect the configured GPS horizontal and speed accuracy limits.

MiniDP also accepts MAVLink target updates as new DP hold points when
`AUTH_MAV_TGT=1` and the EKF/GPS state is DP-ready. Mission Planner map
"go here" / guided reposition commands set the clicked latitude/longitude as
the new `DP_HOLD` point. `SET_POSITION_TARGET_GLOBAL_INT` does the same for a
global target, while `SET_POSITION_TARGET_LOCAL_NED` can set an EKF-origin
local target. `LOCAL_NED` and `BODY_NED` positions are absolute relative to
the EKF origin. `LOCAL_OFFSET_NED` offsets the current vessel position in NE;
`BODY_OFFSET_NED` offsets the current position along forward/starboard axes
and interprets yaw relative to current heading. Offset commands are relative
to the vessel at receipt, so repeatedly sending an offset moves the target.
Velocity, acceleration and yaw-rate control are unsupported: set their ignore
bits. Z/altitude is unused on this surface vessel. Identical absolute targets
preserve their identity and integral state. Invalid requests leave mode and
control ownership unchanged. If the
incoming command does not include yaw, MiniDP keeps the existing DP yaw target
when possible, otherwise it latches the current yaw.

## How DP Hold Works

Easy view: `DP_HOLD` is a "stay here and keep pointing this way" mode. When the
mode is entered, MiniDP remembers the current position and heading, then keeps
comparing the boat against that remembered target. If the boat drifts north,
south, sideways, or rotates away from the target heading, MiniDP asks the
thrusters for the opposite correction. The correction stays small near the
target, grows as the error grows, and is limited before it reaches the motors so
tuning mistakes do not immediately command full thrust.

Technical view: `DP_HOLD` uses the EKF local north/east position, velocity, yaw,
and yaw rate to build normalized `surge`, `sway`, and `yaw` commands. Yaw error
is wrapped to the shortest turn and controlled by `DP_YAW_P`, bounded integral
correction from `DP_YAW_I`/`DP_YAW_IMAX`, and yaw-rate damping from
`DP_YAW_D`. Position error is first shaped by `DP_POS_DZ` and `DP_POS_RAD`, then
the proportional position command from `DP_POS_P` is combined with a bounded
north/east integral correction from `DP_POS_I`/`DP_POS_IMAX`. That position
command is rotated into the body frame so it becomes surge and sway, then
`DP_VEL_D` subtracts body-frame velocity damping. `DP_SRG_MAX`, `DP_SWY_MAX`,
and `DP_YAW_MAX` limit the controller output before the normal `AXIS_*` limits,
frame mixer, motor scaling, and `SERVOx_*` PWM mapping run. Integral state is
cleared when the mode or target changes, when required state becomes invalid, or
when controller output is not active. Setting an I gain to zero also clears
that integral. Controller saturation, axis limiting, unavailable motor mappings,
and steering/allocator limits inhibit further windup while allowing unwinding.

## Output Frame

`FRAME_TYPE` is selected at boot. Changing it requires reboot; unknown values
inhibit managed outputs. Both implemented frames assume bidirectional thrust,
with forward surge, starboard sway, and clockwise yaw positive.

`901` (`OMNI_PLUS`, the default):

| Motor | Default function/output | Physical actuator |
| --- | --- | --- |
| Motor1 | `SERVO1_FUNCTION=33` | Port propulsion screw |
| Motor2 | `SERVO2_FUNCTION=34` | Starboard propulsion screw |
| Motor3 | `SERVO3_FUNCTION=35` | Bow tunnel thruster |
| Motor4 | `SERVO4_FUNCTION=36` | Stern tunnel thruster |

Differential screw yaw depends on port/starboard placement, regardless of
whether the screws are forward or aft. `FRAME_SCR_POS` is a retained legacy
annotation; it no longer changes yaw. `FRAME_SCR_YAW=0` disables screw yaw.
`FRAME_HALF_SPAN` is each screw's lateral distance from the vessel center.
`FRAME_BOW_ARM` and `FRAME_AFT_ARM` are positive longitudinal distances from
the yaw center to the bow and stern tunnel thrusters, in metres.

With unit arms/span and `FRAME_SCR_YAW=1`, demands retain the original
`surge+yaw`, `surge-yaw`, `sway+yaw`, `sway-yaw` convention. Before saturation,
this corresponds to net surge `2*surge`, sway `2*sway`, and moment
`(2+2*FRAME_SCR_YAW)*yaw` in normalized thrust/lever-arm units. Unequal tunnel
arms are compensated so pure sway produces no yaw moment.

`902` (`DUAL_AZ_180_BOW`):

| Motor | Default function/output | Physical actuator |
| --- | --- | --- |
| Motor1 | `SERVO1_FUNCTION=33` | Aft port pod thrust |
| Motor2 | `SERVO2_FUNCTION=34` | Aft port pod steering |
| Motor3 | `SERVO3_FUNCTION=35` | Aft starboard pod thrust |
| Motor4 | `SERVO4_FUNCTION=36` | Aft starboard pod steering |
| Motor5 | `SERVO5_FUNCTION=37` | Bow tunnel thruster |

Both pods must be at the same aft distance (`FRAME_AFT_ARM`) and symmetrically
placed. `FRAME_BOW_ARM` is the bow thruster distance. The allocation solves
net forward/lateral force and yaw moment; pure yaw produces opposing aft and
bow lateral forces with zero net sway. Frame 902 axes represent net normalized
force and moment directly, so gains are not interchangeable with frame 901.

`AZI1_ANG_MIN/MAX` and `AZI2_ANG_MIN/MAX` define physical steering limits
about forward (degrees, total travel at most 180 degrees). `AZI1_REV_FOLD` and
`AZI2_REV_FOLD` allow reverse thrust to produce otherwise inaccessible
vectors. Infeasible vectors are projected onto a feasible direction and
reported as saturated. `AZIRATE` (default 90 degrees/second) must be set no
faster than either real servo can travel under load. All three thrust outputs
stay neutral while either pod is steering. After boot, disabled steering PWM,
or an actuator test, a full travel-time settling period precedes thrust.
Neutral output holds the last steering angle; an explicit failsafe-PWM action
still uses `OUT_FS_PWM`. Steering angle is an estimate from commands and rate,
not measured position feedback; a stalled servo cannot be detected by this model.

Motor outputs use `SERVOx_MIN/TRIM/MAX/REVERSED` on the physical channel
assigned to that motor function. Reassignments are applied together so swapped
functions keep the destination channel's calibration. Assign each motor
function once; duplicated functions share one logical PWM command/calibration.
Normal propulsion requires every motor/steering function for the selected
frame to be assigned; removing one inhibits propulsion. Individual actuator
tests can still check assigned outputs. Changing mappings, steering calibration,
or geometry requires rechecking the
physical force directions before enabling hold.

Other physical outputs can use `RCIN1..16` (`51..66`) or
`RCIN1Scaled..16Scaled` (`140..155`). For example, `SERVO8_FUNCTION=51`
passes RC1 PWM to output 8. Raw RCIN copies input PWM; scaled RCIN applies RC
calibration and that output's range/reversal. These auxiliary outputs do not
require `MAN_ENABLE` or MiniDP arming, but board safety and output protocol
still apply. MiniDP initializes board safety, including `BRD_SAFETY_DEFLT=0`;
otherwise release the hardware safety switch or configure the appropriate
`BRD_SAFETY_MASK`. The output must support PWM, and the servo needs its normal
power supply. Check RC calibration input and `SERVO_OUTPUT_RAW` to distinguish
input/mapping problems from physical output problems.

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
| Yaw | `DP_YAW_P * yaw_error + bounded_yaw_i - DP_YAW_D * yaw_rate` |
| Surge | `body_forward(DP_POS_P * position_error + bounded_position_i) - DP_VEL_D * body_forward_velocity` |
| Sway | `body_lateral(DP_POS_P * position_error + bounded_position_i) - DP_VEL_D * body_lateral_velocity` |

Position error is shaped before it reaches the surge/sway controller. Inside
`DP_POS_DZ`, the position term is zero. Between `DP_POS_DZ` and `DP_POS_RAD`,
the position term ramps in gently. Outside `DP_POS_RAD`, the full position error
is used.

All DP gains default to zero, so the vessel will not actively correct position
or heading until gains are set.

Tune P and D terms first with `DP_YAW_I` and `DP_POS_I` left at zero. After the
vessel is stable, add small I terms only to cancel steady current, wind, or
thruster bias. `DP_YAW_IMAX` and `DP_POS_IMAX` cap the integral contribution,
and MiniDP resets the stored integral correction whenever the mode or target
changes or controller output is no longer active.

## Key Setup Steps

1. Flash the MiniDP firmware and connect with Mission Planner or another
   MAVLink ground station.
2. Confirm the firmware reports as MiniDP. The MAVLink vehicle type is still a
   surface boat for compatibility.
3. Select `FRAME_TYPE=901` or `902` and reboot; set measured lever arms.
4. Assign the four (901) or five (902) motor functions using the table above.
   For frame 902, calibrate steering endpoints, reversal and `AZIRATE`.
5. Check motor direction with `MAV_CMD_DO_MOTOR_TEST` after setting
   `AUTH_TEST=1`.
6. Choose arming policy with `ARM_GPS_REQ`: `0` for no-GPS arming, `1` for GPS
   fix arming, or `2` for DP-ready arming.
7. Arm MiniDP before expecting active motor output.
8. Configure optional `BATT*` battery monitor parameters and battery failsafe
   actions.
9. Set `MAN_ENABLE=1` only after the output mapping is safe.
10. Set MAVLink authority parameters for the control source you intend to use.
11. For RTK, leave GPS autoconfiguration enabled and use Mission Planner's
    standard NTRIP/RTK injection tools.
12. Tune manual limits first, then `AXIS_*`, then DP gains.

## Parameter Storage

MiniDP uses the standard ArduPilot parameter storage area for the board. The
internal `FORMAT_VERSION` marker is force-saved during startup if it is missing,
so normal parameters such as `SERIAL*`, `MAV*`, `RC*`, `SERVO*`, `RSSI_*`,
`SCHED*`, and `DP_*` persist across reboot. MiniDP also flushes pending
parameter writes before honoring Mission Planner's reboot command.

## Scheduler And GCS Servicing

MiniDP uses the standard ArduPilot scheduler pattern used by Rover. RC input is
read by a scheduler task, GCS receive/send are serviced by scheduler tasks, and
the default `SCHED_LOOP_RATE` is `400` Hz. Keep `SCHED_LOOP_RATE=400` for normal
hardware testing so MAVLink links, ELRS MAVLink RC, and Mission Planner streams
are serviced at the same cadence as the vehicle code.

## RTK And NTRIP

MiniDP uses the standard ArduPilot `AP_GPS` frontend for RTK. Mission Planner's
NTRIP client sends RTCM corrections as MAVLink `GPS_RTCM_DATA`; the shared
ArduPilot GCS layer forwards those messages to `AP_GPS`, which handles RTCM
fragment reassembly, `GPS_INJECT_TO`, and receiver injection. MiniDP does not
add a separate NTRIP client or receiver configurator.

Use the same receiver setup as Rover or Copter:

| Parameter | Typical use |
| --- | --- |
| `SERIALx_PROTOCOL=5` | GPS on the serial port connected to the receiver. |
| `SERIALx_BAUD` | Auto or the receiver baud rate. |
| `GPS_TYPE` | Auto/u-blox/etc. for the connected receiver. |
| `GPS_AUTO_CONFIG=1` | Let ArduPilot configure serial GPS receivers. |
| `GPS_SAVE_CFG=2` | Save receiver configuration only when needed. |
| `GPS_INJECT_TO=127` | Default; inject RTCM corrections to all eligible GPS receivers. |
| `GPS_DRV_OPTIONS` | Standard backend options, including RTCM parser options where compiled in. |

For ExpressLRS MAVLink mode, one UART can carry both RC uplink and MAVLink
telemetry. Configure the ELRS transmitter and receiver for MAVLink mode, then
set the flight-controller UART connected to the ELRS receiver to
`SERIALx_PROTOCOL=2`, `SERIALx_BAUD=460`, and `RSSI_TYPE=5`. ELRS sends RC as
standard MAVLink-radio RC data, which MiniDP feeds into the normal ArduPilot RC
frontend used by Mission Planner radio calibration.

Do not use `SERIALx_PROTOCOL=23` for ELRS MAVLink mode. Protocol `23` is the
raw serial RC-input path for receivers sending CRSF/SBUS/etc. bytes directly to
ArduPilot. ELRS MAVLink mode must be configured as a MAVLink port so Mission
Planner can connect on the same link and the shared MAVLink handler can receive
`RADIO_RC_CHANNELS`.

For plain serial RC, configure it the same way as Rover or Copter: connect the
receiver signal to a UART RX pin, set that port to `SERIALx_PROTOCOL=23`, and
reboot. The ArduPilot RC protocol frontend auto-searches the supported serial
receiver formats, including iBUS and CRSF. If `RC_PROTOCOLS` has been restricted,
set it back to `1` to allow all protocols while debugging. MiniDP services the
same `AP_RCProtocol` frontend as normal ArduPilot, and sends `RC_UART=1` when a
`SERIALx_PROTOCOL=23` UART has been registered.

Mission Planner's Radio Calibration page shows MiniDP's outgoing `RC_CHANNELS`
MAVLink stream. MiniDP uses Rover-style default MAVLink stream rates, so
`MAVx_RC_CHAN` defaults to a nonzero rate on new/default parameters. If telemetry
connects but the bars stay blank on an existing parameter set, set the
`MAVx_RC_CHAN` stream rate for the Mission Planner link to `1` or higher
(`MAV1_RC_CHAN` is typical for the USB console link, `MAV2_RC_CHAN` is typical
for the first telemetry UART such as Serial1 when USB is enabled). While
debugging, setting both `MAV1_RC_CHAN=2` and `MAV2_RC_CHAN=2` is harmless. Also
confirm that both ELRS transmitter and receiver are in MAVLink mode, the
receiver UART protocol is MAVLink, the hardware is ESP-based, and
`SERIALx_PROTOCOL`/`SERIALx_BAUD` were followed by a reboot. MiniDP sends RC
diagnostic named values once per second:

| Value | Meaning |
| --- | --- |
| `MAV_CH` | Number of active MiniDP MAVLink backends. USB only is normally `1`; USB plus a MAVLink Serial1 backend is normally `2`. |
| `RC_OK` | `1` when MiniDP has fresh, non-failsafe RC input. |
| `RC_CH` | Current ArduPilot RC input channel count. |
| `RC_UART` | `1` when a `SERIALx_PROTOCOL=23` RCIN UART is registered. |
| `RC_PROT` | Detected ArduPilot RC protocol number. `15` is MAVLink radio. |
| `RC_AGE` | Seconds since MiniDP last read fresh RC input. |
| `RRC_CNT` | Count of accepted MAVLink `RADIO_RC_CHANNELS` packets. |
| `RRC_CH` | Channel count field from the last `RADIO_RC_CHANNELS` packet. |
| `RRC_C1` | Channel 1 from the last `RADIO_RC_CHANNELS` packet, converted to PWM. |
| `RRC_FLG` | Flags from the last `RADIO_RC_CHANNELS` packet. |
| `RCO_CNT` | Count of accepted MAVLink `RC_CHANNELS_OVERRIDE` packets. |
| `RCO_CH` | Highest non-empty override channel in the last override packet. |
| `RCO_C1` | Channel 1 raw PWM from the last override packet. |

In Mission Planner, connect telemetry, open the normal RTK/NTRIP injection
tool, enter the caster credentials and mountpoint, and start injection. Watch
the standard GPS status: fix type `5` is RTK Float and fix type `6` is RTK
Fixed. MiniDP also sends `GPS_FIX`, `GPS_SATS`, and `RTK_FIX` named values once
per second; `RTK_FIX` is `0` for no RTK, `1` for Float, and `2` for Fixed.

For DP, tune `DP_HACC_MAX` and `DP_SACC_MAX` to the accuracy you want MiniDP to
accept for hold. If you require GPS before arming, tune `ARM_HACC_MAX` and
`ARM_SACC_MAX` the same way.

## MiniDP Parameters

### Identity And Common ArduPilot Groups

| Parameter | Default | Use |
| --- | ---: | --- |
| `FORMAT_VERSION` | `0` | Internal storage format marker. MiniDP resets parameters when the expected format changes. |
| `LOG_BITMASK` | `-1` | MiniDP logging bitmask. `-1` logs all useful MiniDP and standard data. |
| `BRD_*` | Board defaults | Board configuration. |
| `SERIAL*` | Board defaults | Serial port setup. |
| `INS*` | Board defaults | IMU setup. |
| `COMPASS_*` | Board defaults | Compass setup. |
| `GPS*` | Board defaults | GPS setup. |
| `BATT*` | ArduPilot defaults | Standard ArduPilot battery monitor setup and failsafe thresholds. |
| `RSSI_*` | ArduPilot defaults | Standard ArduPilot RSSI setup. Use `RSSI_TYPE=5` for ELRS MAVLink mode. |
| `SCHED_*` | `SCHED_LOOP_RATE=400` | Standard ArduPilot scheduler setup. MiniDP uses Rover-style scheduler tasks for RC, GCS, sensors, logging, and output updates. |
| `LOG*` | Board defaults | Logger setup. |
| `NTF_*` | Board defaults | Notify/LED/buzzer setup. |
| `RC*` | Board defaults | RC input calibration and behavior. |
| `SERVO*` | Board defaults | Output function, min, trim, max, reverse, and related servo settings. |
| `MAV*` | Board defaults | GCS/MAVLink settings when GCS support is enabled. |
| `AHRS_*`, `EK2_*`, `EK3_*` | ArduPilot defaults | AHRS/EKF setup. |
| `BARO*` | Not registered by default | Barometer parameters only exist when `MINIDP_BARO_ENABLED=1` at compile time. |

MiniDP `LOG_BITMASK` bits are:

| Bit | Value | Logs |
| ---: | ---: | --- |
| `0` | `1` | Attitude, AHRS, EKF position/origin data. |
| `1` | `2` | GPS `GPS`/`GPA` records from the AP_GPS frontend. |
| `2` | `4` | Battery `BAT`/`BCL` records plus power/MCU status where supported by the board. |
| `3` | `8` | IMU and vibration records. |
| `4` | `16` | Compass records. |
| `5` | `32` | RC input and servo output records. |
| `6` | `64` | MiniDP controller, output, and arming/status records. |

The default `-1` enables all of these.

### Arming

| Parameter | Default | Use |
| --- | ---: | --- |
| `ARMING_REQUIRE` | `1` | Require MiniDP to be armed before active motor output. `0` allows output without an arm command, while the hardware safety switch can still block output. |
| `ARM_GPS_REQ` | `0` | GPS arming requirement: `0` none, `1` GPS fix and accuracy, `2` DP-ready EKF state plus GPS accuracy. |
| `ARM_HACC_MAX` | `10.0` | Maximum GPS horizontal accuracy in meters for GPS-required arming. `0` disables this accuracy check. |
| `ARM_SACC_MAX` | `2.0` | Maximum GPS speed accuracy in m/s for GPS-required arming. `0` disables this accuracy check. |

MiniDP also supports optional RC switch arming. Set `IN_RC_ARM` to a 1-based RC
channel and raise that channel to at least `IN_ARM_PWM` to arm on a switch edge.
Set `IN_RC_DISARM` and lower that channel to at most `IN_DISARM_PWM` to disarm.
The first healthy RC sample only initializes the switch state, so a switch that
is already high at boot does not arm the vehicle.

### Battery Monitor

MiniDP uses the standard ArduPilot `AP_BattMonitor` frontend. Configure
`BATT_MONITOR`, voltage/current pins, scalers, capacity, and low/critical
thresholds as you would on Rover or Copter. MiniDP publishes standard MAVLink
`BATTERY_STATUS` and fills the battery fields in `SYS_STATUS`.

MiniDP interprets `BATT_FS_LOW_ACT` and `BATT_FS_CRT_ACT` as:

| Value | MiniDP action |
| ---: | --- |
| `0` | Report/log only. |
| `1` | Enter MiniDP failsafe mode and drive outputs according to `OUT_FS_ACT`. |
| `2+` | Enter MiniDP failsafe mode and disarm. |

ArduPilot battery failsafes are evaluated while MiniDP is soft-armed.

### Output Safety

| Parameter | Default | Use |
| --- | ---: | --- |
| `OUT_DARM_ACT` | `0` | Disarmed action: `0` disables PWM, `1` sends neutral trim, `2` sends `OUT_FS_PWM`. |
| `OUT_FS_ACT` | `1` | Failsafe action: `0` disables PWM, `1` sends neutral trim, `2` sends `OUT_FS_PWM`. |
| `OUT_KILL_ACT` | `0` | RC kill action: `0` disables PWM, `1` sends neutral trim, `2` sends `OUT_FS_PWM`. |
| `OUT_FS_PWM` | `1500` | Failsafe PWM used when an output safe action is set to `2`. Set `0` to use each motor channel's trim value. |

MiniDP refreshes these output safety settings and each assigned motor channel's
`SERVOx_MIN`, `SERVOx_TRIM`, `SERVOx_MAX`, and `SERVOx_REVERSED` settings while
running.

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
| `FRAME_TYPE` | `901` | Boot-selected frame: `901` OMNI_PLUS, `902` DUAL_AZ_180_BOW. Reboot required; unknown values inhibit outputs. |
| `FRAME_SCR_POS` | `-1` | Legacy mounting annotation; no effect on yaw allocation. |
| `FRAME_SCR_YAW` | `1.0` | Motor1/Motor2 differential yaw scale for frame 901. Runtime clamp is `0..2`. |
| `FRAME_AFT_ARM` | `1.0` | Aft pod or stern tunnel longitudinal arm in metres; clamp `0.05..100`. |
| `FRAME_BOW_ARM` | `1.0` | Bow tunnel longitudinal arm in metres; clamp `0.05..100`. |
| `FRAME_HALF_SPAN` | `1.0` | Frame 901 screw lateral half-span in metres; clamp `0.05..100`. |
| `AZI1_ANG_MIN`, `AZI2_ANG_MIN` | `-90` | Minimum physical pod steering angle in degrees. |
| `AZI1_ANG_MAX`, `AZI2_ANG_MAX` | `90` | Maximum physical pod steering angle in degrees; combined span at most 180. |
| `AZI1_REV_FOLD`, `AZI2_REV_FOLD` | `1` | Allow reverse thrust when allocating pod vectors. |
| `AZIRATE` | `90` | Conservative pod steering rate in degrees/second (frame 902). |
| `IN_RC_SURGE` | `2` | 1-based RC channel for manual surge. `0` disables the axis. |
| `IN_RC_SWAY` | `1` | 1-based RC channel for manual sway. `0` disables the axis. |
| `IN_RC_YAW` | `4` | 1-based RC channel for manual yaw. `0` disables the axis. |
| `IN_RC_KILL` | `0` | 1-based RC kill channel. `0` disables RC kill. |
| `IN_KILL_PWM` | `1800` | Kill threshold PWM. A kill channel at or above this value forces `KILL`/failsafe handling. |
| `IN_RC_ARM` | `0` | 1-based RC arm switch channel. `0` disables RC switch arming. |
| `IN_ARM_PWM` | `1800` | Arm threshold PWM. The arm switch must rise to or above this value. |
| `IN_RC_DISARM` | `0` | 1-based RC disarm switch channel. `0` disables RC switch disarming. |
| `IN_DISARM_PWM` | `1200` | Disarm threshold PWM. The disarm switch must fall to or below this value. |

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
| `DP_YAW_I` | `0.0` | Heading-hold yaw integral gain for steady-state bias correction. |
| `DP_YAW_D` | `0.0` | Yaw-rate damping gain. |
| `DP_POS_P` | `0.0` | Position proportional gain for surge and sway. |
| `DP_POS_I` | `0.0` | Position integral gain for steady wind/current bias correction. |
| `DP_VEL_D` | `0.0` | Velocity damping gain for surge and sway. |
| `DP_SRG_MAX` | `0.5` | DP controller surge output limit. |
| `DP_SWY_MAX` | `0.5` | DP controller sway output limit. |
| `DP_YAW_MAX` | `0.5` | DP controller yaw output limit. |
| `DP_YAW_IMAX` | `0.2` | Maximum normalized yaw output from the yaw integral term. |
| `DP_POS_IMAX` | `0.2` | Maximum normalized surge/sway vector output from the position integral term. |
| `DP_RETARGET` | `1` | Relatch current yaw/position whenever `DP_HOLD` is requested. |
| `DP_POS_RAD` | `2.0` | Soft hold radius in meters. Position correction ramps in below this radius. |
| `DP_POS_DZ` | `0.5` | Position deadband in meters. Position correction is zero inside this distance. |
| `DP_HACC_MAX` | `10.0` | Maximum GPS horizontal accuracy in meters for DP hold. `0` disables this check. |
| `DP_SACC_MAX` | `2.0` | Maximum GPS speed accuracy in m/s for DP hold. `0` disables this check. |

Failsafe recovery is deliberate: remove the kill/fault condition, disarm, then
request `MANUAL` (custom mode 0). Re-arm separately. With `ARMING_REQUIRE=0`,
set it to 1 before recovery so clearing failsafe cannot immediately enable
thrust. Active battery or kill faults prevent clearing.

## Compile-Time Options

`MINIDP_BARO_ENABLED` defaults to `0` in `Config.h`. With the default build,
MiniDP keeps the shared ArduPilot barometer singleton present but does not
register `BARO*` parameters, initialize baro hardware, calibrate the barometer,
or update it in the main loop. No barometer is required. The no-baro build
uses GPS height for `EK3_SRC1/2/3_POSZ=3` and `EK2_ALT_SOURCE=2`, migrating
incompatible inherited barometer selections at startup while preserving other
non-barometer selections. The EKF still estimates in three dimensions even
though the controller uses only horizontal position and yaw. Build with `MINIDP_BARO_ENABLED=1` only if a
future MiniDP configuration intentionally needs barometer data.

## Ground Station Notes

MiniDP sends standard ArduPilot MAVLink heartbeats using
`MAV_AUTOPILOT_ARDUPILOTMEGA` and `MAV_TYPE_SURFACE_BOAT`, with custom modes
for MiniDP. It also sends MiniDP firmware identity text with
`AUTOPILOT_VERSION`, so Mission Planner users can see that the connected
firmware is MiniDP.

When `HEADING_HOLD` or `DP_HOLD` has a valid latched yaw target, MiniDP sends
`NAV_CONTROLLER_OUTPUT` with `nav_bearing` and `target_bearing` set to that
target heading. Mission Planner can use this as the target-heading line.

When `DP_HOLD` has a valid latched position target, MiniDP also sends
`POSITION_TARGET_GLOBAL_INT` and `POSITION_TARGET_LOCAL_NED`. Mission Planner
and MAVLink tools can use these as the DP target position marker.

Mission Planner's map "go here" / guided reposition action is accepted as a new
DP hold target when `AUTH_MAV_TGT=1`, yaw/origin/position/velocity are valid,
EKF is healthy, and GPS accuracy is inside `DP_HACC_MAX`/`DP_SACC_MAX`.

Once per second MiniDP sends `NAMED_VALUE_FLOAT` status values:
`DP_MODE`, `DP_OWN`, `DP_READY`, `DP_HOLD`, `DP_FALLB`, `DP_EKF_BAD`,
`DP_GPS_BAD`, `DP_TGT_ID`, `DP_SAT`, `DP_OUT`, `GPS_FIX`, `GPS_SATS`, `RC_OK`,
`RC_CH`, `RC_PROT`, `RC_AGE`, `RRC_CNT`, `RRC_AGE`, `RRC_CH`, `RRC_FLG`,
`RRC_C1`, `RRC_SYS`, `RCO_CNT`, `RCO_AGE`, `RCO_CH`, `RCO_C1`, `RCO_SYS`,
`RTK_FIX`, `DP_ERR_M`, `DP_TGT_N`, `DP_TGT_E`, `DP_YERR`, `DP_HACC`, and
`DP_SACC` when the underlying values are valid.

MiniDP accepts the standard `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` command for
Mission Planner board reboot and reboot-to-bootloader actions. The shared
ArduPilot reboot handler rejects reboot while soft-armed unless the command uses
the standard force value.

Use Mission Planner's servo output view to verify the selected frame's motor functions.
For passthrough outputs, assign an unused output to `RCIN1` through `RCIN16`
(`SERVOx_FUNCTION=51..66`) or `RCIN1Scaled` through `RCIN16Scaled`
(`SERVOx_FUNCTION=140..155`).

## DataFlash Logs

MiniDP writes compact vehicle-specific logs at about 10 Hz when logging is
enabled. Standard battery monitor records are logged as `BAT` and `BCL` when
`LOG_BITMASK` bit 2 is enabled.
Standard GPS records are logged as `GPS` and `GPA` when `LOG_BITMASK` bit 1 is
enabled; `GPA.RTCMFU` and `GPA.RTCMFD` show RTCM fragments used and discarded.

| Log | Use |
| --- | --- |
| `MDTG` | DP target/state trace: mode, target id, validity flags, target yaw, current yaw, NE position error, NE velocity, GPS horizontal accuracy, and GPS speed accuracy. |
| `MDAX` | Axis trace: mode, control owner, output state, raw surge/sway/yaw command, and post-limiter surge/sway/yaw command. |
| `MDOT` | Output trace: output state, mixer saturation flag, Motor1 through Motor6 normalized demand, Motor1 through Motor6 PWM, and active PWM count. |
| `MDST` | MiniDP status trace: armed/soft-armed state, arming requirements, last arm rejection, mode, owner, output state, link/kill status, GPS fix, and satellite count. |

`MDTG.Flags` bits are: bit 0 target yaw valid, bit 1 target position valid,
bit 2 state yaw valid, bit 3 state position valid, bit 4 state velocity valid,
bit 5 origin valid, and bit 6 EKF healthy.

## Developer Checks

Typical local checks:

```sh
./waf configure --board revo-mini
./waf --targets bin/MiniDP
./waf configure --board sitl
./waf --targets tests/test_minidp_mode,tests/test_minidp_authority,tests/test_minidp_input,tests/test_minidp_axis_limiter,tests/test_minidp_controller,tests/test_minidp_output,tests/test_minidp_actuator_test,tests/test_minidp_arming,tests/test_minidp_dynamics,bin/MiniDP
build/sitl/tests/test_minidp_mode
build/sitl/tests/test_minidp_authority
build/sitl/tests/test_minidp_input
build/sitl/tests/test_minidp_axis_limiter
build/sitl/tests/test_minidp_controller
build/sitl/tests/test_minidp_output
build/sitl/tests/test_minidp_actuator_test
build/sitl/tests/test_minidp_arming
build/sitl/tests/test_minidp_dynamics
python3 Tools/MiniDP/tests/test_sitl.py
```

The SITL script requires `pymavlink` and local TCP access, uses a temporary
parameter store, and prints its artifact directory. It checks transport,
parameters, RCIN, mode/target transactions, motor tests and failsafe recovery.
The C++ dynamics tests use allocated forces in a planar model with drag and
steady disturbances. They establish control regression coverage, not physical
hull tuning or servo performance. The stock Rover SITL model does not represent
MiniDP thruster geometry. The custom MiniDP-F405 board definition remains a
placeholder pending the actual schematic; use an existing matching board.

When adding new MiniDP parameters or behavior, update this file in the same
change.
