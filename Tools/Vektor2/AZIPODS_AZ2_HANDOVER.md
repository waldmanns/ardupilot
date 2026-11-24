# Vektor2 — AZ2: limited-angle azipod manual control

Status: AZ2.2 adds propulsion-aware VTEL sources to AZ2.1 layout-driven activation. ArduPilot target build,
SITL, and hardware commissioning remain outstanding. This supersedes AZ1's
abstract-output behavior, not its architecture. Read the AZ1 handover for
persistent IDs and boot-selected storage design. Keep future work narrowly scoped.

## Scope and architecture

- `MAIN_PROP_TYPE=0` remains the default VSP configuration; `1` constructs only
  the AZI backend. `AZI_CONF=1` selects two aft azipods, AZI1 port and AZI2
  starboard, with both installed drives selected automatically. Both selectors
  are latched after AP_Param load at boot. Unspecified or unsupported layouts
  select no azipods. `azipod_count()` in `MainPropulsion.h` describes installed
  drive count; future layouts may extend it along with matching runtime and
  routing support. No extra drive state is allocated now.
- Bow and stern tunnel thrusters remain independent. The intended first AZI
  vessel uses the bow tunnel thruster only, if independently enabled and routed.
- Each pod is a separate instance of `Components::AzipodControl`. One small
  implementation in `AzipodControl.h/.cpp` contains a pure Cartesian mapper,
  limited-angle solver and stateful servo/ESC command generation. No HAL or
  navigation dependency is added to this controller.
- Each pod supports 180-degree, 270-degree and custom limited-angle PWM
  position servos, asymmetric travel and installation offsets. Forward-only
  and reversible PWM ESCs are supported. Continuous 360-degree positioning,
  encoders, closed-loop steering motor controllers, coordinated allocation,
  navigation authority and AZI-specific MAVLink named values / logging are not implemented.
  AZI receiver telemetry is available through VTEL (see below).
- The shared union and four runtime component slots from AZ1 remain unchanged.
  Both parameter families remain statically registered. Host sizes after AZ2:
  VspBackend 288 B, AzipodBackend 124 B, shared storage 292 B, Logic 976 B
  (host mock AP_Param ABI, NOT measured MCU RAM). The VSP backend is still
  larger; new AZI controller state does not enlarge the union on this host.

## Coordinate and signal contract

Positive X = ahead; positive Y = starboard. Body angles clockwise from ahead.
Both input axes are logical 1000..2000 us, centered on 1500. Radial deadband
is subtracted and the remainder scaled to full demand at radius 500 us;
diagonals are clamped to unit magnitude. A neutral vector holds the last
commanded servo angle and stops the motor. No vessel-level joystick mixer is
added. Each pod requires both independent inputs to be valid.

Mechanical angles are **unwrapped** relative to mechanical zero; do not use
circular shortest-path distance across mechanical stops. Body thrust angle
with positive motor direction = mechanical angle + OFFSET. Negative motor
thrust adds 180 degrees. For each direction, the solver enumerates equivalent
angles within the mechanical interval. With reverse optimization enabled,
travel distance plus the direction-change hysteresis cost determines the
solution. With optimization disabled, a reachable forward solution wins;
reverse is still used when it is the only exact solution. An exact solution
always takes precedence over projection. A reversible pod with at least 180
degrees of usable travel can cover every body direction geometrically. This
is not a guarantee of equal forward and reverse thrust or instantaneous response.

Unreachable policy 0 stops the motor and retains the last angle. Policy 1
selects the best achievable direction by positive cosine projection and
reduces thrust accordingly; zero or negative projection means stop. This can
introduce unwanted vessel forces, so policy 0 is the default.

## Parameters

All listed calibration settings are latched at boot (reboot after changing).
AZI1_ and AZI2_ use identical metadata, with independent values. Layout is the
sole installed-drive selector. No individual enable parameters or migration
logic exist. Calibration indices 2..13 and existing component IDs are unchanged.

| Suffix | Default | Meaning / accepted values |
|---|---:|---|
| ANG_MIN | -90 | Mechanical angle at logical 1000 us, degrees |
| ANG_MAX | 90 | Mechanical angle at logical 2000 us, degrees |
| OFFSET | 0 | Body thrust direction at mechanical zero, degrees |
| REV | 0 | 0 forward-only ESC, 1 reversible ESC |
| REV_OPT | 1 | 0 prefer forward when reachable; 1 travel optimization |
| UNREACH | 0 | 0 inhibit, 1 positive cosine projection |
| DB | 25 | Radial joystick deadband in us, 0..250 |
| RATE | 90 | Conservative commanded/estimated servo speed, deg/s, 1..720 |
| REV_HYST | 15 | Direction-change cost in degrees, 0..180 |
| REV_DLY | 500 | Neutral reversal dwell in ms, 0..5000 |
| ALIGN | 5 | Maximum estimated steering error for thrust, deg, 0..45 |
| SLEW | 1 | Motor slew, normalized units/s, >0..100 |

Angles must lie within -360..360 with ANG_MIN < ANG_MAX and total travel <360.
OFFSET must be -360..360. 180-degree example: -90/+90. 270-degree example:
-135/+135. Custom example: -100/+140. Binary settings must be 0 or 1.
Invalid calibration inhibits both outputs for that pod. Do not choose a larger
travel range than the installed mechanism can safely achieve.

## Routing and PWM calibration

| Component | X input | Y input | Servo output | Motor output |
|---|---:|---:|---:|---:|
| AZI1 | DST 141 | DST 142 | SRC 141 | SRC 142 |
| AZI2 | DST 151 | DST 152 | SRC 151 | SRC 152 |
| Bow thruster | DST 121 | — | — | SRC 121 |

Example only (choose channels that are actually available on the board):

| RT slot | SRC | DST | Meaning |
|---:|---:|---:|---|
| 1 | 1 | 141 | RC1 -> port X |
| 2 | 2 | 142 | RC2 -> port Y |
| 3 | 141 | 1 | Port servo -> PWM1 |
| 4 | 142 | 2 | Port ESC -> PWM2 |
| 5 | 3 | 151 | RC3 -> starboard X |
| 6 | 4 | 152 | RC4 -> starboard Y |
| 7 | 151 | 3 | Starboard servo -> PWM3 |
| 8 | 152 | 4 | Starboard ESC -> PWM4 |
| 9 | 5 | 121 | RC5 -> bow thruster |
| 10 | 121 | 5 | Bow thruster -> PWM5 |

Each pod requires exactly one direct identity-mapped AZ_OUT->PWM and one
MOT_OUT->PWM route; non-PWM consumers and transformed AZI actuator routes
are rejected, and multiple destinations for either output inhibit operation.
Existing per-channel `PWM<n>_MIN/MAX/INV` parameters provide physical pulse
calibration and steering inversion. Do not add another PWM calibration layer
inside AzipodControl. Verify actual parameter naming against MAVLink list.
The motor stop is **logical 1000 us for forward-only ESCs and logical 1500 us
for reversible ESCs**. The latter maps to the midpoint of physical PWM MIN/MAX.
Ensure actual ESC stop/neutral is exactly that calibrated physical pulse.
Use a suitable symmetric range around reversible ESC neutral. Avoid motor
channel inversion unless the entire ESC mapping is explicitly verified.
Servo endpoint calibration must map logical 1000 and 2000 to the configured
mechanical limits, respectively. Mechanical installation offset is separate.

AZ2 requires locally available PWM outputs as checked by existing
`PwmOut::heading_output_ready()` (valid channel calibration, HAL PWM output
available, safety gate not disarmed). This does not prove physical actuator
response. CAN-only PWM channels are not granted AZI authority in this revision.
Existing unrelated CAN behavior is unchanged. Never configure a direct RC or
other bypass route to the physical propulsion channels. There is no systemwide
arming mechanism, nor independent hardware identity tracking of outputs.
Do not edit live PWM calibration or routing while motors are connected.

## State and protection

At startup, route revision, output rejection or loop gap >100 ms,
command the mechanical midpoint, stop the motor and wait at least full travel
at RATE plus 250 ms, and at least REV_DLY. Require a valid centered X/Y vector
before thrust. No external position feedback is available: RATE is an estimate,
not a measured servo speed, and ALIGN is a commanded/estimated error threshold,
not jam detection. Set RATE conservatively from measured loaded servo movement.

During active control, move the servo command at RATE. Inhibit motor thrust
while the preceding commanded angle is farther than ALIGN from the target;
within ALIGN, apply cosine reduction and slew toward the requested thrust.
A change of motor direction first stops the motor and respects REV_DLY before
applying the opposite direction. Safety stops bypass the ordinary slew limit.
Zero demand stops immediately and holds steering. RC invalidity (including
when input routes originate elsewhere), missing/invalid axis, route failure,
and stale loop timing stop thrust and require neutral to re-arm. Installed pods
with valid calibration emit an explicit stop on a remaining usable motor route
when steering is unavailable. Steering is written before motor; a rejected
steering write substitutes motor stop and forces reinitialization. A rejected
motor write similarly forces reinitialization. Output validity describes the
pulse command, not input validity or actual actuator movement.

Uninstalled pods, invalid calibration/layout, unselected backends and missing or
unavailable motor outputs may have **no PWM signal**. Configure ESC no-signal
failsafe to stop and verify it physically. A disconnected/jammed servo cannot
be detected by software. A successful PWM write does not verify ESC or servo
response. Do not commission on the water until all checks below pass.

## Verification

Ran `python3 tests/test_azipod.py`: 57,600 angle/configuration combinations,
including 180/270/custom asymmetric travel, offsets, both ESC types,
optimization on/off and both unreachable policies. Stateful tests exercise
8 hardware/optimization combinations, startup, neutral, reversal, steering,
RC loss, invalid inputs, route loss, timing gaps and invalid config.
Compiled using C++11, warnings-as-errors, ASan/UBSan.

Ran `python3 tests/test_main_propulsion.py`: compact and large profiles passed
with production Logic/Routing/RouteParameters and host HAL/parameter adapters.
Tests include backend exclusivity, legacy VSP outputs, independent tunnel
thrusters, AZI command publication, neutral startup, independent pod operation,
RC loss, steering route readiness/write rejection, malformed routes and
layout-only activation, unsupported layouts and boot latching. C++17 host integration, ASan/UBSan.

All 12 existing standalone host regression suites passed: functions, heading
hold, navigation DP, navigation RC, navigation supervisor, roll stabilization,
sequencer, thrusters, vessel thrust allocator, VSP mapper, VSP vector and
VTelemetry. The measure-points test needs external board hwdef files.
Updated `tests/test_mavlink_main_propulsion.py` for layout-only activation, layout persistence and AZ2 startup commands;
not executed because the ArduPilot dependency tree and target binaries are
not in this archive. Actual AP_Param persistence, target compilation, flash/RAM
usage and hardware actuator behavior remain unverified.

## Next steps

1. Build the actual supported ArduPilot target and run SITL/MAVLink persistence
   tests, including boot switching, both ESC types and real PWM channel readiness.
2. Bench commission one pod at a time with propellers removed and ESC power
   isolated while calibrating servos. Verify travel and mechanical stops, body
   thrust orientation, physical ESC neutral, no-signal failsafe, neutral startup,
   reversal dwell, loss of RC, and safe route changes. Use conservative
   speed and thrust settings and check actual loaded servo lag. Then perform
   controlled low-power propulsion tests before installing the second pod.
3. Only after manual operation is validated, plan AZ3 coordinated twin stern
   azipod/bow thruster control. Leave navigation/allocator and VSP algorithms
   unchanged until their own milestone. No continuous-360 steering in AZ2.

## AZ2.2 — VTEL catalog

Added source IDs 270–272 (active propulsion family and respective layout),
280–285 (AZI1), and 290–295 (AZI2) for logical steering/motor PWM, commanded
and target mechanical angle, signed motor command and controller state.
`VTelemetryPropulsion.h` is a read-only adapter to the existing Logic and
controller state, with inactive/blocked command samples rejected. No new
controller state or parameter, no changes to VSP or AZI control algorithms,
routing, navigation, logging, MAVLink or receiver protocols. See `VTEL.md`
for exact IDs, encoding and state semantics. Layouts remain predefined;
no speculative sources for AZI3/AZI4.

Verification: `python3 tests/test_vtelemetry.py`,
`python3 tests/test_main_propulsion.py` (both profiles), and
`python3 tests/test_azipod.py` passed. The integration test exercises the
actual read-only propulsion adapter and confirms inactive-family isolation,
boot-latched AZI configuration, independent drive commands, homing/ready
state and output-failure suppression. Host runtime sizes remain unchanged
(VSP 288 B, AZI 124 B, shared 292 B, Logic 976 B). Target build and receiver
hardware tests remain outstanding.

## AZ2.3 follow-up

The direct visual calibration interface is now specified in
[AZIPODS_AZ2_3_HANDOVER.md](AZIPODS_AZ2_3_HANDOVER.md). The client agent must
read [AZIPODS_AZ2_3_CLIENT.md](AZIPODS_AZ2_3_CLIENT.md) before implementing
the widget. It uses existing PWM calibration, with no new persistent parameters.
