# Coordinated VSP allocator — AZ2.4 development revision

## Scope

`VspAllocator` is a host-testable pair owner wrapping the existing
`VesselThrustAllocator` minimum-effort solver. `NavigationHeading` supplies yaw;
`NavigationDP` supplies translation in DP. Heading Hold takes the arithmetic
mean of the two manual body-frame stick vectors for translation, and replaces
manual differential yaw with heading control. One shared joystick may be routed
to both drives. Free mode retains the existing manual and VRS paths.

AFT (`VSP_CONF=1`) uses both VSPs plus a configured, available bow if selected.
With bow force zero, AFT also operates on its pair alone. VWT (`VSP_CONF=2`)
uses both VSPs, never allocates bow or stern, and requires neither tunnel
thruster. A positive sway request at forward VSP positions produces opposing
longitudinal components to cancel unwanted yaw. At aft positions these
components reverse. Stern remains independent manual control in both layouts.

The force solver uses body axes +forward, +starboard and +clockwise yaw;
M = sum(x*Fy - y*Fx). The normalized commands are scaled to relative force
and moment units before solving. Existing uniform saturation is retained;
all selected actuators use a common interpolation factor limited by their
individual slew rates. The modeled achieved demand is not a measured force.
No yaw-priority optimizer, RPM-dependent force model, DP integral control or
hydrodynamic compensation is introduced by this revision.

## Parameters

New indices 40–48 in the VNAV group are appended, all default to zero.
Existing parameter IDs and routing IDs are unchanged.

| Parameter | Meaning |
|---|---|
| VNAV_VSP_EN | 1 enables coordinated VSP navigation. At 0, legacy tunnel Heading Hold can use the stern tunnel in VWT. |
| VNAV_VSP1_X | Port VSP longitudinal position, m |
| VNAV_VSP1_Y | Port VSP transverse position, m, negative |
| VNAV_VSP1_F | Port force capacity at full vector |
| VNAV_VSP2_X | Starboard VSP longitudinal position, m |
| VNAV_VSP2_Y | Starboard VSP transverse position, m, positive |
| VNAV_VSP2_F | Starboard force capacity at full vector |
| VNAV_VSP_BOW_F | AFT bow capacity at THR_BOW_HDG_MAX; 0 excludes bow, ignored in VWT |
| VNAV_VSP_RPM | Deprecated. Retained for parameter compatibility. Navigation ignores this threshold. |

Set both `VSP1_VMODE` and `VSP2_VMODE` to 1 and configure the mapper (rod
centers, limit, rotation, installation angle, deadband, slew) first. AFT
requires negative x for each VSP, VWT nonnegative x. Geometry is relative to
the same reference point as EKF position/velocity. Forces may initially be
relative units but must be consistent; yaw uses force-metres. Positive
`VNAV_AL_MAX_SRG`, `VNAV_AL_MAX_SWAY`, `VNAV_AL_MAX_YAW` are required. Set
heading and DP gains/limits and enable flags using existing navigation
parameters. `VNAV_VSP_EN=1` is an explicit opt-in, not a commissioning recipe.

For AFT with bow, enable and configure the bow's existing heading authority,
positive lever arm and heading maximum. The bow's heading polarity must mean
positive physical starboard thrust after all route and PWM calibration.
Set its force capacity at that maximum, not at full unrestricted PWM.
A configured AFT bow is required: no silent reallocation following loss.

## Route/readiness and transition contract

- Both VSP rod inputs must have direct valid RC producers. Both drives must be
  in vector mode, with valid calibration and the startup neutral check complete.
- Each of the four rod outputs (plus the selected AFT bow output) must have
  exactly one terminal PWM route that passes the existing HAL readiness check.
  Component/function/sequencer consumers of these outputs are not supported
  for automatic allocation. This restriction prevents downstream consumers
  from sampling a preliminary manual command before navigation replacement.
- `Routing::prepare` calculates manual input readiness. The pair owner
  replaces both rod output pairs before terminal publication. VRS is inhibited
  by vector mode and pair authority. RPM remains on its existing independent
  input/output or direct RC route. Measured RPM remains available for telemetry.
- Batch publication uses the existing HAL cork/push mechanism. The required
  mask is four rod axes, plus bow only for AFT when selected. A rejected write
  rolls back all selected actuators to their configured neutral where the HAL
  accepts it; failed channels are disabled. This is software acceptance,
  not physical actuator/DroneCAN feedback or guaranteed synchronous movement.
- Normal Free, Heading Hold and Full DP transitions carry the current actuator
  outputs into the next mode and slew toward its demand. Heading Hold and DP
  share the captured heading. DP captures a new position on entry. If a new
  DP request is waiting for a valid estimate or centered sticks, an existing
  Heading Hold stays engaged and the supervisor reports `DP_WAIT`.
- A valid Free selection is required after active control loses readiness,
  configuration or estimator validity, or after DP pilot override when
  `VNAV_MAN_OVR` is enabled. AFT bow stick intervention also revokes pair
  ownership. Fault recovery holds neutral until Free, then checks manual
  neutral before re-engagement.
- Do not edit routing, PWM calibration, mapper settings or geometry while
  under thrust. Live route edits can remove the ability to command neutral;
  independent external emergency stop remains necessary. Verify calibration
  and rotor direction on the bench with propulsion mechanically safe.

## Status contract

Existing `NAV_STATE` values remain unchanged. `HDG_ACTIVE` (2) and `DP_ACTIVE`
(4) require actual software publication acknowledgement, not merely valid
controller maths. `NAV_RBITS` adds bit 15, `0x8000`, for VSP or route readiness.
Bit 14 (`0x4000`) is rearm required and bit
12 (`0x1000`) is bow unavailable, never required by VWT. Existing reason bits
are preserved. `VSP1_AUTH` / `VSP2_AUTH`: 1 manual, 2 automatic yaw, 4 VRS,
8 neutral guard, 16 DP translation. Normal allocated HH = 3, DP = 18.

New `VSP_AL_ST` is a 16-bit integer bitmask, emitted at `VSP_TEL_HZ` via
MAVLink NAMED_VALUE_FLOAT and VTEL source 262 (2-byte number).

| Bit | Hex | Meaning |
|---:|---:|---|
| 0 | 0x0001 | Coordinated VSP navigation selected |
| 1 | 0x0002 | Pair vector inputs and rod routes ready |
| 2 | 0x0004 | Geometry, capacity, and allocator limits valid |
| 3 | 0x0008 | Bow included in this allocation |
| 4 | 0x0010 | Demand uniformly saturated |
| 5 | 0x0020 | Common actuator slew limited |
| 6 | 0x0040 | Heading/yaw ownership confirmed |
| 7 | 0x0080 | DP translation ownership confirmed |
| 8 | 0x0100 | Free/rearm required |
| 9 | 0x0200 | Output rejected (latched until Free) |
| 10 | 0x0400 | Coordinated neutral guard |
| 11 | 0x0800 | Pilot override detected |
| 12 | 0x1000 | Configured AFT bow unavailable |
| 13–15 | — | Reserved |

Status is per cycle except latched rearm/output rejection; saturation and
slew bits do not imply loss of ownership. `VSP_AL_ST` is zero when disabled.

## Verification

Run `python tests/test_vsp_allocator.py` for host allocation/ownership tests;
`python tests/test_main_propulsion.py` for production Logic/Routing tests
with fake HAL writes in both memory profiles. Also run the existing
allocator, supervisor, VSP vector, thruster and VTEL tests. Target ArduPilot
build, actual hardware/DroneCAN publication, SITL estimator-to-output loop,
parameter persistence and vessel performance require separate validation.
