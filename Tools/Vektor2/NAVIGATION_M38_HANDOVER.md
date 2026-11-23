# Vektor2 M3.8 — Historical RC selection design (superseded)

**Superseded by M3.8.1:** This document describes the original two-action
selector design, including historical two-way RC examples which are NOT
supported by the current three-position NavMode action. For current single-slot `NavMode` configuration (type 7),
read [NAVIGATION_M381_HANDOVER.md](NAVIGATION_M381_HANDOVER.md). Old type
5/6 settings are no longer accepted by the current source.

## Original M3.8 notes (for migration only)

**Source baseline:** M3.5 EKF3 and telemetry. **New scope:** use an assignable
RC switch to request Free, Heading Hold or DP so the vessel-agnostic controllers
can capture targets and calculate demands on the board. The VSP, bow and
stern propulsion routing remains exactly as before, with **no automatic
ownership or allocator-to-PWM wiring**.

## Implemented

- Added VFN action **5 = `NavHeading`** and **6 = `NavDP`** to the existing ten
  function slots. Inputs use the existing `RTn_SRC` and `RTn_DST=201..210`
  assignment routes. No dedicated RC channel or new switch parser.
- Navigation actions use **level** trigger only (`VFNn_MODE=0`), not a latch.
  No match selects Free. 2-way Free/DP uses one NavDP slot; 3-way Free/Heading/DP
  uses one NavHeading slot and one NavDP slot on the same RC source.
- Invalid, disabled, disconnected, misrouted or overlapping mode selectors
  request Free. The selection is not silently combined with VFN type 2 (the
  older `Auto heading` enable gate). `VNAV_MODE` mirrors the selected request
  in **volatile RAM** without a parameter save. Remove assignments to resume
  manual `VNAV_MODE` control (at Free after removal).
- Added selectable VTEL source **260 `DP_CALC`** and **261 `HDG_CALC`**, each
  zero or one depending on whether the corresponding controller has a valid
  mathematical output this cycle. The same values are sent as MAVLink named
  floats under existing `DP_TEL_HZ=0..50`. These flags do **not** mean real
  propulsion ownership.
- Preserved unified `NAV_STATE` / `NAV_RBITS`, all previous VTEL source IDs,
  AHRS/EKF3 navigation inputs, targets, reason bits, gains and math. Controller
  updates remain internal; no `VesselDemand` is routed to physical outputs.

## Board setup example: three-position switch on RC7

Use an otherwise unused VFN slot pair and route pair. The example assumes the
RC switch sends approximately **1000 / 1500 / 2000 microseconds**. Verify the
actual RC7 samples before assigning ranges. Change the channel and slot/route
numbers to suit existing Vektor routing without overwriting active routes.

| Parameter | Value | Meaning |
|---|---:|---|
| `RT1_SRC` | 7 | RC7 -> first action |
| `RT1_DST` | 201 | First action input VFN1 |
| `RT2_SRC` | 7 | RC7 -> second action |
| `RT2_DST` | 202 | Second action input VFN2 |
| `VFN1_ENABLE` | 1 | Heading selection active |
| `VFN1_TYPE` | 5 | Select Heading Hold |
| `VFN1_MODE` | 0 | Level |
| `VFN1_LOW` | 1400 | Middle position low threshold |
| `VFN1_HIGH` | 1600 | Middle position high threshold |
| `VFN2_ENABLE` | 1 | DP selection active |
| `VFN2_TYPE` | 6 | Select DP |
| `VFN2_MODE` | 0 | Level |
| `VFN2_LOW` | 1800 | High position low threshold |
| `VFN2_HIGH` | 2200 | High position high threshold |

This yields **low=Free, middle=Heading, high=DP**. For a two-position Free/DP
switch, configure just VFN1 with type **6**, `LOW=1800, HIGH=2200`, and one
route to VFN1. Leave other nav selector types unassigned (`TYPE=0`).

Enable the calculation stages separately:

| Parameter | Example for diagnostic testing | Notes |
|---|---:|---|
| `VNAV_ENABLE` | 1 | Navigation master enable |
| `VNAV_HDG_ENABLE` | 1 | Shared heading controller enable |
| `VNAV_DP_ENABLE` | 1 | DP controller enable |
| `VNAV_HDG_KP` | 0.02 | Nonzero heading gain |
| `VNAV_HDG_KD` | 0.005 | Optional heading damping |
| `VNAV_HDG_MAX_YAW` | 100 | Normalized output ceiling (%) |
| `VNAV_DP_POS_KP` | 0.25 | Position to desired speed |
| `VNAV_DP_VEL_KP` | 0.6 | Velocity-error gain |
| `VNAV_DP_MAX_SPD` | 0.5 | Speed limit in m/s |
| `VNAV_DP_ACC_LIM` | 0.2 | Acceleration limit, m/s^2 |
| `VNAV_DP_RADIUS` | 0.5 | Position deadband (m) |
| `VNAV_DP_VEL_FILT` | 0 | Use EKF velocity unfiltered initially |
| `DP_TEL_HZ` | 5 | MAVLink named-float diagnostic reports, 0..50 |

**These gains are examples for observation only, not tested propulsion tuning
values.** Gains and limiter parameters default to safe/non-operational zero
values, so the switch alone will not generate valid correction demands.

## What to expect on the board

1. Confirm RC input and GNSS/EKF3 yaw, horizontal position and velocity are
   valid. Stationary GPS-derived yaw without a compass may be unavailable.
2. With the low switch position, `VNAV_MODE=0`, DP targets clear, `DP_CALC=0`.
3. Move to the middle position. `VNAV_MODE=1`, current heading is captured once
   estimator and master enables are valid; `HDG_CALC=1` means the yaw demand
   maths is running. Position target is intentionally absent.
4. Move to high. `VNAV_MODE=2`, current position is captured when horizontal EKF
   estimates are valid, and heading target is preserved. `DP_CALC=1` reports a
   valid normalized correction request, e.g. `DP_CMD_N/E` and `DP_CMD_X/Y`.
   A change to position/velocity will change those values in telemetry.
5. `NAV_STATE` correctly stays `HDG_WAIT` / source 140 value 1 while automatic
   outputs are disconnected, and `NAV_RBITS` includes `OUTPUT_NOT_READY`
   (`0x0040`). **This is not a DP-calculation failure.** To diagnose controller
   maths, read `DP_CALC`, `HDG_CALC`, `DP_DIST`, target/actual coordinates and
   `NAV_RBITS` for other blockers.
6. Missing EKF estimates, heading, controller gain/config errors, or estimator
   resets produce `DP_CALC=0` and relevant `NAV_RBITS`, and prevent stale demand.
   After a reset, switch back to Free and then DP to recapture. RC loss also
   selects Free (subject to channel/update timings).

VTEL `VTEL_SLTn=260/261` makes calculation validity available on assigned
receiver fields independently of MAVLink rate. The previous DP sources
`240..259`, status `140/141` and VFN diagnostic inputs `110..139` are retained.
VTEL snapshots remain receiver-polled (typically 10 Hz); `DP_TEL_HZ` affects
MAVLink DP named floats only. Sending all fields at 50 Hz may exceed the serial
link budget, so start with 5 Hz for on-board testing.

## Verification and constraints

- Pure host tests cover 2-way and 3-way selection, transitions, duplicate
  selector overlap, routing/input loss, toggle rejection and the supervisor's
  refusal to claim `ACTIVE` without an actuator owner.
- Full ArduPilot firmware build, SITL, board and vessel runs have **not** been
  completed with this standalone application archive. Full integration needs
  verification against the board's ArduPilot tree and actual RC channels.
- The physical DP reference is still the AHRS/EKF3 body origin. Confirm this
  matches the intended vessel point before any operational DP use.
- M4 retains all pending manual ownership, neutral handover, failsafe and
  physical propulsion integration work. **No physical DP control in M3.8.**
