# Vektor2 Navigation — M3.8.1 observation-only NavMode switch

One `NavigationSupervisor` is responsible for `NAV_STATE` and `NAV_RBITS`.
`VNAV_MODE` requests 0=Free, 1=Heading Hold or 2=Full DP.
M3.8.1 uses one `VFNn_TYPE=7` (`NavMode`) slot and one routed RC source.
The NavMode action **requires a three-position RC switch**, not a two-way
switch. With LOW/HIGH set to 1300/1700 us, 800..1249 selects Free,
1350..1649 Heading and 1750..2200 DP. The intervening 1250..1349 and
1650..1749 us transition zones hold the last stable mode (no change).
This +/-50us guard around each threshold avoids switch-transition chatter.
Invalid sources, configurations, duplicate selectors or retired type 5/6
assignments fail to Free. Firmware cannot infer physical switch positions
from a static PWM sample: verify three distinct levels on the board.
The selected mode is mirrored into `VNAV_MODE` in RAM without a flash save.
See [NAVIGATION_M381_HANDOVER.md](NAVIGATION_M381_HANDOVER.md).
**No automatic propulsion is active in M3.8.1**. Existing manual VSP/thruster
routing remains unchanged, and the supervisor deliberately reports `HDG_WAIT`
when automatic actuator ownership has not been acknowledged.

## Controller core

- `NavigationHeading` is the **single** shared heading loop in Heading Hold and
  Full DP. It captures heading, wraps angular error at 360 degrees and produces
  a normalized yaw demand. Its yaw-rate calculation compensates for roll/pitch
  using attitude and body gyro rates, not just the Z gyro axis.
- `NavigationDP` has no VSP, bow-thruster or layout dependencies. It captures
  horizontal E7 latitude/longitude, compares it with the EKF/AHRS actual
  position, calculates N/E position error, limits desired speed, applies
  velocity damping, limits horizontal acceleration and converts demand into
  vessel surge/sway **using yaw only**. Demands are normalized requests, **not
  measured force**. Drift damping continues inside `VNAV_DP_RADIUS`.
- Roll/pitch do not become surge/sway commands. The EKF horizontal position
  and velocity must be for the **same vessel control point** used for target
  capture. Verify GNSS lever-arm / EKF offsets separately before sea trials;
  no new antenna lever-arm compensation is invented here.
- Captures are performed on mode transitions or explicit rising edges of
  `VNAV_HDG_CAPTURE`, `VNAV_DP_CAP_HDG` and `VNAV_DP_CAP_POS`, once the
  associated estimator conditions are good. Switching to Free or detecting
  an EKF yaw, horizontal position or horizontal velocity reset clears captured
  targets. The AHRS reset timestamps are used directly; after a reset, an
  explicit recapture or fresh mode transition is required.
- `VNAV_DP_POS_KP`, `VNAV_DP_VEL_KP`, `VNAV_DP_MAX_SPD`,
  `VNAV_DP_ACC_LIM`, `VNAV_DP_RADIUS` and `VNAV_DP_VEL_FILT` provide the
  initial simple PD control. They default to safe non-operational values;
  vehicle-specific tuning is required. No integral or wave-prediction model.

## Reason bits: one public mask

`NavigationDP::Output::reason_bits` holds DP-specific diagnostic reasons
using the existing `NavigationSupervisor::Reason` definitions. Multiple
bits may be set at once: heading, position, velocity, target, configuration,
timing and estimator reset. An invalid estimate/configuration/target results
in **zero DP demand**, not a stale previous command.

`NavigationSupervisor` merges these bits **only in requested DP mode** into
`NAV_RBITS`. There is no second `DP_RBITS` telemetry state machine.
An invalid DP-only parameter must not inhibit otherwise valid Heading Hold.
The supervisor also adds mode-aware enable, RC, output-ownership and other
readiness reasons.

| Bit | Hex | Meaning |
|---:|---:|---|
| 0 | `0x0001` | automation disabled |
| 1 | `0x0002` | heading invalid |
| 2 | `0x0004` | position invalid |
| 3 | `0x0008` | horizontal velocity invalid |
| 4 | `0x0010` | RC invalid |
| 5 | `0x0020` | pilot override |
| 6 | `0x0040` | output not ready / no ownership ACK |
| 7 | `0x0080` | wait neutral |
| 8 | `0x0100` | target invalid / missing |
| 9 | `0x0200` | estimator reset |
| 10 | `0x0400` | configuration invalid |
| 11 | `0x0800` | timing fault |

`NAV_STATE`: 0=FREE, 1=HDG_WAIT, 2=HDG_ACTIVE, 3=DP_WAIT (heading active),
4=DP_ACTIVE, 5=FAULT. ACTIVE is never inferred merely from a valid
controller calculation. Final live actuator ownership must acknowledge it.

## Diagnostic telemetry

`DP_TEL_HZ` is a top-level standard rate parameter: **0 disables** the
MAVLink `NAMED_VALUE_FLOAT` DP/heading diagnostic stream, 1–50 Hz requests
its transmission, values above 50 are capped. The rate is a best-effort
maximum and depends on link bandwidth. Each snapshot includes:

- Target geographic E7 position split into exactly representable integer
  degree and signed fractional E7 float fields (`DP_TLAT_I/F`, `DP_TLON_I/F`).
- Position error (`DP_DIST`, `DP_BRG`, `DP_ERR_N`, `DP_ERR_E`).
- Horizontal velocity (`DP_VN`, `DP_VE`).
- Normalized vessel-frame surge/sway and horizontal magnitude/bearing
  (`DP_CMD_X/Y`, `DP_CMD_MAG/BRG`).
- `HDG_TGT`, `HDG_ACT`, `HDG_ERR`, `HDG_CMD` from shared heading control.
- `DP_CMD_N` and `DP_CMD_E`: requested normalized earth-frame correction
  components, distinct from body-frame `DP_CMD_X` / `DP_CMD_Y`.
- Actual heading is observable whenever EKF/AHRS heading is valid, even when
  controller ownership is disabled; inactive yaw demand is still unavailable.

Unavailable readings are NaN, not fabricated zeros. Actual EKF geographic
position is already available via standard MAVLink; exact target and actual
E7 positions are available via VTEL sources 240–243. VTEL also supports
DP diagnostic sources 244–261 (M3.5 adds 257–259; M3.8 adds 260–261).
`DP_CALC` and `HDG_CALC` are 0/1 validity of the computed controller demands,
not propulsion status. They are also sent over MAVLink at `DP_TEL_HZ`. VTEL receiver polling is independent from
MAVLink `DP_TEL_HZ` because the transport is receiver-driven.

`NAV_STATE` / `NAV_RBITS` remain a **separate unified status feed**:
MAVLink at 2 Hz plus bounded notifications when status changes, VTEL
sources 140/141, and `VNST` logging. This preserves the existing state
cadence and reason definitions rather than duplicating them.

## Safety / next steps

M3.8 remains observation-only: no `VesselThrustAllocator` output is routed to
hardware and no Heading Hold or DP can claim real command authority.
DP uses the AHRS/EKF3 body-origin `get_location()` and `get_velocity_NED()`
from the same control cycle. The code does not transform raw antenna GPS
coordinates or add a second roll/pitch/lever-arm correction. Configure ArduPilot
sensor offsets to represent the intended hull DP control point, and **verify**
that EKF position and velocity agree at that point with a rocking vessel before
using physical outputs. Body origin is not automatically the vessel's geometric
center or center of gravity. M3.5 does not claim that hardware verification.
`DP_VN`/`DP_VE` are the DP controller's possibly filtered EKF velocities;
raw AHRS N/E velocity remains available as VTEL sources 13/14.

VTEL samples configured values at its standard 10 Hz snapshot cadence, subject
to receiver polling. `DP_TEL_HZ=0..50` governs the **MAVLink diagnostic stream**,
not receiver requests; zero does not invalidate VTEL source assignments.

Next: physically verify estimator point/antenna offsets; define Heading Hold
translation inputs, independent VSP
pair and bow-thruster ownership, neutral handovers and failure behavior.
Then test SITL, full firmware build and hardware with actuators disconnected
before considering controlled vessel trials.
