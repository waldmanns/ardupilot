# Vektor2 M3.8.1 — Single NavMode RC action (observation only)

**Baseline:** M3.8 observation-only DP. This is a small VFN switch-selection
revision. No DP/heading maths, EKF3 integration, telemetry codec or actuator
routing has changed.

## What changed

- One assignable VFN function **TYPE=7 `NavMode`**, requiring a physical
  **three-position RC switch**, selects `VNAV_MODE` (0=Free, 1=Heading,
  2=Full DP) from one RC input. A two-position switch is not a supported
  setup; check the three stable PWM levels at commissioning.
- `VFNn_LOW/HIGH` are ordered thresholds **for NavMode only**. For ordinary
  function actions the inclusive interval and toggle behavior are unchanged.
- `VFNn_MODE` **must be 0 (level)** for NavMode. With +/-50us no-change
  guard zones, three stable PWM regions must remain. LOW/HIGH are transition
  centers and settings too close together are rejected.
- In either guard zone, **no mode change occurs**: the last stable mode is
  retained, starting at Free on boot. RC loss, timing faults, bad routing,
  disabled selector or duplicate assignment immediately fails to Free.
- The switch selection is volatile: it updates `VNAV_MODE` in RAM, not flash.
  RC loss, disabled/unrouted input, malformed settings or a duplicate selector
  fails off to Free. Removing the last selector clears the previous request.
- Old navigation actions **TYPE=5 / TYPE=6 are retired**, not repurposed.
  Stored old settings produce a VFN configuration error and fail Free until
  explicitly migrated. Multiple active navigation assignments conflict.
- All existing DP/heading and VFN telemetry sources remain available,
  including `NAV_STATE`, `NAV_RBITS`, `HDG_CALC` and `DP_CALC`.
- **Still observation-only:** no automatic VSP or thruster output command.

## Board setup: one switch on RC7

For an RC switch producing about **1000 / 1500 / 2000 us**, use one otherwise
unused route and VFN slot. Verify raw RC pulse values first, and adapt
thresholds if your transmitter has different ranges.

| Parameter | Example | Meaning |
| --- | ---: | --- |
| `RT1_SRC` | `7` | RC7 |
| `RT1_DST` | `201` | VFN1 input |
| `VFN1_TYPE` | `7` | New single `NavMode` action |
| `VFN1_MODE` | `0` | Level; no toggle |
| `VFN1_LOW` | `1300` | Free/Heading boundary |
| `VFN1_HIGH` | `1700` | Heading/DP boundary |
| `VFN1_ENABLE` | `1` | Enable after setting bounds and route |

| RC pulse with LOW=1300, HIGH=1700 | Three-position mode |
| ---: | --- |
| 800..1249 us | Free |
| 1250..1349 us | No change (between switch detents) |
| 1350..1649 us | Heading Hold |
| 1650..1749 us | No change (between switch detents) |
| 1750..2200 us | Full DP |

The transition zones prevent bouncing around a threshold. Holding a mode
within a transition zone is **not** a failsafe. Lost/invalid RC always selects
Free. It is not possible to prove the presence of three physical switch
positions from an individual RC pulse, so verify all three during setup.

**Migration from M3.8:** Before enabling the new action, set every old
`VFNn_TYPE=5/6` slot to `TYPE=0`, disable the unused slots and remove only
old navigation routes that are not otherwise in use. Assign **exactly one**
`TYPE=7` slot. Do not leave a second NavMode assignment installed.

The ordinary VFN defaults `LOW=1700`, `HIGH=2000` are activation bounds for
other actions. **Do not rely on those defaults for NavMode**, since a high
switch pulse slightly below 2000 us would be interpreted as Heading rather
than DP. Set thresholds deliberately between the measured switch values.

## Calculations and verification

Keep the existing M3.8 observation-only diagnostic configuration:

| Parameter | Suggested observation-only example |
| --- | ---: |
| `VNAV_ENABLE` | `1` |
| `VNAV_HDG_ENABLE` | `1` |
| `VNAV_DP_ENABLE` | `1` |
| `VNAV_HDG_KP` | `0.02` |
| `VNAV_HDG_KD` | `0.005` |
| `VNAV_HDG_MAX_YAW` | `100` |
| `VNAV_DP_POS_KP` | `0.25` |
| `VNAV_DP_VEL_KP` | `0.6` |
| `VNAV_DP_MAX_SPD` | `0.5` |
| `VNAV_DP_ACC_LIM` | `0.2` |
| `VNAV_DP_RADIUS` | `0.5` |
| `VNAV_DP_VEL_FILT` | `0` |
| `DP_TEL_HZ` | `5` |

These values are **diagnostic examples only**, not propulsion tuning. With
valid EKF3/heading estimates, Free clears targets, Heading captures/updates
yaw calculation and DP captures/updates position and heading demands.
`DP_CALC=1` means the DP calculation is valid. `HDG_CALC=1` reports the
heading calculation. `NAV_STATE` remains `HDG_WAIT` and `NAV_RBITS` includes
`OUTPUT_NOT_READY` because no actuator ownership is implemented.

Useful diagnostic sources: `VFN1_IN` (sampled pulse), `VFN1_ST` (non-Free
request bit), `VFN1_ERR` (configuration/input faults), `VNAV_MODE` (RAM
parameter request), `NAV_STATE` / `NAV_RBITS` (authority/reasons), VTEL
`DP_CALC=260`, `HDG_CALC=261`, DP correction and heading values. For NavMode,
VFN `Matched` and `Requested` flags mean a **non-Free switch position**; they
do not distinguish Heading vs DP on their own. `VNAV_MODE` does.

## Verification limits

Host tests cover three-position switch boundaries, both no-change transition
zones (including reverse transitions), RC signal loss, invalid input, duplicate
selectors, retired M3.8
stored actions, improper toggle/threshold configuration, route failure,
control-loop timing fault, and unchanged observation-only authority semantics.

A full ArduPilot firmware build, SITL test and physical board test remain
outstanding. The installation should be verified with propulsion disconnected
before any later automatic-output revision. M4 still owns actuator
arbitration, manual override, neutral handover and safety-output policies.
