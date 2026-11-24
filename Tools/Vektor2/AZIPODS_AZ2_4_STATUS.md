# AZ2.4 — Azipod diagnostic status: firmware and client-agent contract

Baseline: Vektor2_AZ2.3. Read this file together with
[AZIPODS_AZ2_3_CLIENT.md](AZIPODS_AZ2_3_CLIENT.md). This milestone adds
observability, not new actuator restrictions or modes. The supported layout
is AZI AFT (AZI1 port, AZI2 starboard). VSP and independent tunnel thruster
behavior, navigation, allocator, routing IDs and calibration protocol are unchanged.

## State and reason bits

The existing state enum is unchanged: 0 Blocked, 1 Homing, 2 WaitNeutral,
3 Ready, 4 Calibration. Each installed drive also reports a uint16_t reason
mask. Bits may coexist. State describes the control stage; reasons describe
currently observed limitations. Decode by bitwise AND, not equality.

| Bit | Decimal | Symbol | Client text |
|---:|---:|---|---|
| 0 | 1 | CalibrationInvalid | Invalid calibration |
| 1 | 2 | OutputRouteMissing | Output route missing/unavailable |
| 2 | 4 | OutputRejected | Output publication rejected |
| 3 | 8 | RcInvalid | RC signal invalid |
| 4 | 16 | InputInvalid | Thrust input missing/invalid |
| 5 | 32 | StartupCentering | Startup centering |
| 6 | 64 | NeutralRequired | Waiting for neutral |
| 7 | 128 | AlignmentPending | Steering alignment pending |
| 8 | 256 | ReversalDelay | Motor reversal stop/dwell |
| 9 | 512 | DirectionUnreachable | Requested direction unreachable |
| 10 | 1024 | DirectionProjected | Projected thrust solution selected |

Reasons are reset on every controller execution and augmented by output
publication failures in the same cycle. RC and input validity and route
readiness are sampled even while homing. Stage-specific flags are emitted
only when their stage is evaluated (e.g. reversal dwell is evaluated after
alignment). A selected projection sets both bits 9 and 10, even before the
motor can deliver that solution. Neutral demand clears demand-related flags.
Calibration bypasses normal RC, input, alignment and reversal gates, so those
normal-control flags are not reported in state 4. Both pods enter Calibration
state, but only the selected pod receives the active test demand. Missing
routes, invalid calibration and rejected outputs still report reasons.

A zero mask means no currently reported limitation, not motor running,
physical movement confirmed or a global arming signal. Reasons are live
rather than latched faults; a transient event may be missed by periodic
telemetry. An installed but not yet executed controller may report Blocked
with mask zero. Uninstalled drives are unavailable rather than healthy zero.
Alignment is based on the open-loop commanded-angle estimate, not feedback.

## VTEL catalog

New source 286 `AZI1_RB`, 296 `AZI2_RB`, numeric mask, generic i-BUS type
0x02, 2 bytes, scale 1, bias 0. State and reasons remain readable when the
drive is Blocked, while five command fields are invalid. No slot/default
changes. HoTT destination-specific scaling/clipping still applies. See VTEL.md.

## MAVLink stream (implemented)

`AZI_TEL_HZ` (AP_Int8, Logic group index 15): default 5, range 0..10 Hz,
zero disables, values above 10 capped at runtime. Live setting; no reboot
needed. Existing VSP_TEL_HZ semantics unchanged. Both propulsion families
reuse the existing rate limiter; never emit AZI messages on VSP, no-backend
or unspecified AZI layout boots. Seven `NAMED_VALUE_FLOAT` fields per installed
pod at the configured rate, best-effort with the existing GCS transport.
All names fit the MAVLink 10-character field. Do not subscribe to guessed
long names or assume VTEL IDs are MAVLink message IDs.

| VTEL ID (1 / 2) | VTEL suffix | Exact MAVLink name (1 / 2) | Unit |
|---|---|---|---|
| 280 / 290 | AZ_PWM | AZI1_AZPWM / AZI2_AZPWM | logical us |
| 281 / 291 | MOT_PWM | AZI1_MTPWM / AZI2_MTPWM | logical us |
| 282 / 292 | ANG_CMD | AZI1_ACMD / AZI2_ACMD | mechanical degrees |
| 283 / 293 | ANG_TGT | AZI1_ATGT / AZI2_ATGT | mechanical degrees |
| 284 / 294 | MOT_CMD | AZI1_MCMD / AZI2_MCMD | signed normalized command |
| 285 / 295 | STATE | AZI1_STATE / AZI2_STATE | enum 0..4 |
| 286 / 296 | RB | AZI1_RB / AZI2_RB | integer bitmask represented exactly as float |

MAVLink values are unscaled (angles in degrees, motor in -1..1). Blocked or
otherwise invalid command fields send NaN, explicitly clearing the old value;
state and reason bits stay available. The stream is not an atomic snapshot:
individual fields can be skipped if link buffers fill. Do not wait for all
fields to share a timestamp. Use per-field freshness; missing/stale/NaN is
unavailable, never zero. A client should not infer active propulsion solely
from pending parameter values. Clear AZI display when reconnecting/rebooting,
when switching vessels, or if the stream goes stale. Keep the calibration
command ACK/session handling independent of periodic telemetry.

## Client work (instructions for the client agent)

1. Extend the existing named-float dispatcher with the exact 14 keys above.
   Do not introduce a second MAVLink connection or polling loop. Decode only
   finite integral state/mask samples in their respective ranges; show unknown
   bits as unknown rather than silently dropping them.
2. Add a compact state line and reason chips/text to each azipod visual panel
   and calibration widget. Show all set bits, not a single priority reason.
   Distinguish normal transitions (homing, neutral, alignment, reversal) from
   route/calibration/input faults and projected-direction limitations.
3. Display logical steering/motor PWM and commanded/target mechanical angle.
   For predicted body direction add the installed offset (wrapped for display).
   Negative motor command reverses the nominal thrust arrow by 180 degrees.
   Do not animate measured steering or propeller rotation from the command.
   Keep actual observed CW/CCW separate and use SERVO_OUTPUT_RAW for physical
   channel pulse confirmation. If Blocked or stale, suppress the command arrow.
4. Suggested stale timeout: max(1 s, 3 / AZI_TEL_HZ seconds) plus reasonable
   link margin; if rate is zero show telemetry disabled. Do not disable the
   calibration controls simply because an optional diagnostic stream is missing.
5. Tests: decode 0, individual bits, 1536 (unreachable + projection), simultaneous
   RC/input/neutral reasons, unknown bits, NaN clearing, stale/reconnect, both
   pods independent, and calibration state with no RC. Verify exact aliases
   including 10-character AZI1_STATE/AZI1_AZPWM/AZI1_MTPWM and AZI2 equivalents.
   Status 3 alone is not evidence of thrust; show Ready + alignment/reversal
   reasons correctly. Preserve existing client calibration protocol and controls.

## Firmware implementation and verification

- `AzipodControl.*`: reason bits only; existing control decisions, output
  values and state transitions retained. `Logic.cpp` supplies route readiness
  during calibration for reporting only.
- `VTelemetryPropulsion.h`, `VTelemetryCore.cpp`, `VTelemetry.cpp`: shared
  reader and two new IDs; same validity rules for VTEL and MAVLink.
- `Telemetry.*`, `Vektor2.cpp`: periodic AZI reporting, shared rate limiter;
  `Logic.*`: one new telemetry rate parameter, all existing keys unchanged.
- Host controller tests: all 11 bits, simultaneous reasons, clearing, calibration,
  and previous 57,600 solver cases / 8 controller combinations pass.
- Host integration: both profiles pass, including independent status, route
  rejection and unavailable sources. Host sizes unchanged: VSP 288 B, AZI
  124 B, shared 292 B, Logic 976 B. These are not embedded measurements.
- VTEL codec/catalog tests pass, including unscaled multi-bit wire value.
- All 14 runnable host scripts pass (including both integration and sequencer
  profiles). The 15th, test_measure_points.py, cannot start without the
  ArduPilot MatekH743 hwdef.dat file. This is an unavailable dependency,
  not a tested pass. The new SITL script was syntax-checked only.
- Target build, SITL and physical receiver/actuator commissioning remain
  unverified; ArduPilot dependencies and target binary not in archive. Run
  `tests/test_mavlink_azipod_status.py` with the built SITL binary.
- Client code not included; agent implementation and end-to-end validation
  pending. No additional control feature should be built into this milestone.
