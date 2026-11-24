# AZ2.3 firmware handover — direct visual calibration

Baseline: `Vektor2_AZ2.2.zip`. Scope: direct temporary azipod servo and
propeller test, plus a client implementation contract. No changes to VSP,
tunnel thrusters, layout selection, navigation, allocator, normal AZ2 control
algorithm, route IDs or persistent parameter schema.

Read [AZIPODS_AZ2_3_CLIENT.md](AZIPODS_AZ2_3_CLIENT.md) for the exact wire
protocol, client widget and visual calibration workflow. It is the authoritative
client-agent handover; the client code is not included here.

## Implementation

- `GCS_MAVLink.*`: dispatch `MAV_CMD_USER_3` LONG/INT and reply with COMMAND_ACK
  plus detail code. Uses existing MAVLink command transport; USER_1/2 unchanged.
- `Routing.*`: small volatile one-owner session, lease=1000 ms, selected pod,
  1000–2000 logical steering command, signed integer motor test ±20%. Requires
  valid AZI AFT boot selection, calibration, paired local PWM routes and
  compatible ESC type. Route revision/timeout/selected output failure cancel
  the session. No RC input required. Normal AZI execution is replaced only
  during the session. Other component execution remains unchanged.
- `Logic.*`, `AzipodControl.*`: direct commissioning path through existing
  runtime slots and PWM routing; normal controller invalidated so exit invokes
  the usual startup centering/neutral guard. Nonselected pod holds its last
  commanded angle at motor stop. Direct tests bypass slew, solver, alignment
  and optimization; client performs zero/dwell before reversal. State 4 =
  Calibration; existing VTEL source IDs and slot counts unchanged.
- `PwmOut` is unchanged. Existing channel endpoint/inversion params do final
  mapping. No new persistent calibration params, no new HAL driver or CAN
  support. The user must visually verify servo and propeller movement.

## Verification

- `python3 tests/test_main_propulsion.py`: both compact and large profiles pass
  (real Logic/Routing with mock HAL, ASan/UBSan); calibration tests cover
  forward-only and reversible ESCs, isolated pod selection, no RC, bounds,
  ownership/token conflicts, direction change through zero, exit/timeout,
  selected steering write failure, route loss, and inactive family rejection.
- `python3 tests/test_azipod.py`: 57,600 solver combinations and 8 controller
  configurations passed unchanged.
- `python3 tests/test_vtelemetry.py`: catalog/protocol tests passed.
- Host sizes: VSP backend 288 B, AZI backend 124 B, shared backend 292 B,
  Logic 976 B (unchanged); Routing adds one small session record.
- `tests/test_mavlink_azipod_calibration.py`: target/SITL acceptance script
  for USER_3 COMMAND_LONG and COMMAND_INT, real parameter routes, PWM, reverse,
  stop and timeout. Syntax checked only. ArduPilot dependencies, target binary
  and pymavlink are not included; target compile, wire integration and
  hardware commissioning remain unverified. Run the script with --binary
  pointing to the built Vektor2 SITL binary.

## Commissioning limitations

No measured azimuth, propeller spin, RPM or thrust from this calibration
interface. A servo may jam; an ESC may ignore pulses. Other propulsion and
auxiliary outputs remain live under their original control. No global arming
was added. Calibration is deliberately a short direct low-power test, not a
new vessel mode. Physically verify ESC stop and no-signal stop before using
motor test. Do not alter route assignments during the test. End calibration
before reboot and re-enter explicitly. PWM endpoint edits are live; AZI
settings are boot-latched. Use the client contract's distinction between
command and observation. Continue with bench validation, then AZ3 coordinated
AFT manoeuvring. Do not introduce speculative layouts or 360° steering.
