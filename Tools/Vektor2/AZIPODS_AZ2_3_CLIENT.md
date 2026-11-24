# AZ2.3 — Azipod visual calibration widget: client-agent contract

This document is the implementation specification for the ground-station/client
agent. Implement the widget in the existing client UI and MAVLink transport;
there is no client source in the firmware archive. Do not invent a second
parameter schema, implement a second controller, or change the VSP calibration
UI. See `AZIPODS_AZ2_HANDOVER.md` for AZ2.2 and earlier control architecture.

## 1. Scope and discovery

- `MAIN_PROP_TYPE=1`, `AZI_CONF=1` (boot-selected AZI AFT) installs AZI1 port
  and AZI2 starboard. No per-drive enable flags. Show the azipod calibration
  widget only for the active AZI AFT configuration. Propulsion/layout and all
  `AZI<n>_*` calibration parameters are reboot-required. A pending parameter
  value is not proof of the live layout; track the reboot/configuration status.
- Find the steering and motor physical output channel numbers from the active
  `RTn_SRC`/`RTn_DST` routes (n=1..32). AZI1 steering source=141, motor=142;
  AZI2 steering=151, motor=152. Destination=PWM channel number (1-based).
  Exactly one local PWM output route for each port of the selected pod is
  required. No hardcoded physical channels. Show missing/duplicate routes and
  disable Start. Do not silently create/replace routes. Input RC routes are not
  required for calibration.
- Read the existing `PWM<channel>_MIN`, `PWM<channel>_MAX`, `PWM<channel>_INV`
  for each of these two channels and `AZI<n>_ANG_MIN`, `ANG_MAX`, `OFFSET`,
  `REV`, `RATE`, `REV_DLY`. Other AZ2 control parameters are unchanged.
- Client needs MAVLink2 to read the detailed command ACK. Use the existing
  vehicle sysid/compid discovered from heartbeat, and a nonzero client
  sysid/compid. Do not broadcast calibration commands.

## 2. Widget layout and interaction

A compact, per-drive calibration panel (not a vessel manoeuvring panel).

1. Header: AZI1 Port / AZI2 Starboard tabs, assigned steering and motor PWM
   channels, connection and session status. Start calibration / End
   calibration. Before changing tabs, send zero motor and End, then open a
   new session for the other pod. One selected pod at a time.
2. Main visual: top-down circular azimuth scale, bow upward (0°), starboard
   +90°, astern ±180°, port -90°. Shade accessible arc using `ANG_MIN/MAX`
   plus `OFFSET`. Draw the pod and an arrow for its predicted forward thrust
   orientation. Draw a separate effective thrust arrow when motor demand is
   nonzero, flipped by 180° for negative demand. Show an explicit label
   "Commanded / predicted, not measured". Do not imply measured angle, RPM,
   thrust or propeller spin direction. Optionally add a small top-down vessel
   preview highlighting port/starboard pod. No live camera/encoder required.
3. Steering: logical PWM slider 1000..2000 µs, step 1 or 5; Min / Center /
   Max buttons (1000 / 1500 / 2000). For first endpoint discovery begin at
   center, use small steps. Numeric readout of predicted mechanical angle,
   predicted body angle, logical PWM and calibrated physical PWM. Use a
   separate draft calibration editor with 180° and 270° presets, custom
   asymmetric angle endpoints, offset, physical PWM min/max and invert.
   Presets change only the draft ANG_MIN/MAX (±90, ±135), not servo output.
   User visually checks actual installed travel and adjusts draft values.
4. Motor: forward-only/reversible capability selector (parameter editor,
   requires reboot); live test slider 0..20% (forward only) or -20..+20%
   (reversible), integer percent. Reverse / Stop / Forward buttons command
   -10 / 0 / +10%; reverse disabled for forward-only. Begin at zero; user
   may use smaller nonzero values to find the lowest visible rotation.
   Display logical and physical motor PWM separately. Stop stays available
   throughout the session. On reversal, first command zero and wait at
   least the configured REV_DLY before sending the opposite sign. This is
   a test control, not the normal vector solver.
5. Visual observation: show "Observed propeller rotation, looking from the
   propeller toward the motor: CW / CCW / not checked" for the positive
   test direction, and for negative if supported. Also ask whether expected
   positive thrust direction is correct. CW/CCW is an operator observation,
   not a firmware setting; keep it in client commissioning notes if the
   client supports notes. Never infer CW/CCW from motor command sign or
   confuse handedness with thrust direction. A forward-only ESC must not
   be made reversible by inverting PWM; correct actual rotation using
   the ESC's direction setting or motor wiring with power off. Invert PWM
   only if that is the verified ESC pulse convention.
6. Apply/Save draft using the existing parameter editor and PARAM_SET/PARAM_VALUE
   confirmations. Stop the motor before editing calibration; physically
   isolate motor power when changing ESC type or its endpoint/neutral mapping.
   PWM mapping changes are live; AZI calibration changes take effect after
   reboot. End the session before reboot, then read back parameters and
   begin a new session. Never re-enter a motor test automatically.

Keep the interface simple. No wizard infrastructure, no automatic angle
identification, no continuous 360° controls, no new persistent configuration.

## 3. MAVLink calibration API (implemented firmware)

Use `MAV_CMD_USER_3` (31012 in common.xml). `USER_1` and `USER_2` remain the
sequencer commands. Both COMMAND_LONG and COMMAND_INT are accepted. For INT,
frame=MAV_FRAME_MISSION, current=0, autocontinue=0; map p5->x, p6->y, p7->z.
For LONG, confirmation may be 0. All fields are finite integers.

| Field | Value |
|---|---|
| param1 | Operation: 1 BEGIN, 2 UPDATE, 3 END |
| param2 | Pod number: 1 AZI1, 2 AZI2 (not component IDs 4/5) |
| param3 | Logical servo PWM, 1000..2000 µs |
| param4 | Signed motor percent, -20..+20; must be zero for BEGIN/END and nonnegative for forward-only |
| param5 | Client-generated session token, 1..65535, fresh for each BEGIN |
| param6 | Reserved, 0 |
| param7 | Reserved, 0 |

Examples (p1..p7; exact vehicle target required):

```
BEGIN AZI1 centered:       [1, 1, 1500,   0, 42, 0, 0]
Move steering only:        [2, 1, 1750,   0, 42, 0, 0]
Run positive motor 10%:    [2, 1, 1750,  10, 42, 0, 0]
Stop, retain angle:        [2, 1, 1750,   0, 42, 0, 0]
END:                       [3, 1, 1750,   0, 42, 0, 0]
BEGIN AZI2, new token:     [1, 2, 1500,   0, 43, 0, 0]
Reverse 10% if reversible: [2, 2, 1500, -10, 43, 0, 0]
```

### ACK and lifetime

`COMMAND_ACK.command=MAV_CMD_USER_3`, addressed to client source sysid/compid;
`progress=0`. `result_param2` detail codes:

| Detail | Meaning | MAV_RESULT |
|---:|---|---|
| 0 | Accepted | ACCEPTED (0) |
| 1 | Invalid field or unsupported negative/direction-change test | DENIED (2) |
| 2 | Busy: other sender, link, pod or token owns session | TEMPORARILY_REJECTED (1) |
| 3 | Not available: inactive pod, invalid boot calibration, missing/duplicate/unavailable output | FAILED (4) |
| 4 | No active session for UPDATE | DENIED (2) |

Accepted means the requested command was stored for the next normal 100 Hz
routing cycle. It does not confirm actual physical movement. A session is
owned by source sysid, source compid, link, token and selected pod. Do not
use multiple concurrent command streams. BEGIN must have zero motor and may
be repeated by the same owner/token, resetting the demand to zero. END is
idempotent if no session exists. Wrong vehicle target is ignored.

Send UPDATE at **5 Hz** while the panel is active, even if sliders are
unchanged. One outstanding command at a time; coalesce slider changes into
the next update. Do not flood the link. The lease expires after 1000 ms with
no accepted command. Stop sending nonzero demand if an ACK is missing or
negative; try zero/END if connected, mark the widget inactive, require
explicit new BEGIN. Never blindly retransmit an old nonzero command or
restart a timed-out session. Route changes and selected-output rejection also
cancel calibration; UPDATE returns NoSession on the next request. To close,
switch pods, disconnect, or leave the page, send zero UPDATE then END when
possible; stop refresh regardless. No durable calibration mode exists.

In calibration both installed AZI controllers are suspended. The other pod
holds its last commanded steering angle and outputs motor stop. Selected pod
uses the direct PWM/percent commands. RC input is not required. Tunnel
thrusters and unrelated outputs remain under their existing control; this is
not global arming/disarming. Exit/timeout restores normal homing and neutral
interlocks. Firmware rejects a direct nonzero sign change; client sends zero
and waits for the configured reversal dwell. Test with the propeller area
clear, at low speed, and ensure ESC stop/no-signal behaviour is correct.

## 4. Display math (not a second control algorithm)

Let `u` be steering logical PWM, `aMin/aMax` mechanical endpoints, `offset`
installation offset, `m` signed integer motor percent. Body angle is clockwise
from bow. Physical output parameters are independent for steering and motor.

```
mechanicalDeg = aMin + (aMax-aMin)*(u-1000)/1000
bodyDeg = wrap180(mechanicalDeg + offset)
nominalEffectiveDirection = wrap180(bodyDeg + (m < 0 ? 180 : 0))
logicalMotor = reversible ? 1500 + 5*m : 1000 + 10*m
scaled = round((logicalPWM-1000)*(PWM_MAX-PWM_MIN)/1000)
physicalPWM = PWM_INV ? PWM_MAX-scaled : PWM_MIN+scaled
```

Mechanical coordinates are unwrapped, not headings; travel must be positive
and less than 360°. Offset and endpoints are signed. `PWM_INV` affects only
physical mapping, not the predicted body angle after correct calibration.
The physical motor stop is calibrated MIN (or MAX when inverted) for
forward-only, midpoint for reversible. `REV=1` does not imply equal physical
forward/reverse thrust. Use the live, boot-applied REV for the test commands,
not a draft value. Label draft angle visualization as a preview until saved
and rebooted. No midpoint trim parameter exists; the installed servo must
be acceptably linear across the calibrated range. Use a printed angle scale
and visually verify several intermediate points.

## 5. Telemetry

Use standard MAVLink `SERVO_OUTPUT_RAW` (message 36, request about 10–20 Hz
with SET_MESSAGE_INTERVAL if not already streamed). It reports the actual
commanded calibrated pulses for physical channels 1..16; zero means disabled.
For channels above 16, do not invent live confirmation; show calculated PWM
and an unavailable measured-output indicator. Even a reported PWM is not
angle, thrust or spin feedback. Do not add a new MAVLink dialect.

VTEL catalog source IDs 270–272 describe the active family and layout; IDs
280–286 and 290–296 describe pod logical outputs/angle/motor/state/reason bits. New state
4 = Calibration (both installed controllers suspended during a session).
Existing states 0 blocked, 1 homing, 2 waiting neutral, 3 ready are unchanged.
VTEL is receiver telemetry (i-BUS/HoTT). As of AZ2.4 the firmware also
streams the seven per-drive fields as MAVLink `NAMED_VALUE_FLOAT` when
`AZI_TEL_HZ` is nonzero. See [AZIPODS_AZ2_4_STATUS.md](AZIPODS_AZ2_4_STATUS.md)
for exact wire aliases, reason-bit decoding, validity and UI instructions.
Use the telemetry for live diagnostics; ACK and SERVO_OUTPUT_RAW still
provide command acceptance and physical-channel PWM, respectively. Do not
infer physical position, propeller rotation or thrust from these messages.

## 6. Client acceptance checklist

- AZI AFT only; both tabs work, each resolves its own route/channel params.
- 180°, 270° and custom asymmetric arc render correctly with offsets/inversion.
- Min/center/max, slider, signed thrust arrow, stop and forward/reverse controls
  produce the documented command values, including correct stop at 0%.
- Physical PWM displays match calibrated output; actual observed CW/CCW can
  be recorded without claiming sensor feedback.
- No motor commands before accepted BEGIN. Zero on entry, exit and tab switch;
  negative command disabled for forward-only; reversal passes through zero.
- Refresh at 5 Hz; ACK failure/timeout/disconnect closes the live session;
  no auto-resume. New token per explicit start.
- Existing parameter writes/readback and reboot-required UI work. New widget
  does not change other vessel panels, routing or normal propulsion behaviour.
