# Paired VSP heading hold

`VHDG_ENABLE=1` enables a shared compass-heading controller. The default actuator
selection uses both VSPs; independent bow/stern participation is described below.
The compiled default is disabled, with P/I/D gains zero. Disabled heading hold
retains the original component execution order and X/Y/RPM outputs, does no
heading sensor reads and emits no heading telemetry. The reserved
`VNAV_HDG_*` parameters do not configure this controller.

When a [VFN function](VFN.md) selects Auto heading, its aggregated runtime
request also gates VHDG_ENABLE. Releasing it discards the heading target;
reactivation starts the ordinary neutral wait and captures a fresh target.

## Manual override and target capture

All four **routed component rod inputs** (VSP1 X/Y and VSP2 X/Y) must be in
the inclusive interval `1500 - VHDG_STK_DB .. 1500 + VHDG_STK_DB`. This is
independent of each actuator's `VSPn_X_C` / `VSPn_Y_C` offsets. RPM must be a
valid routed pulse but is excluded from the stick deadzone.

Moving any one rod axis outside this interval removes **both** heading
corrections in the same routing cycle, clears integral/slew state, and discards
the target. Manual commands continue through the existing geometry mapping.
Returning all axes to neutral starts `VHDG_IDLE_MS`; movement restarts the wait.
At the **end** of this uninterrupted interval, capture the current EKF yaw as
the new target. Correction starts from zero on the following cycle. The target
is runtime state, not a saved parameter, and is discarded on reboot/disable.

Heading hold requires a detected, enabled, healthy compass used by AHRS for
yaw, a compass sample at most 300 ms old, valid initialized EKF attitude,
healthy INS updated within 100 ms, fresh RC/override input and all six valid
routed VSP input pulses (1000..2000 us). Compass calibration or a pending
calibration reboot inhibits it. GPS position is not required; compass-less GSF
yaw does not qualify. Missing upstream input validity propagates through the
router, so a component cannot synthesize valid control from a missing source.

Sensor/input loss, an EKF yaw reset, invalid configuration or a loop gap over
100 ms immediately removes correction. Recovery starts a new neutral wait and
captures a fresh target. Live heading settings, ring radius/center changes and
route edits also restart acquisition. There is no residual ramp-down on
manual override, disable or inhibition.

The controller removes its automatic addition on control loss; the existing
manual geometry and routing loss policy still govern the baseline outputs.
Direct RC-to-PWM routes retain their existing HoldLast behavior.

## Controller and output composition

One sensor/input snapshot drives one paired decision each cycle:

```text
error = wrap_to_minus180_plus180(target_heading - EKF_heading)
e = sign(error) * max(0, abs(error) - HDG_DB)
rate = low_pass(earth_frame_vertical_gyro_rate, FILT_HZ)
integral += I * e * dt, constrained to +/-IMAX with anti-windup
request = P * e + integral - D * rate
demand = constrain(request, -MAX, MAX)
```

The demand is a signed fraction of each ring radius. Heading deadband affects
P/I; rate damping still acts at the target. The existing integral is retained
inside the deadband to counter a sustained disturbance. Normal command changes
obey SLEW, including reversals. Anti-windup limits integration against MAX,
slew and the common available physical travel while allowing unwinding.

Apply heading correction **after** the original `mapCoordinates()` angle/ring
mapping, along the actuator's Y axis:

```text
VSP1_Y = mapped_Y1 + (Y1_INV ? -1 : +1) * command * VSP1_LIM
VSP2_Y = mapped_Y2 + (Y2_INV ? -1 : +1) * command * VSP2_LIM
```

Inversion affects only the automatic heading addition. It does not change
manual Y, `THR_ANG`, routing or `PWMn_INV`. Establish the physical yaw effect of
each positive actuator Y movement and set the two inversions accordingly.
Any downstream routing/scaling/reversal remains part of that physical mapping.

Preserve X and constrain only the added Y to the ring and 1000..2000 us
component endpoints. Both corrections share the smaller normalized headroom;
if one drive has no travel, both yield. Reduced travel takes effect immediately.
Integer corrections truncate toward zero (less than one microsecond error per
drive); the geometric limit is computed for integer Y coordinates. Invalid
baseline geometry inhibits heading hold without repairing the original output.
RPM remains a passthrough: its input is a command, not measured shaft speed.

VRS runs after heading allocation with the remaining travel. A VRS ray whose
Y component opposes an applied heading correction yields for that cycle,
reporting VRS inhibition bit 1024. Rays that fit and do not oppose heading
remain available. When heading is disabled/manual/waiting, VRS retains its
existing behavior.

The router collects both VSP inputs before running the pair. Independent
fan-out is supported. Component links involving thrusters inhibit heading;
manual routing remains available. A direct **or
indirect** path from one VSP into the other inhibits heading hold: the original
router still executes that valid chain, but automatic paired correction would
otherwise be applied twice. The disabled feature retains the original ordering.

## Parameters

Invalid control settings inhibit control rather than silently being clamped.
Telemetry rate is independent of control: changing it does not discard the
heading target, reset the controller or change actuator outputs. All values are
ordinary persistent AP_Param/MAVLink parameters; existing storage IDs are unchanged.

| Parameter | Default | Range / meaning |
| --- | --- | --- |
| `VHDG_VSP_EN` | 1 | Include paired VSPs; zero permits thruster-only heading |
| `VHDG_ENABLE` | 0 | Only 1 enables; every other value bypasses |
| `VHDG_STK_DB` | 30 | 0..250 us, neutral half-width on all four rod inputs |
| `VHDG_IDLE_MS` | 1500 | 0..10000 ms, continuous neutral wait before capture |
| `VHDG_P` | 0 | 0..10, ring radii per degree of error beyond HDG_DB |
| `VHDG_I` | 0 | 0..10, ring radii per degree-second |
| `VHDG_D` | 0 | 0..10, ring radii per degree/second of yaw rate |
| `VHDG_IMAX` | 0.10 | 0..1, maximum integral contribution in ring radii |
| `VHDG_HDG_DB` | 1 | 0..30 degrees, P/I deadband |
| `VHDG_MAX` | 0.20 | 0..1, maximum addition as a fraction of each ring radius |
| `VHDG_SLEW` | 0.5 | 0.01..100, ring radii per second |
| `VHDG_FILT_HZ` | 2 | 0.1..20 Hz, yaw-rate filter cutoff |
| `VHDG_Y1_INV` | 0 | 0/1, invert only automatic VSP1 Y |
| `VHDG_Y2_INV` | 0 | 0/1, invert only automatic VSP2 Y |
| `VHDG_TEL_HZ` | 5 | 0..50 Hz, heading measurement/correction floats; 0 stops these measurements |

Gains describe rod displacement and require a vessel tune at the intended
operating RPM. With the default `VSPn_LIM=25`, MAX=0.2 permits only 5 us per
drive. Neither the host tests nor the rover simulator establish vessel gains.

## Named float telemetry

Heading telemetry uses its own `VHDG_TEL_HZ`, independently of `VSP_TEL_HZ`.
The ten measurement/correction floats follow this rate (default 5 Hz, bounded
to 50 Hz). `VHDG_TEL_HZ=0` stops those ten floats.

Status has a separate clock: **HDG_STATE, HDG_INHIB and HDG_REASON** transmit at
**1 Hz below 10**, including zero, or **2 Hz at 10 and above**. Changing the
measurement rate does not force an extra status transmission. Status is sent
first when both groups are due, ahead of the VSP/roll telemetry burst. Status
fields retain their common sample timestamp and are retried per link between
status ticks if the transmit buffer is full. Disabled heading hold emits neither group.

The twelve floats use `NAMED_VALUE_FLOAT`. The reason text uses the existing
ArduPilot MAVLink `NAMED_VALUE_STRING` message, also used for `RC_PROTO`.
The receiver's dialect must support that message. All names fit the MAVLink
ten-character field; the reason text fits the 64-character value field.

| Wire name | Units / meaning |
| --- | --- |
| `HDG_NOW` | Current AHRS heading, 0..360 degrees; consult validity flags |
| `HDG_TGT` | Captured target heading, degrees; NaN until HOLD |
| `HDG_ERR` | Shortest signed target-minus-current error, degrees; NaN until HOLD |
| `HDG_RATE` | Filtered earth-frame yaw rate, degrees/second |
| `HDG_REQ` | Signed PID request after MAX, before slew/travel limiting, ring radii |
| `HDG_I` | Current integral contribution, ring radii |
| `HDG_STATE` | 0 disabled, 1 inhibited, 2 manual, 3 neutral wait, 4 hold |
| `HDG_INHIB` | Exact integer bitmask described below; includes limiting reasons |
| `HDG_WAIT` | Seconds remaining before capture; full wait while manual/inhibited |
| `HDG_SCALE` | 0..1 common travel scale after slew; 1 means no travel reduction |
| `V1_HDG_US` | Actual integer automatic Y addition to VSP1, microseconds |
| `V2_HDG_US` | Actual integer automatic Y addition to VSP2, microseconds |
| `HDG_REASON` | Named **string** with the primary inhibition/limiting reason, or `OK` |

`HDG_INHIB` bits may be combined; `HDG_REASON` gives one short primary reason.
Configuration/routing errors take priority over sensor/input faults, followed
by manual override, waiting and normal output limiting. The numeric mask retains
every concurrent flag.

| Value | Meaning | Short reason |
| --- | --- | --- |
| 0 | No inhibition or limiting | `OK` |
| 1 | Compass unavailable, unhealthy, stale, not used for yaw, or calibrating | `Compass invalid` |
| 2 | Invalid EKF attitude, unhealthy/stale INS, or nonfinite heading/rate | `Attitude invalid` |
| 4 | Missing/invalid RC or routed X/Y/RPM source | `Control input lost` |
| 8 | Invalid settings or baseline ring geometry | `Config/ring invalid` |
| 16 | Loop gap over 100 ms | `Loop gap >100ms` |
| 32 | Routes connect one VSP into the other | `VSP cross-feed` |
| 64 | EKF yaw reset; discard target and reacquire | `EKF yaw reset` |
| 128 | At least one rod input outside the neutral deadzone | `Manual override` |
| 256 | Waiting for uninterrupted neutral time | `Waiting for idle` |
| 512 | MAX or common available Y travel limits correction | `Travel limited` |
| 1024 | Normal slew limits correction | `Slew limited` |

Target/error NaN avoids plotting a discarded target as if it were active.
`HDG_RATE` is zero while inhibited. `HDG_SCALE` is 1 when there is no requested
travel to limit; use state/flags and the applied-microsecond values together.
Existing `VSP1_XA/YA` and `VSP2_XA/YA` show final component outputs, including
heading and any VRS addition. `SERVO_OUTPUT_RAW` shows physical PWM after
downstream scaling/reversal.

Useful plots: NOW/TGT/ERR for acquisition and tracking; ERR/RATE/REQ/I for
tuning; V1_HDG_US/V2_HDG_US/SCALE/INHIB for actuator authority; STATE/WAIT for
manual override and recapture.

## Verification

```sh
python3 Tools/Vektor2/tests/test_heading_hold.py
python3 Tools/Vektor2/tests/test_roll_stabilization.py
./waf configure --board sitl
./waf --targets bin/Vektor2
python3 Tools/Vektor2/tests/test_mavlink_vhdg.py
python3 Tools/Vektor2/tests/test_mavlink_vrs.py
```

Host tests run production controllers under address/undefined sanitizers,
including 8,640 geometry cases, all four stick axes, timed acquisition,
loss/recovery, wraparound, polarity, anti-windup and VRS priority. In an
environment using ptrace, set `ASAN_OPTIONS=detect_leaks=0` because LeakSanitizer
cannot attach there. The integration test uses an isolated temporary parameter
store, real EKF/compass data and local TCP ports 6002/6003. It exercises all
parameter names/defaults, telemetry, external yaw disturbance, overrides,
compass failure, source loss, route dependencies and live disable. It also
checks string reasons, independent telemetry at 0/5/9/10/20 Hz, the 1/2 Hz
status threshold and target retention during telemetry rate changes. If the
installed pymavlink dialect predates NAMED_VALUE_STRING, the test generates a
temporary Python dialect from this checkout's XML without changing the install.

## Independent thruster participation

`VHDG_VSP_EN` defaults to 1 for paired VSP operation. Set it to 0 for thruster-only
heading without VSP input or geometry requirements. Bow/stern independently opt in
through their HDG_EN parameters. Valid off-neutral manual input on either enabled
thruster releases heading. See [Thrusters.md](Thrusters.md) for scaling, enable gates,
route restrictions and state reporting.
