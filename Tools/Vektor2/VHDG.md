# Vector-based heading hold

VSP heading uses one PID and the shared vector mapper. The legacy rod-Y heading
path is removed. Manual rod mode remains available, but automatic VSP heading
requires **VSP1_VMODE=1 and VSP2_VMODE=1**. Mixed or legacy modes inhibit VSP
heading with configuration status; the firmware does not silently change modes.

## Control flow

1. Snapshot both drives' manual inputs, neutral interlocks and mapping settings.
2. Resolve actuator readiness/manual override and the common slew bound.
3. Update the heading capture/state machine and PID exactly once.
4. If holding, select opposed body-frame vectors; otherwise select manual input.
5. Apply each selected vector once through VspMapper, then publish final rods.

Positive heading command turns toward starboard:

```
VSP1 (port):      forward = +command, starboard = 0
VSP2 (starboard): forward = -command, starboard = 0
```

Equivalently, positive effort requests direction 0 degrees on VSP1 and 180 on
VSP2. Negative effort reverses them. Each drive's DIR (0 CW, 1 CCW), MAP_ANG,
X_C/Y_C and LIM translate the body vector into its own physical rods. THR_ANG is
not applied again. The mapping works for the side-by-side pair in both AFT and
VWT layouts; this is not full surge/sway allocation or a calibrated torque model.

HeadingHold contains no rod geometry or PWM corrections. Its actuation snapshot
provides availability, manual override, validity, revision and normalized limits.
HeadingHoldBackend owns sensor sampling. Logic owns pair selection. VspVectorControl
prepares manual readiness and applies the selected command through the same mapper
used by manual vector control. No second PID, artificial RC pulses or second
manual slew stage is used for heading.

## Engagement and release

Set VHDG_ENABLE=1 and VHDG_VSP_EN=1. Existing VFN heading requests still gate
activation. Both vector drives must be ready, with valid X/Y routes, and all four
inputs must remain inside both VHDG_STK_DB and their drive's VDB around 1500 us.
After VHDG_IDLE_MS continuously neutral time, capture the current heading. Target
capture occurs at the end of the wait; automatic effort starts on the next cycle.

Any pilot input outside either neutral interval releases both automatic commands
immediately. Manual vector slew starts from zero, not from the old heading vector.
Returning to neutral restarts capture. Route/mapper/mode/control-configuration
changes, stale input, control gaps above 100 ms, EKF yaw resets and invalid sensors
also clear the target and integral. No stale automatic contribution remains.

RPM routing is independent and is not required for heading readiness. Firmware
does not measure rotor speed: a valid rod command does not prove actual thrust.
Heading still requires healthy enabled compass yaw, initialized AHRS attitude,
fresh INS data and valid control input. GPS is not required. Component cross-feed
involving selected actuators inhibits heading to avoid feedback loops.

## PID, limits and slew

Heading error wraps to [-180,180). P and I use error outside HDG_DB; filtered
world-vertical yaw-rate damping remains active inside the band. MAX bounds signed
normalized effort and IMAX bounds the integral contribution.

For VSP heading the common slew is the minimum of VHDG_SLEW, VSP1_VSLEW and
VSP2_VSLEW, plus any participating thruster slew constraint. Both opposed vectors
use that one constrained effort. The PID's conditional integration uses this
constrained normalized command, including saturation and slew, to prevent windup.
Constraint calculation is side-effect free; anti-windup may evaluate it twice,
but output state is committed once. PWM quantization is applied only by the mapper;
it is not fed back as a hard zero that could prevent I-only control from starting.
TDIR/TMAG report quantized vector estimates, not measured force.

## Bow/stern and VRS

Independent bow/stern manual controls and heading participation remain. Their
existing HDG_EN/SIGN/MAX, enable/function gates and neutral interlocks apply.
Selected thrusters share the same constrained yaw command. Manual operation of an
enabled thruster releases heading. VHDG_VSP_EN=0 selects thruster-only heading
without VSP input/mode requirements. This is fixed yaw mixing, not force allocation.

VRS remains available for legacy manual rods. It is inhibited while either VSP
is vector-owned; it is not added behind the vector heading mapper. The old
heading-protected rod-Y VRS overlay has been removed.

## Parameters and migration

Existing parameter storage IDs remain unchanged. VHDG_Y1_INV and VHDG_Y2_INV
are removed; IDs 12/13 are reserved. Use each drive's DIR and MAP_ANG for physical
mapping. No saved inversion is silently translated into a mounting angle.

| Parameter | Default | Meaning |
| --- | --- | --- |
| VHDG_ENABLE | 0 | Master heading enable |
| VHDG_VSP_EN | 1 | Include the vector VSP pair; 0 is thruster-only |
| VHDG_STK_DB | 30 | Additional heading-neutral half-width, us |
| VHDG_IDLE_MS | 1500 | Neutral wait before capture |
| VHDG_P / I / D | 0 | Shared heading gains |
| VHDG_IMAX | 0.1 | Integral contribution limit |
| VHDG_HDG_DB | 1 | Heading error deadband, degrees |
| VHDG_MAX | 0.2 | Maximum normalized yaw effort |
| VHDG_SLEW | 0.5 | Maximum effort change per second |
| VHDG_FILT_HZ | 2 | Yaw-rate filter cutoff |
| VHDG_TEL_HZ | 5 | Measurement telemetry rate |

The new opposed longitudinal thrust has different yaw effectiveness from the old
rod-Y addition. Verify physical direction and retune gains for this actuation.
Legacy saved gains are retained, but legacy mode will not engage heading until
both vector modes are explicitly enabled and their neutral checks pass.

## Telemetry and logging

HDG_NOW/TGT/ERR/RATE/REQ/I/WAIT/SCALE remain. **HDG_CMD** is the signed constrained
normalized yaw command replacing V1_HDG_US/V2_HDG_US; those old rod-Y keys are no
longer sent. The VHDG binary log likewise replaces Y1/Y2 with Cmd.

VSPn_XA/YA and TDIR/TMAG describe final mapped outputs. VSPn_AUTH=3 means manual
baseline plus captured heading authority, including zero-error hold; releasing
heading returns to manual or neutral guard. VSPn_VST reports actuator readiness.

State numbers remain 0 disabled, 1 inhibited, 2 manual override, 3 waiting,
4 holding. HDG_INHIB retains bit positions; configuration now covers unsupported
VSP modes/mapping instead of rod-Y geometry. Status/reason messages continue at
1 Hz below TEL_HZ=10 and 2 Hz at 10 or above, even when measurement rate is zero.

## Validation

```
ASAN_OPTIONS=detect_leaks=0 python3 Tools/Vektor2/tests/test_heading_hold.py
ASAN_OPTIONS=detect_leaks=0 python3 Tools/Vektor2/tests/test_vsp_vector.py
ASAN_OPTIONS=detect_leaks=0 python3 Tools/Vektor2/tests/test_thrusters.py
./waf configure --board sitl
./waf --targets bin/Vektor2
python3 Tools/Vektor2/tests/test_mavlink_vhdg.py
python3 Tools/Vektor2/tests/test_mavlink_vsp_vector.py
python3 Tools/Vektor2/tests/test_mavlink_thrusters.py
```

SITL yaw disturbances come from separate rover propulsion channels: tests verify
sensor feedback, mapped outputs and transitions, not VSP hydrodynamic performance.
