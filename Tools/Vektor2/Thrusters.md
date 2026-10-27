# Independent bow and stern thrusters

Bow and stern are independent stateful actuators. Neither requires a VSP route,
VSP geometry, RPM input, or VRS. They can run manually or consume the shared
heading controller's yaw demand. They never receive roll correction.

## Enable and manual operation

`THR_BOW_ENABLE` and `THR_STN_ENABLE` default to **0**. This replaces the previous
unconditional passthrough: existing installations must configure and enable the
thrusters explicitly. Disabled outputs send MID, not hold-last or PWM disable.
MID is a component pulse before the existing PWM output scaling/inversion.
Check that the physical output corresponding to MID stops the installed drive.

VFN TYPE=3 gates bow enable; TYPE=4 gates stern enable. Existing level/toggle
behavior applies. No assigned function means the master parameter alone controls
the gate; an assigned but disabled, invalid or unrequested function blocks it.
Multiple slots for the same action combine with OR. Function changes take effect
on the next routing cycle, as with VRS and heading functions.

Enabling, input loss, route edits, configuration edits, or a loop gap over 100 ms
requires a valid neutral manual input before movement is allowed. An off-center
stick cannot restart the motor simply by switching enable on. Source loss and
disable stop immediately. A normal nonneutral manual command slews from neutral;
returning into the deadband stops immediately.

| Suffix (both THR_BOW_ and THR_STN_) | Default | Meaning |
| --- | --- | --- |
| ENABLE | 0 | Master enable, 0 or 1 |
| MID | 1500 | Neutral/stop component pulse, 1000..2000 us |
| DB | 30 | Inclusive neutral half-width, 0..250 us |
| MAX | 1 | Manual limit, fraction 0..1 of symmetric travel about MID |
| SLEW | 2 | Travel fractions/second, >0..100 |
| HDG_EN | 0 | Optional heading participation, 0 or 1 |
| HDG_SIGN | 1 | +1 or -1: PWM direction for positive yaw demand |
| HDG_MAX | 0.3 | Travel fraction at unit heading demand, 0..MAX |
| DST | 0 | Reserved; not used |

Symmetric travel is `min(MID - 1000, 2000 - MID)`. Manual commands are bounded by
MAX; final physical scaling remains in PWMn_MIN/MAX/INV. Invalid configuration
inhibits the actuator. If MID itself is invalid, the component falls back to
1500 us and reports configuration error.

## Heading hold

One VHDG PID supplies the shared normalized yaw command. `VHDG_VSP_EN=1` preserves
the existing paired VSP heading behavior; set **VHDG_VSP_EN=0** for thruster-only
heading. Disabled VSP participation removes VSP input, geometry and saturation
requirements. VSP manual routing and VRS remain independent.

Each enabled, neutral, valid thruster with HDG_EN=1 and nonzero HDG_MAX receives:

```
output = MID + yaw_command * HDG_SIGN * HDG_MAX * symmetric_travel
```

VHDG_MAX bounds yaw_command. For example, at MID=1500, HDG_MAX=0.3 and VHDG_MAX=0.2,
the maximum automatic addition is 30 us. HDG_MAX controls relative strength;
HDG_SIGN must be calibrated independently for bow and stern and includes physical
output inversion. Identical signs are not automatically correct.

Automatic slew respects both VHDG_SLEW and each participating thruster's SLEW.
The existing anti-windup uses the constrained shared command. With VSP participation,
VSP ring headroom also constrains that command; thruster-only operation has no VSP
saturation dependency. This is fixed mixing, not measured torque allocation.

A valid off-neutral manual input on either enabled thruster releases heading hold,
even when its HDG_EN=0. Manual operation continues. Heading reacquires after the
existing VHDG_IDLE_MS neutral interval and captures current heading at its end.
Disabled/unrouted thrusters do not block engagement. Losing one thruster removes
its contribution and restarts capture with remaining available actuators; losing
all selected actuators inhibits heading and clears stored demand.

Thruster assistance/manual arbitration requires direct RC/override input routes:
component-to-component links involving thrusters inhibit heading while retaining
ordinary manual routing. This avoids automatic corrections feeding back through
another actuator. Unrelated VSP cross-feed does not inhibit thruster-only heading.

Example: RC1 bow, RC2 stern, RC3 bow-enable switch, RC4 stern-enable switch:

```
RT1_SRC=1    RT1_DST=121
RT2_SRC=2    RT2_DST=131
RT3_SRC=121  RT3_DST=9
RT4_SRC=131  RT4_DST=10
RT5_SRC=3    RT5_DST=201
RT6_SRC=4    RT6_DST=202
VFN1_TYPE=3  VFN1_ENABLE=1
VFN2_TYPE=4  VFN2_ENABLE=1
THR_BOW_ENABLE=1
THR_STN_ENABLE=1
```

Switch windows and level/toggle behavior use the normal VFN parameters. For
heading assistance additionally configure each HDG_EN/SIGN/MAX, VHDG_VSP_EN,
VHDG gains and VHDG_ENABLE. Heading still requires healthy compass/AHRS data.

## Status and tests

THR_TEL_HZ controls THRBOW_PWM / THRSTN_PWM, THRBOW_EN / THRSTN_EN and
THRBOW_ST / THRSTN_ST. State values: 0 disabled, 1 waiting for neutral, 2 manual,
3 heading assist, 4 input lost (including timing faults/unrouted input), 5 invalid
configuration. EN is the valid master/function gate, not a guarantee of movement.

```
python3 Tools/Vektor2/tests/test_thrusters.py
python3 Tools/Vektor2/tests/test_functions.py
python3 Tools/Vektor2/tests/test_heading_hold.py
python3 Tools/Vektor2/tests/test_mavlink_thrusters.py
```

The host tests use production actuator/heading code with ASan/UBSan. The MAVLink
test requires a current build/sitl/bin/Vektor2 and pymavlink. It tests real routing,
VFN gates, neutral interlocks and thruster-only heading without VSP input routes;
the rover simulation does not model physical thruster effectiveness.
