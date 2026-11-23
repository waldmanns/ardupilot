# Independent bow and stern thrusters

> **Navigation-base revision:** Any references below to `VHDG_*`, `VHDG.md` or `HDG_STATE/INHIB/REASON` describe the removed pre-navigation firmware. There is no active legacy heading controller. The only navigation status interface is `NAV_STATE`/`NAV_RBITS`; see `NAVIGATION.md`.

Bow and stern are independent stateful actuators. Neither requires a VSP route,
VSP geometry, RPM input, or VRS. They currently run manually; future navigation will supply a shared
yaw demand. They never receive roll correction.

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

## Navigation integration (not active yet)

The thruster controller supports a bounded yaw correction (`apply_heading()`),
including individual sign and travel limits. This is an internal actuator
capability, **not** an active Heading Hold mode in this revision. No `VHDG_*`
controller runs, and no navigation code invokes the yaw correction.

The next stage will feed yaw from the single `VNAV_` heading controller through
one tested output-authority path. Until then, only routed manual thruster
commands are used. For initial bench configuration, set `HDG_EN=0`.

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
