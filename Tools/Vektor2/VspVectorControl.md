# Manual VSP vector mode

Each drive has independent `VSPn_VMODE`: 0 (default) preserves legacy rod input;
1 selects manual thrust-vector input. Existing route IDs are unchanged.

| Routed input | Vector mode meaning |
| --- | --- |
| IN1 | 1000 backward, 1500 neutral, 2000 forward |
| IN2 | 1000 port, 1500 neutral, 2000 starboard |
| RPM_IN | Existing independent passthrough to RPM_OUT |

Vector mode requires fresh valid IN1/IN2, not RPM input. A motor can be routed
separately. Changing VMODE never changes the RPM policy or starts/stops a motor.

The shared mapper uses `DIR` (0 CW, 1 CCW, viewed from above), `MAP_ANG`, `X_C`,
`Y_C` and `LIM`. It applies installation rotation and the circular limit once.
Legacy `THR_ANG` is ignored in vector mode. `VSP_CONF=1` (AFT) and `2` (VWT)
remain vessel-layout metadata; this mode is not a vessel allocator or DP.

## Neutral transitions and limits

- `VSPn_VDB` defaults to 30 us (allowed 0..250): both inputs must be within
  1500 +/- VDB before movement is enabled. Outside that interval each axis is
  rescaled so 1000/2000 still requests full magnitude.
- `VSPn_VSLEW` defaults to 2 normalized vector units per second (positive, at
  most 100). Demands are radially limited before vector-space slew and mapping.
- Startup in vector mode, changing mode in either direction, mapper/control
  configuration edits, route revision changes, input loss/recovery, or a control
  loop gap above 100 ms require neutral before movement. Any route edit resets
  vector readiness, including an edit elsewhere in the routing table.
- Returning both inputs to neutral immediately commands neutral pitch. Input
  loss and invalid configuration also immediately command neutral pitch, with
  no ramp-down or HoldLast. Pitch neutral uses X_C/Y_C. An invalid center falls
  back to 1500 us; a configuration error is reported through the state key.
- Invalid VMODE values do not silently fall back to legacy control. To recover,
  select 0 or 1, correct the mapper/settings, and center both inputs.
- Returning to legacy emits a complete neutral cycle before releasing ownership.
  A drive that boots and remains in mode 0 retains the original behavior.

As before, physical PWM scaling/inversion follows component outputs. Verify that
configured centers actually put the physical rods at neutral. Vector magnitude
represents normalized pitch demand, not measured thrust or regulated rotor RPM.

## Controller ownership

VSP heading uses the same vector output path when both drives are in VMODE=1.
At neutral, the heading controller may capture and supply opposite longitudinal
vectors. Either manual input releases automatic ownership immediately. Legacy or
mixed modes inhibit VSP heading; there is no rod-Y fallback. See [VHDG.md](VHDG.md).

VRS remains inhibited while either drive is vector-owned or transitioning back
to legacy. Its enable parameter is preserved. Bow/stern manual and heading
controls remain independent, including thruster-only heading with VHDG_VSP_EN=0.

## Telemetry

Existing XA/YA and TDIR/TMAG reflect final commanded rods.
[VSPn_AUTH](VspAuthority.md) identifies manual ownership or a neutral guard and
any legacy heading/VRS combinations. At VSP_TEL_HZ, each
drive also publishes `VSP1_VST` / `VSP2_VST` via NAMED_VALUE_FLOAT:

| Value | State |
| --- | --- |
| 0 | Legacy ownership |
| 1 | Waiting for centered inputs |
| 2 | Vector mode ready/active (including neutral) |
| 3 | Missing/invalid X or Y input |
| 4 | Invalid mode or mapper/control configuration |

Neutral TDIR remains NaN and TMAG is zero when inputs and configuration are valid.
Missing rod inputs invalidate the estimate even though neutral is being commanded;
use XA/YA and VST to distinguish a stop from valid movement.

## Verification

```
ASAN_OPTIONS=detect_leaks=0 python3 Tools/Vektor2/tests/test_vsp_vector.py
./waf configure --board sitl
./waf --targets bin/Vektor2
python3 Tools/Vektor2/tests/test_mavlink_vsp_vector.py
```

Host tests cover state transitions, vector slew, immediate neutral, input loss,
route/configuration changes and stalls. SITL checks actual routed PWM for both
drives, legacy compatibility, independent modes, CW/CCW, installation angle,
THR_ANG bypass, RPM passthrough and RC-loss recovery. Physical force direction
still requires commissioning on the installed drives.
