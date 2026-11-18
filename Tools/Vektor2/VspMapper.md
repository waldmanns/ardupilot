# Shared VSP mapper

`VspMapper` is a pure, shared per-drive conversion between normalized body-frame
thrust demand and component rod pulse widths. It does not allocate vessel forces or command RPM.
[Manual vector mode](VspVectorControl.md) uses its forward API; telemetry uses
the inverse on final outputs from either legacy or vector control.

## Coordinates

Forward is positive toward the bow; starboard is positive to the right.
Direction is clockwise viewed from above: 0 forward, 90 starboard, 180 backward,
270 port. This is force on the vessel, not water discharge direction.
Positive rod X physically moves right; positive rod Y moves up (Y maximum).
Configure physical PWM inversion/linkages to satisfy these conventions.

For zero mounting angle, CCW rotation gives `rod_x=-forward`,
`rod_y=-starboard`; CW gives both positive. Rotor rotation is viewed from above.
Thus CCW left rod gives forward thrust, right gives reverse, up gives port and
down gives starboard. Mixtures combine both axes.

Installation angle is the local forward axis clockwise from the bow. Mapping
rotates the body vector into this frame, applies rotation sense and scales about
X_C/Y_C using LIM. The inverse returns to the body frame. Magnitudes above one
are radially limited; integer rounding is corrected inward to stay in the disk.
Quantization can slightly change direction, especially with a small LIM.

`VSP_CONF=1` is AFT and `VSP_CONF=2` is VWT (Voith Water Tractor). VSP1 is port,
VSP2 starboard, side by side. AFT locates the pair aft; VWT toward center/front.
Layout does not reverse individual thrust directions. Lever arms and vessel yaw
allocation belong in the future allocator, not this mapper.

## Parameters and compatibility

- `VSPn_DIR`: 0 CW (default), 1 CCW; other values are invalid.
- `VSPn_MAP_ANG`: installation angle in degrees, 0 default, valid [0,360).
- Existing `VSPn_X_C`, `VSPn_Y_C`, `VSPn_LIM` define the rod envelope.

The existing DIR parameter retains its storage ID and default. There is no
separate MAP_ROT parameter. DIR and MAP_ANG affect estimates in legacy mode,
and rod commands in opt-in vector mode. VMODE defaults to legacy; existing outputs are preserved by default.

THR_ANG already rotates legacy rods before heading/VRS; MAP_ANG describes physical
installation when decoding the final rods. The decoder does not undo THR_ANG.
Vector control uses the mapper directly, without passing its rod outputs
through the legacy THR_ANG transform a second time.

## Debug telemetry

At `VSP_TEL_HZ`, standard MAVLink NAMED_VALUE_FLOAT messages publish:

| Key | Meaning |
| --- | --- |
| VSP1_TDIR / VSP2_TDIR | Estimated body-frame thrust direction, degrees [0,360) |
| VSP1_TMAG / VSP2_TMAG | Normalized rod excursion, [0,1] |

Estimates use final component rod outputs after heading/VRS, before physical PWM
scaling/inversion. Existing XA/YA telemetry remains unchanged. Invalid mapping,
unavailable X/Y rod inputs, or rods beyond the circular envelope plus rounding tolerance yield NaN for both fields. Neutral yields intensity zero and direction
NaN. The inverse estimate tolerates up to one microsecond of integer rounding
per rod axis at the ring boundary and caps reported intensity at one, preserving
direction. This changes diagnostics only, not rod commands or controller limits.
RPM input is not required: it may be controlled separately or unrouted.
This does not change controller or routing validity rules. Setting VSP_TEL_HZ=0
disables the stream. Transport uses the existing
nonblocking named-value sender and may drop values on congested links.

This is commanded pitch direction/intensity, not measured force. Rotor speed,
linkage calibration, drive availability and hydrodynamics are not inferred.
A stopped rotor can still have nonzero reported pitch intensity. No vessel-wide
resultant force or yaw estimate is published until an allocator/model exists.

## Validation

```
python3 Tools/Vektor2/tests/test_vsp_mapper.py
python3 Tools/Vektor2/tests/test_roll_stabilization.py
./waf configure --board sitl
./waf --targets bin/Vektor2
python3 Tools/Vektor2/tests/test_mavlink_vsp_mapper.py
```

The mapper test covers both rotation directions, cardinal/diagonal demands,
mounting rotation, quantization, saturation, extreme and invalid inputs, and
inverse estimates. The SITL test checks actual named floats, both drives, layout
independence, final rod agreement, invalidation and preservation of rod outputs.
