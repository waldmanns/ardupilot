# VSP rod authority telemetry

`VSP1_AUTH` and `VSP2_AUTH` are standard MAVLink NAMED_VALUE_FLOAT values at
`VSP_TEL_HZ`. Each contains an exact integer bitmask captured in the output
execution path for that control cycle. No new enable parameter is needed.

| Bit value | Meaning |
| --- | --- |
| 1 | Manual baseline owns rods: legacy inputs or ready manual vector control |
| 2 | Automatic heading is holding a target through this VSP |
| 4 | VRS actually added a quantized correction to this VSP this cycle |
| 8 | Vector transition/input-loss/configuration guard commands neutral rods |

Common values: 0 no valid manual owner or active assistance, 1 manual, 3 manual
plus vector heading, 5 legacy manual plus VRS, 8 neutral guard. Value 7 remains
defined by the bitmask but cannot occur while VRS is inhibited in vector mode.
Bits can be decoded independently; values 2, 4 and 6 describe assistance without
a manual baseline should a future command owner permit those combinations.

Manual includes centered sticks: it identifies the baseline command source,
not whether that source requests movement. Thus existing automatic heading
normally reports 3, not 2. Heading remains an owner at zero heading error while
holding the target; waiting for capture, manual release, sensor inhibition and
thruster-only heading do not set the VSP heading bit.

VRS is an intermittent overlay rather than an exclusive owner. Its bit is set
only on the drive receiving an actual correction, including continuous-mode
ramp-down. An enabled but idle, inhibited, zero-headroom or quantized-to-zero VRS
request does not set it. At low telemetry rates brief bursts can be missed: this
is a current-cycle snapshot, not a latched event or history of activity.

For vector mode, waiting for neutral, invalid input/configuration, and the final
neutral transition cycle report 8. Ready manual vector control reports 1, even
at neutral. VSPn_VST provides the more specific vector state. Both drives in vector mode can share heading authority (value 3); VRS remains
inhibited in vector mode.

Legacy missing X/Y inputs report 0 unless assistance actually owns/contributes.
This is an ownership indicator, not a promise that physical PWM is zero: legacy
fallback behavior is unchanged. RPM ownership is separate and is not included.
The bitmask is not derived from DIR, VMODE, VHDG_ENABLE or VRS_ENABLE alone.

All actuator calculations and arbitration are unchanged by this telemetry.
