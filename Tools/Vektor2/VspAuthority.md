# VSP rod authority telemetry

`VSP1_AUTH` / `VSP2_AUTH` are per-drive `NAMED_VALUE_FLOAT` bitmasks emitted
at `VSP_TEL_HZ`, reflecting the actual rod command path for this cycle.

| Value | Meaning |
| ---: | --- |
| 1 | Manual baseline owns rods (legacy inputs or active manual vector) |
| 4 | Roll stabilization (VRS) applied a nonzero correction |
| 8 | Vector controller guards output at neutral during a transition or invalid input |

Bit values combine (for example, 5 = manual plus VRS). A value of zero can
mean that legacy inputs were invalid, and does not promise that physical PWM is
at neutral. RPM routing is independent.

**There is currently no automatic navigation actuator owner.** The old bit 2
(heading correction) is unused and is not published as a compatibility alias.
Future heading and DP actuator ownership will be explicitly integrated after
the common allocator is tested. See `NAVIGATION.md` for the navigation status
and reason mask.
