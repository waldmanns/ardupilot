# VSP rod authority telemetry

`VSP1_AUTH` / `VSP2_AUTH` are per-drive `NAMED_VALUE_FLOAT` bitmasks at
`VSP_TEL_HZ` and VTEL sources 66/67. RPM ownership is independent.

| Value | Meaning |
| ---: | --- |
| 1 | Manual baseline / pilot translation |
| 2 | Confirmed coordinated navigation yaw ownership |
| 4 | Roll stabilization applied a nonzero correction |
| 8 | Neutral guard / automatic rollback |
| 16 | Confirmed DP translation ownership |

Typical combinations: 1 = manual, 5 = manual + VRS, 3 = Heading Hold
(pilot translation + automatic yaw), 18 = Full DP, 8 = neutral guard.
Both VSP authority masks are committed together for automatic control, after
all required output writes succeed. A neutral guard or zero mask is not proof
that the physical actuator has reached neutral. See VSP_ALLOCATOR.md for the
pair status and NAV_RBITS definitions.
