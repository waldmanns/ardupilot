# M2 — Host-only vessel thrust allocation (not an active navigation mode)

This is the **second implementation step** following M1's observation-only
navigation supervisor. The agreed navigation/propulsion baseline dated
23 November 2026 remains the design authority. M2 adds one small,
standalone `VesselThrustAllocator`, with **no links to live `Logic`,
`Routing`, `VspVectorControl`, `ThrusterControl`, or hardware PWM**. Neither
Heading Hold nor DP propels the boat yet.

## Inputs, coordinates and outputs

Call `VesselThrustAllocator::allocate(geometry, demand, bow_available)`.

- Vessel coordinates: `x` forward and `y` starboard, relative to a consistent
  reference point. Surge and sway use the same positive directions; the yaw
  moment is `x * sway_force - y * surge_force` (positive by that convention).
- `geometry.vsp1` and `.vsp2` contain individual `x`, `y` and *positive*
  nominal force capacities. The units must be consistent across drives.
- `geometry.bow_x` is the bow thruster's longitudinal lever arm, and
  `bow_force` its nominal lateral force capacity.
- `bow_available` is an **external ownership/health decision**, not inferred
  from the magnitude of a non-centering pot or joystick. It can only enable
  bow allocation with `Layout::AFT`. `Layout::VWT` never allocates the bow.
- The result contains separate VSP1 and VSP2 signed body-frame force vectors
  with norm <= 1, a signed bow command in [-1,1], computed achieved
  surge/sway/yaw, `saturated`, `demand_scale`, `valid`, and `bow_allocated`.
- A `valid=false` result is neutral in all three commands. The *future final
  owner* must reject invalid results. Neutral here is **not** a physical
  actuator safety guarantee, because M2 does not touch outputs.

The normalized VSP vectors are compatible in direction/convention with
`VspMapper::Vector`, but they are **not raw X/Y rod commands**. Do not
route them through RC channels. Future controlled VSP injection must reuse
`VspVectorControl` and `VspMapper`, with independent handover and slew guards.

## Model and deterministic constraint policy

For a normalized actuator vector `u = [VSP1_x, VSP1_y, VSP2_x, VSP2_y, bow]`:

```
Fx = c1 * u[0] + c2 * u[2]
Fy = c1 * u[1] + c2 * u[3] + cb * u[4]
Mz = c1 * (x1*u[1] - y1*u[0])
   + c2 * (x2*u[3] - y2*u[2]) + cb * xb*u[4]
```

The allocator computes the minimum Euclidean normalized-actuator effort
solution of this 3-axis linear model using a fixed-size 3x3 system. It
returns invalid if the configuration has no well-conditioned control over
all three axes. If either VSP's vector exceeds unit magnitude or the bow
exceeds unit amplitude, it **scales every output by a common factor**. This
conserves the requested wrench direction, while reporting saturation and the
reduced achieved wrench. It does *not* implement unagreed yaw-vs-translation
priority, saturation re-optimization, per-actuator slew, or thrust feedback.
There are no allocations on the heap and no third-party dependencies.

Uniform scaling is conservative. It can reduce all three axes even when a
more sophisticated reallocation would achieve more of the request. This
limitation and the absence of calibrated thrust curves must be addressed
before real DP tuning. Magnitudes here are only normalized *force requests*,
not a prediction of force from rod position or rotor RPM.

## Configuration / deployment limits

- Values in host tests are illustrative, **not vessel geometry**, not
  default firmware parameters, and not approved motor limits.
- M2 neither adds live geometry parameters nor activates any of the existing
  `VNAV_*` reserved gains and allocation weights. `VSP_CONF=0` remains an
  invalid automatic-layout request when integrating.
- No automatic output ownership, RC takeover detection, or return-to-auto
  logic is integrated. M1's `NavigationSupervisor` continues to report
  `HDG_WAIT` / `OUTPUT_NOT_READY` for requested automatic modes.
- The existing stern thruster stays excluded from navigation allocation.
- If the bow is manually owned, the allocator must be called with
  `bow_available=false`. The separate manual bow path is untouched.
- Full DP permissibility without an available bow in AFT has **not** been
  decided. A mathematically solvable VSP-only allocation does not make
  that an approved degraded DP operating mode.

## Verification

Run from the project directory:

```
python3 tests/test_vessel_thrust_allocator.py
```

This compiles C++11 under `-Wall -Wextra -Werror` with AddressSanitizer and
UndefinedBehaviorSanitizer. The host test checks neutral, independent VSP
vectors, all three motion axes, AFT bow on/off, VWT bow exclusion, saturation,
reported achieved wrench, rank deficiency, invalid geometry/NaN and a sweep
of signs/combined demands. The test has no access to actuators.

A live ArduPilot firmware compilation, SITL integration and hardware safety
check **have not been performed** in M2.
