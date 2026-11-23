# Vektor2 navigation / propulsion — M2 review and handover

**Date:** 23 November 2026  
**Input:** `Vektor2_Navigation_Base_M1(1).zip` plus the agreed
`Vektor2_Navigation_Propulsion_Agreed_Baseline(1).md`.

## What changed in M2

- Added one **pure C++11** `VesselThrustAllocator` (`.h` / `.cpp`).
- Added the host test (`tests/test_vessel_thrust_allocator.cpp`) and its
  sanitizer-backed runner (`tests/test_vessel_thrust_allocator.py`).
- Documented mathematical limits, wiring boundary and test scope in
  `ALLOCATION_M2.md`; updated `NAVIGATION.md` and `README.md`.
- **No actuator, routing, RC, navigation-state, firmware parameter or
  telemetry implementation was changed.** The existing manual behavior and
  M1 observation-only navigation are preserved by construction.

## Review of M1 against the agreed design

| Finding | Source inspection | Consequence / recommendation |
| --- | --- | --- |
| Good foundation | `NavigationSupervisor` exposes only state and reason bits; `Vektor2.cpp::update_navigation_status` sets `yaw_owned=false`, `translation_owned=false`, `output_ready=false`. | M1 correctly withholds ACTIVE status. Preserve this until the final actuator owner acknowledges control. |
| **Critical integration gate** | `Logic::run` and `Routing` execute live manual component/PWM routing, without an automatic navigation owner. | Put explicit arbitration before final actuator commands. Never inject fabricated RC values or write a parallel PWM path. |
| **Critical pair-override gate** | `VspVectorControl::apply` evaluates assistance per drive, conditional on that drive's centered inputs. | Neither drive may stay automatically vectored when the pilot has taken over the VSP pair using either stick. Pair-wide ownership and one neutral handover are needed. |
| **Critical independent bow gate** | `ThrusterControl::manual_override()` interprets non-neutral input as manual takeover in the existing heading-assist helper. | Do not reuse that condition for navigation bow arbitration with a non-centering pot. Require an explicit, agreed takeover event. |
| **Critical failsafe gate** | `Routing` retains a configurable RC-loss policy, with `HoldLast` as its default. | Confirm safe neutral/disable behavior per hardware drive before enabling navigation or propulsion bench testing. |
| Reserved controls only | Several `VNAV_*` parameters exist without executable control logic; for example the override parameter exposes Full/Blend/Nudge choices. | Do not activate blending, nudge, saturation weighting or gains until their actual ownership semantics are approved. |
| Status precision | DP_WAIT has the documented special meaning that heading ownership remains active. | Keep heading target and acknowledged yaw authority when position control waits; do not claim DP_WAIT if yaw authority is lost. |

These are **implementation gates**, not claims that an M1 automatic-control
fault has already occurred. The corresponding automatic paths are not live.

## Input and ownership contract for the next revision

**Agreed rules:** Free = each self-centering stick directly controls one VSP's
X/Y rods. In automatic modes, `VesselDemand{surge,sway,yaw}` is allocated to
independent VSP vectors. Deliberate motion of **either** VSP stick takes **both**
VSPs into independent manual rod control and suspends automatic VSP correction.
Bow takeover is independent and removes bow from automatic allocation without
changing VSP ownership. Safety/fault actions outrank every owner.

**Proposed interface boundary, not an adopted RC policy:** The future input
layer should report *events/booleans*, not synthetic rod PWM: `vsp_manual_claim`,
`vsp_reengage_requested`, `vsp_pair_neutral_confirmed`, `bow_manual_claim`,
`bow_auto_reengage_requested`, and `bow_auto_eligible`. The ownership layer
should yield mutually exclusive `VSP_PAIR_MANUAL` vs `VSP_PAIR_AUTO`, and
`BOW_MANUAL` vs `BOW_AUTO` vs `BOW_UNAVAILABLE` internally, without creating
new public navigation state telemetry. On a transition, the affected actuator
should be neutral-gated until its final path confirms safe ownership. The
existing `NAV_RBITS` and `NAV_STATE` must be populated from *actual authority*.

**Do not select specific RC channels yet.** The baseline leaves Heading Hold
surge/sway input unresolved while both sticks remain available for direct rod
override. A permanent pot is not a switch, and not all nonzero pot positions
indicate takeover. Keep all such choices out of executable code for now.

## Decisions required before active integration

1. Heading Hold vessel-level translation control mapping, separate from raw
   VSP rod inputs, including how it coexists with two sticks.
2. Pair-wide VSP override threshold, debounce, latch/hold and rearm action.
3. Bow manual-claim detection and return-to-auto action for non-centering input.
4. Whether AFT DP is inhibited or formally degraded when bow is unavailable.
5. Measured VSP/bow locations, actuator signs and calibrated relative force
   capabilities; RPM dependencies and neutral response.
6. Surge/sway/yaw saturation priorities, and whether reallocation is needed.
7. Exact RC-loss, timing-fault and drive-specific neutral/disable policy.

## Verified tests in this package

**Passed on host** (each runner invokes its own compilation):
`test_functions.py`, `test_navigation_supervisor.py`,
`test_roll_stabilization.py`, `test_sequencer.py`, `test_thrusters.py`,
`test_vessel_thrust_allocator.py`, `test_vsp_mapper.py`,
`test_vsp_vector.py`, `test_vtelemetry.py`.

**Not run:** `test_measure_points.py` needs
`libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat`, missing from the uploaded
source-only archive. `test_mavlink_*` requires a running firmware/SITL and is
not part of these host runs. A complete firmware Waf build is not possible
from this partial archive alone. This review must not be used as hardware
safety certification.

## Recommended M3 sequence

1. Agree on the exact translation input scheme and takeover/rearm policies.
2. Implement and host-test one **pair-wide VSP ownership arbiter** and an
   independent bow arbiter. Include RC loss, neutral gates and no simultaneous
   command owners. Do not connect propulsion yet.
3. Add safe allocator injection into the existing vector/rod map pipeline
   and bow-thruster conditioner, with owner acknowledgements. Test with
   actuators physically disconnected.
4. Build the **single shared heading controller** and confirm `HDG_ACTIVE`
   only when output ownership is real. DP follows once heading is stable.
5. Implement DP N/E position/velocity loop, transition preservation and
   saturation/availability policy; then full firmware, SITL and hardware tests.
