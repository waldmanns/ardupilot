# Routed functions (VFN)



Ten fixed function slots convert routed microsecond signals into runtime
requests for built-in controller features. The compiled defaults leave every
slot unused and disabled; existing routing, master enables and controller
behavior are retained. This interface is implemented in firmware only. The
desktop/browser function drafts need a later client update to synchronize it.

## Parameters and discovery

All configuration uses normal persistent AP_Param/MAVLink parameters. The
presence of `VFN1_TYPE` identifies this interface. Existing parameter storage
IDs remain unchanged; the new subgroup is Logic group 10.

Replace `n` with 1..10, without an underscore after VFN: for example,
`VFN1_ENABLE`, `VFN10_HIGH`. All parameter names fit MAVLink's 16-byte field.

| Parameter | Default | Values |
| --- | --- | --- |
| `VFNn_ENABLE` | 0 | 0 disabled, 1 enabled |
| `VFNn_TYPE` | 0 | 0 unused, 1 VRS gate, 2 Auto heading gate, 3 Bow gate, 4 Stern gate, **7 NavMode** (5/6 retired; invalid until reconfigured) |
| `VFNn_MODE` | 0 | 0 level, 1 toggle on entry (NavMode requires 0) |
| `VFNn_LOW` | 1700 | Normal actions: inclusive lower bound. NavMode: Free / Heading boundary |
| `VFNn_HIGH` | 2000 | Normal actions: inclusive upper bound. NavMode: Heading / DP boundary |
| `VFN_TEL_HZ` | 1 | 0 stops function telemetry; 1..10 Hz |

There are 51 parameters. Unsupported types/modes, nonbinary ENABLE values,
reversed bounds, NavMode bands too narrow for +/-50us transitions, and out-of-range bounds report a configuration error and
contribute no request. Values are not silently corrected. Telemetry rate does
not affect evaluation, timing or the toggle latch; positive rates above ten
are bounded to ten. Custom display names remain client settings.

## Router interface

Use the existing `RT1_SRC/RT1_DST` through `RT32_SRC/RT32_DST` table:

| Destination | Code |
| --- | --- |
| Function 1 input (`FUNC1.IN`) | 201 |
| Function 2..9 input | 202..209 |
| Function 10 input (`FUNC10.IN`) | 210 |

Every existing router source is supported: RC1..RC18 (1..18), VSP1 X/Y/RPM
outputs (101/102/103), VSP2 outputs (111/112/113), bow output (121), and stern
output (131). Function consumers use the common source sampler, so added
source kinds can also be used here. Physical PWM ports remain consumers;
fan out their producer to a function to observe the same logical signal.

Each function input occupies one of the 32 routes and may have one producer.
A producer may feed multiple functions, component inputs and PWM consumers.
Duplicate configured producers inhibit the entire affected function, rather
than allowing whichever route was accepted first to control it. An invalid
source code targeting a function likewise inhibits that function.

Parameter-configured function routes are identity mappings over 800..2200 us;
they do not apply the actuator routes' 1000..2000 us clamp. Programmatic Route
scaling/reversal, if used, runs before the interval comparison. The original
source and final function pulse must both be valid and within 800..2200 us.
Component input validity propagates through all upstream components: a
nonzero fallback output cannot conceal missing source input. Physical PWM
HoldLast values never keep a function source valid.

## Evaluation and execution order

Each 100 Hz cycle:

1. Snapshot slot parameters and input-route bindings. Reset state for changed
   slots/bindings, errors or a loop gap greater than 100 ms.
2. Apply the requests sampled on the previous cycle, with the master enables.
3. Run each component once in the existing dependency order (or paired VSP
   order for effective heading enable).
4. Sample all function routes and evaluate requests for the next cycle.
5. Finish the ordinary physical PWM routing.

Every source has the same one-cycle application delay, normally 10 ms. This
also permits a VSP output to control a function affecting that same VSP. It
is a sampled feedback connection, with no recursive or repeated component
execution. Such a connection can change state every cycle if its output
moves across the activation interval; a delayed feedback path does not
guarantee a stable control choice.

For ordinary VFN actions, level mode requests the action whenever
`LOW <= pulse <= HIGH`. A zero-width interval (LOW=HIGH) matches exactly that
microsecond value. **NavMode is the sole exception**: `LOW` and `HIGH` are
ordered transition thresholds for a **required three-position RC switch**.
For NavMode, `Matched` and `Requested` reflect the selected non-Free mode,
including when the mode is held in a transition band. There is a fixed +/-50us
no-change interval around each threshold (not an additional mode): during a
transition, the last stable request is retained. RC loss/errors reset to Free.
Set LOW/HIGH between the *measured* stable switch positions.

Toggle mode starts off and requires a valid sample outside the interval to
arm. The next entry flips its request once. Remaining inside does not flip
again; leaving re-arms without changing the latched request. Booting or
recovering inside the interval cannot create an entry. Source loss, slot
disable, slot setting changes, input-route changes and loop gaps clear the
latch and arming state. Unrelated slot edits and unrelated route table edits
do not reset an unchanged function input. A removed/reconnected input resets
if the router actually observes the disconnected binding.

Newly sampled source loss removes the function request on the following
cycle, just like other function changes. The controllers' existing direct
sensor/input validity checks continue to apply independently. Toggle state,
captured heading targets and requests are never saved to persistent storage.

## M3.8.1 single NavMode action (observation only)

`VFNn_TYPE=7` (`NavMode`) is the **only** navigation mode selector. Use **one**
VFN slot and **one** routed RC input. Unlike other VFN actions, `LOW` and `HIGH`
are PWM thresholds, not a single inclusive activation interval:

`NavMode` **requires a physical three-position switch**, verified to have
three stable pulse values (for example 1000 / 1500 / 2000 us). It does not
support a documented two-position Free/DP configuration. `LOW` and `HIGH` are
transition centers, with a fixed 50 us no-change zone on either side:

| Input pulse (LOW=1300, HIGH=1700) | Requested `VNAV_MODE` |
| --- | --- |
| 800..1249 us | 0 Free |
| 1250..1349 us | No change: hold previously selected mode |
| 1350..1649 us | 1 Heading Hold |
| 1650..1749 us | No change: hold previously selected mode |
| 1750..2200 us | 2 Full DP |

On boot, the retained mode starts Free. Invalid/lost RC or invalid assignment
immediately resets to Free, even when the pulse lies in a no-change zone.
`VFNn_MODE` must be `0` (level). Thresholds must leave nonempty stable Free,
Heading and DP ranges and transition zones. A PWM signal alone cannot verify
the hardware switch has three detents; commissioning must confirm all three.
Standard VFN defaults (1700/2000) are not recommended: configure boundaries
between the actual stable switch readings.

Stored M3.8 types **5** (`NavHeading`) and **6** (`NavDP`) have been **retired**.
They are deliberately rejected with VFN configuration errors and cannot be
reinterpreted as type 7. Remove any old assignments (`TYPE=0`, disable the
unused slot and remove its route) and assign one new type-7 slot. A second
NavMode slot or mixing an old and new selector is a conflict that fails to
Free. Unrouted, disabled, invalid, out-of-range or lost RC input also fails
to Free; `NAV_RBITS` includes `CONFIG_INVALID` for configuration conflicts,
and the VFN slot errors explain invalid routes/inputs. Type 2 remains the
separate older auto-heading **enable gate**, not the navigation mode selector.

The selected request updates `VNAV_MODE` in **RAM only**; it is never saved
from RC changes. Removing the last selector returns the request to Free and
then allows direct parameter mode selection. The navigation master enables
are still separate. Only calculations and telemetry are enabled in this
revision: `NAV_STATE` remains `HDG_WAIT` when auto actuator ownership is
absent, even with `DP_CALC=1` / `HDG_CALC=1`.

Board configuration and migration instructions:
[NAVIGATION_M381_HANDOVER.md](NAVIGATION_M381_HANDOVER.md).

## Master enables and arbitration

`VRS_ENABLE` and `VNAV_HDG_ENABLE` remain persistent master enables. VFN does not
write or save them during runtime switching.

For each action:

```text
assigned = any slot has this TYPE, including disabled or invalid slots
request = OR of valid, enabled slots requesting this TYPE
effective_enable = master_enable AND (NOT assigned OR request)
```

Thus an assigned but disabled, disconnected or invalid slot fails off; it
cannot fall back to an enabled master. Other valid requests for the same
action can still enable it. Set TYPE=0 to remove an assignment. With no
assignments of that type, the master works exactly as before. Turning off a
master overrides every request; it does not alter slot configuration/latches.

Effective disable gates the relevant function. In the navigation-base stage,
heading gates `VNAV_HDG_ENABLE` readiness but no automatic yaw is yet wired.
Mode changes and eventual target capture are owned by the navigation supervisor.

## Telemetry

`VFN_TEL_HZ` independently controls a coherent snapshot of 31
`NAMED_VALUE_FLOAT` fields. Their common timestamp is the sample time, even
when transmission spans multiple control loops. Unsent fields are retried
per active link without blocking control. No VFN stream is emitted before
any slot is configured. Removing the last configuration sends one final
clearing snapshot, then stops. Setting the rate to zero stops pending sends
as well. Navigation status is independent and uses `NAV_STATE` / `NAV_RBITS`.

| Wire name | Meaning |
| --- | --- |
| `VFNn_IN` | Routed input pulse in us; NaN when invalid/unrouted |
| `VFNn_ST` | Exact integer status flags below |
| `VFNn_ERR` | Exact integer error flags below |
| `VFN_VRS` | Effective master/function-gated VRS enable used this cycle, 0/1 |

Here n=1..10. All names fit the ten-byte MAVLink name field. Routed input
and request match are available even for a disabled/unused slot, whenever
another slot keeps the stream active. ST bit 16 describes the **next** cycle's
request; `VFN_VRS` reports the VRS gate already used in the current cycle.
Navigation control status is only reported by `NAV_STATE`/`NAV_RBITS`.

| ST bit value | Meaning |
| --- | --- |
| 1 | Slot ENABLE=1 |
| 2 | A runtime producer route exists |
| 4 | Current source/input pulse valid |
| 8 | Pulse matches ordinary interval; for NavMode, a non-Free position |
| 16 | Slot requests its action |
| 32 | Toggle armed by a valid outside-range sample |

| ERR bit value | Meaning |
| --- | --- |
| 1 | Invalid slot configuration |
| 2 | Enabled assigned slot has no producer route |
| 4 | Invalid/duplicate configured route to this function |
| 8 | Enabled assigned slot has an invalid/missing/out-of-range source |
| 16 | Control loop gap exceeded 100 ms; state cleared |

## Example

```text
VRS_ENABLE  1
VFN1_TYPE   1
VFN1_MODE   0
VFN1_LOW    1700
VFN1_HIGH   2000
RT15_SRC    103       # VSP1 RPM_OUT (use 7 for RC7 instead)
RT15_DST    201       # Function 1 input
VFN1_ENABLE 1
```

Configure slots while disabled, then enable them after their parameters and
routes are ready. This example requests VRS while VSP1's valid routed RPM
command is inside the interval. RPM remains a command, not measured speed.

## Verification

```sh
ASAN_OPTIONS=detect_leaks=0 python3 Tools/Vektor2/tests/test_functions.py
python3 Tools/Vektor2/tests/test_heading_hold.py
python3 Tools/Vektor2/tests/test_roll_stabilization.py
./waf configure --board sitl
./waf --targets bin/Vektor2
python3 Tools/Vektor2/tests/test_mavlink_functions.py
python3 Tools/Vektor2/tests/test_mavlink_vhdg.py
python3 Tools/Vektor2/tests/test_mavlink_vrs.py
python3 Tools/Vektor2/tests/test_mavlink_rc.py
```

Host tests exercise production evaluator boundaries, toggle transitions,
loss/recovery, invalid settings/bindings, OR arbitration, next-cycle timing,
feedback and millisecond wrap under ASan/UBSan. SITL checks all parameters,
all current source endpoints, all ten slots, real routing/validity, master
gates, heading reacquisition and persistent configuration with volatile latches.

## Independent thruster enable functions

TYPE=3 gates bow enable; TYPE=4 gates stern enable. Each combines with its own
THR_BOW_ENABLE / THR_STN_ENABLE master. Both level and toggle modes work as above.
Disabling sends neutral immediately; enabling requires neutral before movement.
See [Thrusters.md](Thrusters.md).
