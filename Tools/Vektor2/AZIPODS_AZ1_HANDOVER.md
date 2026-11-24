> **Historical AZ1 milestone.** For current AZ2 actuator behavior and commissioning restrictions, see [AZIPODS_AZ2_HANDOVER.md](AZIPODS_AZ2_HANDOVER.md). The architecture and IDs below remain relevant; the abstract-output restrictions have been superseded. Current layout selection is authoritative: AFT selects both drives automatically.

# Vektor2 — AZ1: abstract azipods and boot-selected main propulsion

## Status and intent

AZ1 is a source-level infrastructure milestone based on the Vektor2 0.6.4
archive `Vektor2(20261123-210757).zip`. Firmware version is deliberately unchanged.
It introduces the mutually exclusive VSP/AZI runtime families and the logical
azipod interface. It does **not** implement azimuth positioning or motor control.
No new automatic propulsion authority is granted. Host integration and regression
tests passed; the actual ArduPilot target build, persistent parameter verification
and hardware commissioning remain pending. Do not treat this archive as a
flash-ready or hardware-certified release.

The user wants one firmware supporting either VSP or azipods, never both main
propulsion mechanisms at once. Bow and stern tunnel thrusters are common,
independently enabled actuators in either configuration. The first AZI layout is
two stern azipods assisted by a bow tunnel thruster. Do not make the stern
thruster unavailable globally, or automatically enable the bow thruster.

## 1. Vessel geometry and coordinate convention

AFT, viewed from above:

```text
                 BOW / +X
                    ^
              /-----------\
             |  THR_BOW   |  transverse tunnel, optional/configured separately
             |            |
             |     O ---->| +Y (starboard)
             |            |
             | AZI1  AZI2 |  port and starboard, aft of origin
              \__________/
                  STERN
```

`AZI1` is port (negative Y); `AZI2` is starboard (positive Y). Both have
negative longitudinal X for AFT. Bow thruster is ahead of the reference point.
Use the existing body-frame convention: +X forward, +Y starboard, positive yaw
clockwise viewed from above. No assumed lever arms or geometry dimensions are
introduced. `AZI_CONF` is separate from `VSP_CONF`. The existing VSP AFT/VWT
meanings and VSP mapping parameters remain unchanged.

## 2. Parameters and persistent storage contract

| Name | Default | Values / semantics | Logic subgroup index |
| --- | ---: | --- | ---: |
| `MAIN_PROP_TYPE` | 0 | 0 VSP, 1 AZI; reboot required | 11 |
| `AZI_CONF` | 0 | 0 unspecified, 1 AFT; reboot required | 12 |

All names are within the 16-character MAVLink parameter limit. Existing Logic
subgroup IDs (1, 2, 3, 4, 5, 6, 7, 8, 10), nested VSP IDs, root parameter key
and reserved/retired slots are untouched. No format-version bump or storage
migration is intended. VSP remains the default for existing installations.

`AP_Param` registration is still static, as in the original firmware. Both
families' parameter objects exist, remain visible and are loaded by
`AP_Param::load_all()`. Their **runtime controllers** are exclusive, not their
parameter objects. This intentionally preserves stable persistent calibration.
Changing `MAIN_PROP_TYPE` or `AZI_CONF` via MAVLink changes the stored request,
not the active boot selection. `VSP_CONF` remains the original live parameter
within VSP mode; `Logic::vsp_configuration()` returns 0 outside VSP mode.
`AZI_CONF` is latched into `AzipodBackend::layout`; unsupported values are
interpreted as unspecified, with no AZI authority. Invalid `MAIN_PROP_TYPE`
values construct neither main backend (tunnel thrusters remain independent).

Do not simply move AP_Param stores into a union. Before making them selective,
inspect the exact target ArduPilot revision's support for pointer/dynamic
subgroups, early loading of the selector, defaults and parameter counting,
MAVLink enumeration, storage lookup and preservation of inactive parameters.
Retain existing keys and group elements. Validate switching both ways against
real saved parameters before changing the parameter storage model.

## 3. Runtime architecture and memory

`App::setup()` calls `logic.initialize_main_propulsion()` immediately after
`load_parameters()`, before board/output setup and route sync. The selection
is initialized once, including the invalid-selector case. Boot STATUSTEXT
reports VSP, AZI (abstract) and layout, or invalid selector.

`MainPropulsion.h` owns the permanent component IDs, family classification,
boot selection enum, runtime slot mapping and `MainPropulsionStorage<Vsp,Azi>`.
The storage is a tagged union with explicit placement construction/destruction.
Only one backend is constructed. Copy is prohibited; reinitialization is a
no-op. No heap allocation or runtime backend switching is introduced.

`VspBackend.h` contains the two `VspVectorControl` objects, the existing
`RollStabilizationBackend`, authority flags and shared roll enable state.
The VSP rod and vector algorithms were not rewritten. `AzipodControl.h`
contains the abstract `Components::AzipodControl`, separate `Components::AZI1`
and `Components::AZI2` types and `AzipodBackend`. The two components share an
implementation but remain independent instances. The small header-only stub
can be split into implementation files when AZ2 adds actual control logic.

`Logic::_runtime` is still four slots, rather than expanding to six:

| Slot | VSP selected | AZI selected |
| --- | --- | --- |
| 0 | VSP1 | AZI1 |
| 1 | VSP2 | AZI2 |
| 2 | ThrusterBow | ThrusterBow |
| 3 | ThrusterStern | ThrusterStern |

A non-selected ID maps to a sentinel outside the array. All public component
input/output/read-validity methods reject it before accessing slots. Never
index `_runtime` by a permanent ID without translating/checking the ID first.
The existing internal VSP drive and tunnel indices still map to their original
slots. `Logic::begin_cycle()` clears shared slots and updates roll stabilization
once only in VSP mode. `Logic::run()` dispatches by explicit family; no assumption
that every non-tunnel component is a VSP remains.

RAM nuance: shared static storage reserves the size of the larger backend
plus the tag/alignment in both modes. It avoids reserving the sum as AZI grows
and avoids two extra component I/O slots; it does not release the unused part
of the larger union on an AZI boot. Both parameter families and small common
telemetry objects still occupy RAM. Both algorithms remain in firmware flash.
Current host `sizeof` values (x86-64, mocked parameter/HAL adapters):
`VspBackend=288`, `AzipodBackend=3`, shared storage `292`, `Logic=928` bytes.
These are diagnostic host figures, NOT measured target RAM savings. The AZI
stub is tiny, so no material net saving over the VSP-only baseline is claimed.
Measure actual target .bss/.data, linker map and free heap for the old and new
firmware, and verify stack headroom on the compact board profile. If recovering
unused VSP RAM on AZI boot becomes essential, consider a one-time checked
allocation of only the selected backend, or separate build profiles. Do not
allocate or switch backends in the control loop.

## 4. Stable components and routing

| Permanent ID | Component | Destination codes / inputs | Source codes / outputs |
| ---: | --- | --- | --- |
| 0 | VSP1 | 101/102/103 IN1/IN2/RPM_IN | 101/102/103 OUT1/OUT2/RPM_OUT |
| 1 | VSP2 | 111/112/113 IN1/IN2/RPM_IN | 111/112/113 OUT1/OUT2/RPM_OUT |
| 2 | ThrusterBow | 121 | 121 |
| 3 | ThrusterStern | 131 | 131 |
| 4 | AZI1 | 141/142 X_IN/Y_IN | 141/142 AZ_OUT/MOT_OUT |
| 5 | AZI2 | 151/152 X_IN/Y_IN | 151/152 AZ_OUT/MOT_OUT |

All route signals remain `uint16_t` microseconds. The provisional AZI demand
contract is Cartesian: 1000..2000 us, 1500 = zero, 2000 = positive full axis
(forward/starboard), 1000 = negative full axis. Both outputs reserve logical
PWM endpoints, not yet calibrated angles or ESC commands. Third ports are
invalid. `AzipodControl::run()` emits zero (invalid/unpublished) and
`operational()` is false. Both `Logic::output_valid` overloads explicitly
reject AZI output. This separate gate must not be removed until a later
controller has a real validity/safety contract. Do not convert invalid zero
into 1000 us via output scaling or publish a guessed neutral pulse.

`RouteParameters.cpp` retains the formula-based decoder and existing 32 slots.
`Routing.cpp` uses explicit tunnel family checks for route masks and heading
output accounting. Execution graph validation includes all six registered IDs;
`Routing::prepare()` skips unselected components. Dormant routes are accepted
as syntactically valid and keep their destination claims; they do not acquire
valid source data or drive PWM. They participate in duplicate/cycle validation
just as active routes do. AZI outputs remain invalid even if both inputs
are valid. Downstream components, sequencer and VFN receive invalid
producer samples. Existing stale-output release behavior remains in place.

There is **one common RT table**, not independent stored route profiles.
Switching propulsion family does not rewrite RT parameters, infer wiring or
resolve duplicate physical outputs automatically. Disable/replace old routes
when commissioning the alternate configuration. Existing direct RC-to-PWM
and sequencer-to-PWM routes are not propulsion-aware and remain operational
according to their normal policies. A direct ESC route cannot be made safe
by this selector. Do not route physical azipod actuators directly to RC or
sequencer to bypass the abstract controller. Existing duplicate-consumer
validation retains its original first-accepted-route behavior. Verify actual
output wiring and remove conflicting routes before applying motor power.

Example abstract AFT configuration (not a motor commissioning recipe):

```text
MAIN_PROP_TYPE = 1
AZI_CONF = 1
# THR_BOW_ENABLE and THR_STN_ENABLE remain independent.
# RTn_SRC/RTn_DST may be configured for the logical demands:
RT1_SRC = 1
RT1_DST = 141
RT2_SRC = 2
RT2_DST = 142
RT3_SRC = 3
RT3_DST = 151
RT4_SRC = 4
RT4_DST = 152
# Leave actuator outputs unconnected for AZ1. Reboot after type/layout change.
```

## 5. Integration boundaries

- VSP named telemetry and VVSP log records are suppressed outside VSP mode.
  `VTelemetry` VSP/VRS sources become unavailable; existing source IDs are not
  reassigned. `Logic::roll_controller()` is nullable. VRS does not update in
  AZI/invalid mode, even if its saved enable is set. No new AZI telemetry yet.
- Tunnel thrusters retain their existing manual, VFN, neutral-wait and optional
  heading functionality in all boot configurations. The existing shared
  navigation heading code can still request tunnel-thruster authority. It is
  not observation-only when those thrusters are configured to accept heading.
- No changes to `VesselThrustAllocator`, navigation geometry, DP, or VSP
  mapper/control algorithm. No allocation to AZI, steering feedback or motor
  authority is added. Preserve calculation/actuator ownership separation.
- Local PWM and DroneCAN remain owned by Routing/PwmOut. Invalid component
  output is not written and previously driven route outputs are released.
  For CAN, removing a command is not an explicit actuator stop command.
  Hardware failsafe/timeout behavior and startup pulses require bench testing.

## 6. Verification and reproduction

From `Tools/Vektor2` (or this extracted folder):

```sh
python3 tests/test_main_propulsion.py
```

This new host integration test compiles the actual `Logic`, `Routing`,
`RouteParameters`, parameter metadata, VSP mapper/vector/legacy algorithms,
thruster controller, functions and sequencer with minimal host AP_Param,
RC/PWM and roll-sensor adapters. ASan/UBSan are enabled. Both compact and large
sequencer profiles passed. It tests default VSP, AZI and invalid selectors,
backend construction/destruction, reinitialization, slot isolation across all
byte-sized IDs, independent AZI inputs, legacy rod/RPM outputs,
invalid AZI source propagation, dormant output claims, invalid port rejection,
rewire release, VSP roll-update inhibition and both tunnel thrusters.

All 12 existing self-contained host suites also passed: functions, heading
hold (legacy algorithm), navigation DP, navigation RC, navigation supervisor,
roll stabilization, sequencer, thrusters, vessel thrust allocator, VSP mapper,
VSP vector and VTelemetry. `test_measure_points.py` depends on board hwdef
outside the archive and was not run. Host tests do not establish a valid target
AP_Param ABI, flash persistence, real HAL behavior or flight-board memory use.

The archive does not contain the ArduPilot tree, toolchain/board definitions
or an existing SITL binary. The following remain **not run**:

```sh
# From the project's existing ArduPilot integration root:
./waf configure --board sitl
./waf --targets bin/Vektor2
python3 Tools/Vektor2/tests/test_mavlink_main_propulsion.py
# Then run existing test_mavlink_*.py as applicable to this baseline.
./waf configure --board <actual-board>
./waf --targets bin/Vektor2
```

The new SITL test checks parameter defaults and persistence across VSP -> AZI
-> VSP process restarts using the same saved parameter store, no live backend
switch, VSP calibration retention, absent main outputs in AZI, and usable bow
thruster. It is provided but unexecuted. Run on the actual baseline and check
MAVLink enumeration, metadata and parameter groups. Compare board memory maps
with the original 0.6.4 build. Check boot and route transitions on a powered
controller with motors and propulsion actuators disconnected before any wet test.

## 7. Next milestone — AZ2 (not implemented)

First decide the steering hardware: position-controlled servo with mechanical
limits, continuous steering drive with measured angle, or intelligent angle
actuator. Establish whether propeller ESCs are reversible. Do not invent these
hardware assumptions. Build a pure vector-to-azimuth/thrust mapper and then a
stateful controller under the abstract AZI interface. Define calibration,
zero-demand angle hold, wrap/limits, invalid input handling, neutral interlock,
steering misalignment thrust gating based on actual feedback where available,
rate limits, reversal protection, and safe ownership before publishing outputs.
Add independent telemetry/logging and hardware commissioning procedures. Only
later generalize `VesselThrustAllocator` around main-unit geometry and actual
achievable thrust, with AZI steering latency explicitly modeled. Existing
VSP and tunnel behavior must remain regression-compatible.
