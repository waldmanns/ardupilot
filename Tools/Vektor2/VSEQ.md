# Vektor2 sequencer

Set `SEQ_ENABLE=1` and refresh the parameter list. Edit the lane, block and
routing parameters in your usual ground station. Each setting saves through
the standard parameter system and survives reboot. The sequencer uses those
parameters directly, so it works without an SD card or an Apply step.

## One-lane example

These settings route lane 1 to PWM output 1 and send 1700 µs for one second,
then return to the 1500 µs idle value.

| Parameter | Value |
| --- | ---: |
| SEQ_ENABLE | 1 |
| SEQ1_ENABLE | 1 |
| SEQB_1_LANE | 1 |
| SEQB_1_POS | 1 |
| SEQB_1_DUR | 1000 |
| SEQB_1_TO | 1700 |
| RT1_SRC | 301 |
| RT1_DST | 1 |

Set `SEQ1_AUTOSTART=1` to start lane 1 on reboot. Set `SEQ1_REPEAT=0` to
play once or `SEQ1_REPEAT=1` to keep repeating until stopped. The same
repeat setting applies to manual and trigger starts. Both settings default
to 0 and can be edited while the lane is disabled.

## Parameters

| Parameter | Value |
| --- | --- |
| SEQ_VER | Control and telemetry interface 2 |
| SEQ_SCHEMA | Current file schema 3 |
| SEQ_LANES | 4 (compact) or 16 (large) |
| SEQ_PER_LANE | 16 or 32, respectively |
| SEQ_BLOCKS | 48 or 256, respectively |
| SEQ_OUTPUTS | Bit mask 3 (servo, binary via PWM) |
| SEQ_TYPES | Bit mask 7 (Set, Ramp, Wait) |
| SEQ_EDGES | Bit mask 7 (off, rising, falling) |
| SEQ_STORE | 1 for standard parameter storage |
| SEQ_MS_MIN / SEQ_MS_MAX | Block duration limits 100 / 60000 ms |
| SEQ_TEL_MAX | Maximum configured playback snapshot rate, 10 Hz |
| SEQ_EDGE_LAT | Trigger latency, 1 control cycle |

The selected target sets the profile explicitly. MatekH743 and sitl use large,
revo-mini and other targets use compact. Configure SITL with
`--vektor2-seq-profile=compact` to exercise the compact profile. The MatekH743 and revo-mini store their
parameters in onboard flash.

SEQ_ENABLE is a saved 0/1 activation switch, default 0. It follows the
capabilities in AP_Param key 20, group index 9. The dependent parameters follow
it in the same group and are omitted from enumeration when disabled. Refresh
the parameter list after activation changes. An in-flight list is restarted
on its next send if discovery has changed. Reads and writes by name work
while the sequencer is disabled. A rejected write returns the current value
and a warning.
All values must be finite integers in the documented range.

| Names | Range / interpretation | Default |
| --- | --- | --- |
| SEQ_HZ_TEL | 0–10 Hz, 0 disables periodic snapshots | 5 |
| SEQ_EDGE, SEQ_LOW, SEQ_HIGH | shared trigger fields (see below) | 0, 1500, 2000 |
| SEQn_ENABLE | 0/1, includes the lane in playback while active | 0 |
| SEQn_TYPE | 0 servo, 1 binary | 0 |
| SEQn_IDLE | Servo 1000–2000 µs or binary 0/1 | 1500 |
| SEQn_CLOCK | 0 shared, 1 independent | 0 |
| SEQn_EDGE, SEQn_LOW, SEQn_HIGH | independent trigger fields | 0, 1500, 2000 |
| SEQn_NAME1..4 | Three ASCII bytes per integer, least significant byte first | 0 (Lane n) |
| SEQn_AUTOSTART | 0 manual or trigger start, 1 opt in to boot playback | 0 |
| SEQn_REPEAT | 0 play once, 1 repeat until stopped | 0 |
| SEQB_n_LANE | 0 unused, 1–SEQ_LANES owner | 0 |
| SEQB_n_POS | 1–SEQ_PER_LANE, dense order within the lane | 0 |
| SEQB_n_TYPE | 0 Set, 1 Ramp, 2 Wait | 0 |
| SEQB_n_DUR | 100–60000 ms | 0 |
| SEQB_n_FROM, SEQB_n_TO | logical values, unused fields zero | 0 |

Name bytes are printable ASCII (32–126) followed by zero padding, at most 12
bytes. An empty name displays as “Lane n”. A shared lane's own EDGE must be
off. Edge 0 disables the trigger, 1 detects entry into the inclusive LOW–HIGH
window, and 2 detects exit. Window bounds are 800–2200 µs, LOW <= HIGH.
An invalid sample clears the baseline. Set uses TO. Ramp uses both FROM and
TO. Wait uses the previous output. A live slot must have a unique lane and
POS pair.
When LANE is zero the other fields stay saved and are ignored during
playback. Disabled lanes keep their content. A lane can remain empty, in which
case it publishes idle when enabled but cannot join a run.

A change to the effective sequence stops playback and clears playback power.
An incomplete or invalid combination stays editable and inhibits playback
until corrected. Disabling the sequencer releases its routed outputs and
keeps the saved settings. Vektor2 uses the board's parameter store, including
the space reserved for missions in other vehicle firmware. Existing Vektor2
parameter keys and offsets are preserved.

## Boot autostart

Set SEQn_AUTOSTART to 1 for each lane to run at boot. Enabled, nonempty
lanes with the flag set start from their beginning. Independent lanes start
individually. Selected shared lanes start together. Ordinary shared starts
include every enabled, nonempty shared lane. Disabled or empty lanes retain
their autostart and repeat settings. Enabling a lane during operation makes
it available for manual or trigger starts. Its boot settings apply on the
next reboot.

A selected independent lane or shared group with a configured trigger has
five seconds from sequencer initialization to receive a valid routed sample. That sample
qualifies the boot start and establishes the trigger baseline. When this
window expires, remaining pending starts are cleared. A lane or shared Stop
cancels the corresponding pending start. Stop All, explicit power control,
or deactivation cancels all pending starts. Playback power turns on when
the first selected run starts and remains on after completion. Outputs
return to idle and normal trigger handling continues.

Boot autostart requires a valid sequence and SEQ_ENABLE=1 at startup.
Changing the autostart flag takes effect on the next reboot. A pending start
is visible in VS_STATE bit 6.

## Routing and timing

RTn_SRC 301–316 are lane outputs 1–16. RTn_DST 401–416 are independent
triggers 1–16 and 417 is the shared trigger. Codes beyond the compiled
profile are rejected. All consume the ordinary 32 slots. Duplicate/invalid
trigger routes inhibit the consumer. Logical binary 0/1 is routed as
1000/2000 µs. PWM calibration applies at the final output as for other sources.

Power-off and Stop yield the configured idle of a valid enabled lane. With
activation off, a disabled lane or an unavailable document, the route
explicitly releases PWM, including under HoldLast. A dependent input route
receives an invalid sample. Triggers are sampled after component evaluation
and consumed on the next tick, including feedback. Initial/recovered inputs
set baseline without generating an edge. Binding changes stop affected runs
and reset their baselines. A configured trigger that becomes invalid stops
its run, including a manually started run. Command-only runs require no
RC signal. A tick gap above 100 ms stops playback and clears baselines.

Each block owns the half-open interval from its start to its end. At a
boundary the next block publishes immediately. A leading Wait holds idle.
Ramp starts at explicit FROM and uses integer interpolation rounded to
nearest with ties away from FROM. At the end, the ramp's final value is
exact. Short shared lanes hold their endpoint and final block until the
shared cycle ends. The longest participating lane sets the cycle duration
for the entire run. At each boundary, repeating shared lanes restart
together. Once-only shared lanes return to idle and report Complete at the
first boundary. Independent lanes use their own duration. A repeat keeps
the same run ID and wraps its elapsed time. The usual stop and trigger-loss
rules apply to repeats. Complete remains visible until a new run or a
stop/reset. Timestamps are unsigned boot milliseconds and wrap safely.

## Importing a legacy file

A MAVLink editor can import an old SD-based configuration once. Download the
desired legacy VSEQ0.BIN or VSEQ1.BIN from the card and upload it as
`VSEQNEW.BIN` using MAVFTP CreateFile, sequential WriteFile and
TerminateSession. A completed upload belongs to its sender system and
component. With playback powered off, use USER_2 operation 2 and the
uploaded file's CRC. A successful import writes the ordinary sequencer
parameters. Subsequent boots use those parameters.

All multibyte fields in the 20-byte header are little-endian.

| Offset | Field |
| --- | --- |
| 0 | Four bytes ASCII VSEQ |
| 4 | u8 schema (1, 2 or 3) |
| 5 | u8 lane count |
| 6 | u16 used block count |
| 8 | u32 exact length (26 + lane count × lane entry size + block count × 11) |
| 12 | u32 document revision |
| 16 | u32 IEEE CRC-32 (reflected polynomial EDB88320, initial and final xor FFFFFFFF, bytes 16–19 treated as zero) |

All schemas follow the header with u8 rate, u8 shared edge, u16 shared low
and u16 shared high, for a total of 26 bytes before the lane entries.
Lane entries are u8 one-based slot, u8 enabled, u8 type, u8 clock, u16 idle,
u8 edge, u16 low, u16 high, and 12 name bytes. Schema 1 ends the lane entry
there (23 bytes), with autostart and repeat off. Schema 2 appends a u8
SEQn_AUTOSTART value (24 bytes per lane). Schema 3 also appends a u8
SEQn_REPEAT value (25 bytes per lane). Block entries are u16 one-based pool slot,
u8 one-based lane, u8 one-based position, u8 type, u16 duration, u16 from
and u16 to. Each slot appears at most once. Omitted lanes default to disabled.
The file revision is ignored when importing legacy content. The
command separately checks the current active revision to guard against
overwriting a sequence that was changed after the editor read it.

## Commands

Address the device system and component explicitly. COMMAND_LONG parameters
and COMMAND_INT equivalents are identical. For COMMAND_INT, use
MAV_FRAME_MISSION and put fields 5 and 6 in x and y. Unused fields are zero.
The IDs are MAV_CMD_USER_1 and MAV_CMD_USER_2.

| Field | USER_1 controls | USER_2 validation and import |
| --- | --- | --- |
| 1 | 1 power on, 2 power off, 3 shared Start, 4 shared Stop, 5 lane Start, 6 lane Stop, 7 Stop All, 8 status | 1 validate current parameters, 2 import closed FTP document |
| 2 | lane 1..SEQ_LANES for 5/6, otherwise zero | zero |
| 3/4 | expected active revision low/high u16 (zero for status, stops and power-off) | expected active revision low/high u16 |
| 5 | nonzero u16 operation token | nonzero u16 operation token |
| 6/7 | zero | header CRC low/high u16 for FTP, zero for parameters |

Each sender has one outstanding command and one cached result. Increase the
operation token modulo 65536 and skip zero. Forward distance must be 1–32767.
The four-sender cache expires after 120 seconds without a request. Within
this window a repeated token returns its original result and never starts a
second run. Retransmit using the same payload. Simultaneous stops take
precedence over starts for their target. A start while already running is
ignored. Queued commands initially return IN_PROGRESS. Terminal ACK may be
retried on the addressed link under backpressure. Terminal result_param2
is 0 success, 1 locked, 2 busy, 3 stale revision/token, 4 invalid, 5 unsupported,
6 storage error, 7 unavailable, 8 inhibited. Malformed requests are DENIED.
The active revision is derived from the effective parameter values and stays
the same across reboot. Zero indicates an invalid sequence.

## Snapshot telemetry

Named integer values use the same boot timestamp for all fields in a
snapshot. Associate only complete frames between VS_BEGIN and VS_END with the
same timestamp and snapshot serial. Integer bit fields and u32 values use
the bit pattern of the signed MAVLink field. Per-link pending frames are
sent incrementally without mixing fields. At up to 30 bytes per unsigned
MAVLink 2 packet, a compact frame uses up to 750 bytes and a large frame up
to 2550 bytes. The sender runs at up to 40% of the reported capacity of each
link with a small burst allowance. Actual delivered cadence can be lower
than the configured rate under load. Request status with USER_1
operation 8 even at zero Hz. State changes can also request a snapshot.

| Field | Value |
| --- | --- |
| VS_BEGIN / VS_END | Identical snapshot serial |
| VS_REV | Active document revision |
| VS_STATE | bit 0 activated, bit 1 power, bit 2 config valid, bit 3 parameter storage has capacity, bit 4 busy, bit 5 shared trigger valid, bit 6 boot autostart pending, bits 8–10 shared state, bits 12–15 shared reason, bits 16–19 config status |
| VS_SHARED | Shared run ID, zero until first shared run |
| VSLnn_OUT | bits 0–15 logical value, bits 16–31 routed µs (when valid) |
| VSLnn_BLK | bits 0–8 stable one-based block slot (0 none), bits 9–14 one-based order (0 none), bit 15 holding, bits 16–18 lane state, bit 19 output valid, bit 20 trigger valid, bits 21–24 reason, bit 25 binary |
| VSLnn_TIME | block elapsed in low u16, block duration in high u16, both ms |
| VSLnn_EL | lane elapsed ms, clamped to lane length |
| VSLnn_RUN | last run ID (0 before first) |

States: 0 Disabled, 1 Idle, 2 Armed, 3 Running, 4 Complete. Reasons:
0 none, 1 stopped, 2 power off, 3 input loss, 4 route changed, 5 loop gap,
6 configuration, 7 deactivated. Configuration states are 0 loading, 2 valid,
and 3 invalid.

## Verification

Run `python3 Tools/Vektor2/tests/test_sequencer.py` for both engine and codec
profiles. Run `python3 Tools/Vektor2/tests/test_mavlink_sequencer.py --binary
<SITL binary>` against the large and compact SITL builds for parameter
readback, persistence and boot playback without sequence files.
