# Vektor PC Client Handoff — Current Firmware

This file is the implementation handoff for an agent building a PC client for
the Vektor firmware in this workspace. It describes what the firmware actually
supports now, including the current reduced F405 (`revo-mini`) target.

The complete wire-level design is in
[`VEKTOR_SERIAL_PROTOCOL_TRUTH.md`](VEKTOR_SERIAL_PROTOCOL_TRUTH.md). The
current implementation is in [`Vektor_Protocol.cpp`](Vektor_Protocol.cpp),
[`Vektor_SerialProtocol.cpp`](Vektor_SerialProtocol.cpp), and
[`Vektor_Schema.cpp`](Vektor_Schema.cpp). If this handoff and the code ever
disagree, use the current firmware code and update this file.

## Required first deliverable

Build a client that can:

1. discover and open the Vektor USB CDC port;
2. encode and decode Vektor Protocol v1 frames;
3. complete `HELLO`;
4. download all current descriptors with `DESCRIBE`;
5. fetch all exposed parameters with `GET_ALL_PARAMS`;
6. read and apply one parameter with `GET` and `SET`, understanding the
   asynchronous persistence contract;
7. list, create, replace, and delete assignment routes;
8. subscribe to current realtime fields and expose their quality state;
9. reconnect cleanly after the device resets or re-enumerates.

Do not use MAVLink for this connection. The application USB port currently
carries the native Vektor binary protocol.

## USB CDC transport

On `revo-mini`, the hardware serial order begins with `OTG1`. The board
capability explicitly maps the Vektor transport to HAL serial index 0, and the
firmware opens that same index. The advertised USB endpoint is therefore the
application CDC serial byte stream carrying this session. UART-capable future
boards must provide endpoint-to-HAL-index mappings, and capability validation
rejects a claimed UART endpoint whose mapped index differs from the index the
firmware will open.

Current reduced-board USB information:

- VID:PID: `1209:5741`;
- Linux: normally `/dev/ttyACM0`, but never assume the number;
- Windows: a dynamically assigned `COM` port;
- macOS: normally a `/dev/cu.usbmodem*` device;
- application identity after `HELLO`: board ID `124`, hardware type
  `Vektor Core Reduced F405`, revision `revo-mini`.

Use VID/PID only to produce candidates. Confirm a device by receiving a valid
Vektor `HELLO` response. The bootloader can enumerate separately and will not
answer the application protocol; wait for re-enumeration after flashing or
reset.

Open the CDC port using conventional `115200 8N1`, no flow control. USB CDC has
no physical UART baud, so the line-rate selection is not significant for USB.
The `SYS_PROTO_BAUD` parameter matters when this adapter is later placed on a
real UART.

Serial reads are arbitrarily chunked. One read can contain a partial frame,
one frame, or several frames. Never equate an OS read with a protocol frame.
Run serial I/O and parsing away from the UI thread.

The firmware emits a `0x00` delimiter when its protocol service starts. Empty
delimiter spans are legal and must be ignored.

## Frame codec

All multibyte values are little-endian.

Build the decoded frame as:

```text
offset  size  value
0       1     version = 1
1       1     message_type
2       1     flags
3       2     request/stream sequence
5       2     payload length N
7       N     payload
7+N     4     CRC-32/ISO-HDLC
```

The CRC covers the decoded bytes from `version` through the last payload byte.
Append the CRC as a little-endian `u32`, COBS-encode the complete decoded
frame, then append one `0x00` stream delimiter.

Current limits:

- maximum payload: 256 decoded bytes;
- decoded frame size: `11 + payload_length`;
- maximum decoded frame: 267 bytes;
- maximum encoded stream buffer: 271 bytes including delimiter.

CRC parameters are the standard Ethernet/zlib CRC-32:

```text
polynomial  0x04C11DB7
reflected   0xEDB88320
init        0xFFFFFFFF
xorout      0xFFFFFFFF
check       CRC32("123456789") = 0xCBF43926
```

For example, Python's `zlib.crc32(data) & 0xffffffff` produces the required
numeric CRC.

The client parser must:

- accumulate nonzero bytes until `0x00`;
- ignore an empty block;
- discard an oversized encoded block until the next delimiter;
- COBS-decode the block;
- require at least 11 decoded bytes;
- require `version == 1`;
- require `decoded_length == 11 + payload_length`;
- verify CRC before dispatch;
- discard malformed frames and resume at the next delimiter.

Use the canonical PING vector in the protocol truth document as a codec unit
test. Its complete delimited stream is:

```text
00 03 01 03 04 34 12 0C 11 78 56 34 12 08 07 06 05 04 03 02 01 56 B8 D9 95 00
```

## Sequences, responses, and retry behavior

Use a monotonically increasing `u16` sequence for host requests. It wraps at
65535. A direct response:

- copies the request sequence;
- sets `FLAG_RESPONSE` (`0x01`);
- can be an `ERROR` response instead of the expected success type.

For the first client, allow only one outstanding control request at a time.
This keeps reconnect, timeout, and paging behavior simple while still allowing
unsolicited telemetry to be processed.

On timeout, resend the exact same encoded request with the same sequence. The
device keeps four recent responses and will replay a matching request. Do not
reuse a recent `(message_type, sequence)` with different payload bytes; the
device returns `SEQUENCE_CONFLICT`.

Relevant flags are:

```text
0x01 FLAG_RESPONSE
0x02 FLAG_MORE
0x04 FLAG_URGENT
0x08 FLAG_VOLATILE
```

Ignore unknown flag bits for protocol major 1.

## Implemented message set

The following requests are implemented now:

```text
0x01 HELLO
0x02 DESCRIBE
0x03 PING
0x10 GET
0x11 SET
0x13 GET_MANY
0x14 SET_MANY
0x16 GET_ALL_PARAMS
0x20 SUBSCRIBE
0x21 UNSUBSCRIBE
0x40 ROUTE_LIST
0x41 ROUTE_SET
0x42 ROUTE_DELETE
```

The client must receive these device message types:

```text
0x01 HELLO
0x02 DESCRIBE
0x03 PING
0x04 ERROR
0x12 VALUE
0x15 VALUES
0x20 SUBSCRIBE
0x21 UNSUBSCRIBE
0x22 TELEMETRY
0x43 ROUTES
```

Actions, events, and all file messages are reserved by protocol v1 but are not
implemented. A valid request for a known but unimplemented message receives
`NOT_AVAILABLE`. `max_file_chunk` is currently zero.

## Connection and discovery sequence

Use this sequence every time a port is opened or the device reconnects:

1. Reset the host stream parser and request state.
2. Optionally write a single `0x00` parser-reset delimiter.
3. Send `HELLO` and wait for its correlated response.
4. Read `schema_hash` and `capability_hash` from the response.
5. Load cached descriptors only when both hashes match the cache key.
6. Otherwise page through the implemented `DESCRIBE` domains.
7. Fetch current values with `GET_ALL_PARAMS`.
8. Fetch persistent routes with `ROUTE_LIST`.
9. Start requested realtime subscriptions.

A new successful `HELLO` resets the firmware's subscriptions and request replay
state. Treat it as the start of a new logical session. With the exception of
protocol response frames, every other request is rejected with `INVALID_STATE`
until `HELLO` succeeds. An identical `HELLO` retry is replayed; any other valid
`HELLO` starts a fresh session even if it reuses a sequence number.

### HELLO

Request type `0x01`, flags `0`:

```text
client_min_major       u8  = 1
client_max_major       u8  = 1
client_proto_minor     u16 = 0
client_feature_flags   u32 = 0
client_nonce           u32 = random/non-repeating session value
```

Response type `0x01`, `FLAG_RESPONSE`:

```text
selected_major          u8
server_proto_minor      u16
firmware_version        str8
hardware_type           str8
hardware_revision       str8
board_id                u32
schema_hash             u64
capability_hash         u64
device_id               u64
capability_flags        u64
max_payload             u16
max_file_chunk          u16
max_subscriptions       u16
max_realtime_hz         u16
control_update_hz       u16
attitude_update_hz      u16
server_nonce            u32
```

`str8` is a one-byte length followed by that many UTF-8 bytes, without a NUL.
Current firmware reports protocol minor `0`, max payload `256`, max file chunk
`0`, up to four subscriptions, and up to 100 Hz realtime output.

Do not identify functionality from the display strings or board name. Use the
capability flags and structured descriptors.

### PING

Request type `0x03`:

```text
cookie       u32
host_time_us u64
```

Response type `0x03`:

```text
cookie          u32
host_time_us    u64
device_time_us  u64
```

Use PING for connection diagnostics, not as a clock synchronization protocol.

## Descriptor download

`DESCRIBE` requires a successful `HELLO` first.

Request type `0x02`:

```text
domain       u8
cursor       u32
max_records  u16
```

Use cursor zero for the first page. A `max_records` of zero asks the device to
use its configured page size, currently four by default. Each subsequent page
is a new request and should use a new host sequence.

Response type `0x02`:

```text
domain        u8
next_cursor   u32
record_count  u16
repeat record_count times:
    record_length  u16
    record         u8[record_length]
```

Continue until `next_cursor == 0`. `FLAG_MORE` is also set while another page
exists. Parse using `record_length`; skip trailing bytes that are unknown to
the client.

Implemented domains:

```text
0x01 BOARD
0x02 COMPONENT
0x03 FIELD
0x04 ENDPOINT
0x05 TIMER_GROUP
0x07 RUNTIME_LIMIT
```

Domains `STORAGE_AREA`, `ENUM_TABLE`, `EVENT_TYPE`, and `ACTION_SCHEMA`
currently return a valid empty page. Do not mistake the storage hardware
endpoint for an implemented file service.

Current reduced F405 counts are expected to be:

```text
BOARD          1
COMPONENT      6
FIELD         69
ENDPOINT      15
TIMER_GROUP    2
RUNTIME_LIMIT  1
```

Use those counts only as integration-test expectations. Production client code
must enumerate until `next_cursor == 0`.

### Descriptor record layouts

BOARD record v1:

```text
record_version     u8
board_id           u32
hardware_type      str8
hardware_revision  str8
display_name       str8
```

COMPONENT record v1:

```text
record_version     u8
component_id       u32
component_type_id  u32
instance           u16
flags              u32
name               str8
display_name       str8
```

FIELD record v1:

```text
record_version      u8
field_id            u32
owner_component_id  u32
field_kind          u8
type_id             u8
flags               u32
name                str8
display_name        str8
units               str8
[minimum typed value if FIELD_HAS_MIN]
[maximum typed value if FIELD_HAS_MAX]
[default typed value if FIELD_HAS_DEFAULT]
```

Field kinds:

```text
0x01 INPUT
0x02 OUTPUT
0x03 PARAMETER
0x04 OBSERVABLE
```

Field flags:

```text
bit 0   READABLE
bit 1   WRITABLE
bit 2   PERSISTENT
bit 3   REALTIME
bit 4   HAS_MIN
bit 5   HAS_MAX
bit 6   HAS_DEFAULT
bit 7   APPLY_LIVE
bit 8   APPLY_RECONFIGURE
bit 9   APPLY_REBOOT
bit 10  ROUTABLE
bit 11  HIDDEN_NORMAL_UI
bit 12  DEVELOPER_ONLY
```

ENDPOINT record v1:

```text
record_version      u8
endpoint_id         u32
endpoint_kind       u8
parent_endpoint_id  u32
flags               u64
name                str8
display_name        str8
```

For UART and USB records, flags bit 0 means externally usable input, bit 1
means externally usable output, and bit 2 identifies the endpoint carrying the
Vektor protocol. Flex records interpret the same word as their per-port mode
bits. On the reduced board, endpoints currently describe USB, one PWM bank,
six PWM channels, three UARTs, three ADCs, one IMU, and storage hardware.

TIMER_GROUP record v1:

```text
record_version       u8
group_id             u32
member_count         u8
member_endpoint_ids  u32[member_count]
rate_count           u8
supported_rates_hz   u16[rate_count]
current_rate_hz      u16
```

Treat `member_endpoint_ids` as authoritative. Firmware builds this list from
explicit board channel masks, not from contiguous channel assumptions. On
`revo-mini`, the two records are PWM 1-2 (TIM3) and PWM 3-6 (TIM2); the timer
names are an implementation note and are not carried on the wire.

RUNTIME_LIMIT record v1:

```text
record_version       u8
limit_id             u32
capability_flags     u64
max_payload          u16
max_file_chunk       u16
max_subscriptions    u16
max_realtime_hz      u16
control_update_hz    u16
attitude_update_hz   u16
```

## Primitive values

Current fields use these protocol types:

```text
0x04 U16      2 bytes
0x05 I16      2 bytes, two's complement
0x06 U32      4 bytes
0x07 I32      4 bytes, two's complement
0x0A FLOAT32  4 bytes, IEEE-754 binary32
```

Implement the full type table from the truth document in the reusable codec,
but always decode a field using the type reported by its descriptor. In
`VALUE` and `VALUES`, each value includes its `type_id`. In telemetry, types
are omitted because the subscription order and field descriptors define them.

Stable IDs are FNV-1a 32-bit hashes of canonical UTF-8 paths:

```text
offset basis  0x811C9DC5
prime         0x01000193
```

Prefer IDs delivered in descriptors. Computing IDs from paths is useful for
tests and command-line tooling, but the UI must build itself from discovery.

## Parameter API

### GET and VALUE

`GET` request type `0x10`:

```text
field_id  u32
```

Successful `VALUE` response type `0x12`:

```text
field_id  u32
type_id   u8
value     typed bytes
```

### SET

`SET` request type `0x11`:

```text
field_id  u32
type_id   u8
value     typed bytes
```

Successful response is `VALUE` containing the accepted/applied value, which
firmware has queued for `AP_Param` persistence. This is not a physical-save
acknowledgement: immediate power loss can restore the prior value. If a
workflow needs proof across reboot, reconnect after reboot and `GET` the value.
Honor the descriptor's bounds and type locally, but always handle firmware-side
validation errors.

### GET_MANY, SET_MANY, and GET_ALL_PARAMS

`GET_MANY` request type `0x13`:

```text
count      u16
field_ids  u32[count]
```

`SET_MANY` request type `0x14`:

```text
count  u16
repeat count times:
    field_id  u32
    type_id   u8
    value     typed bytes
```

Both are currently limited to eight fields per request. Duplicate IDs in a
`SET_MANY` request are rejected. Validation overlays the complete batch before
checking related PWM `minimum_us`, `trim_us`, `maximum_us`, and `failsafe_us`
constraints, so a valid coordinated calibration update succeeds and an invalid
final tuple changes nothing.

Like `SET`, a successful `SET_MANY` response means accepted/applied and queued,
not physically durable.

Successful bulk response type `0x15`:

```text
count  u16
repeat count times:
    field_id  u32
    type_id   u8
    value     typed bytes
```

`GET_ALL_PARAMS` is request type `0x16` with an empty payload. Current firmware
returns all board-available readable parameters in one `VALUES` frame. Code the
receiver so it can tolerate future `FLAG_MORE` pages even though the current
implementation does not emit them here.

### Current parameter inventory

The client should still discover these through FIELD descriptors. This list is
an acceptance-test and UI-semantics reference.

| Canonical field path | Type | Range/default | Apply |
| --- | --- | --- | --- |
| `component/system/0/parameter/sys_options` | I32 | 0..2147483647, default 0 | live |
| `component/system/0/parameter/sys_desc_page` | I16 | 1..16, default 4 | live |
| `component/system/0/parameter/sys_protocol_baud` | I32 | 9600..921600, default 115200; UART transport builds only | reboot |
| `component/rcin/0/parameter/uart_port` | I16 | 0..9, default 1 | reboot |
| `component/rcin/0/parameter/timeout_ms` | I16 | 50..5000, default 500 | live |
| `component/rcin/0/parameter/protocol_mask` | I32 | 0..131071, default 1 | live |
| `component/pwm_input/0/parameter/channel_1_pin` through `channel_6_pin` | I16 | -1..255, default -1 | reboot |
| `component/pwm_input/0/parameter/timeout_ms` | I16 | 20..2000, default 100 | live |
| `component/pwm_input/0/parameter/minimum_us` | I16 | 800..2200, default 1000 | live |
| `component/pwm_input/0/parameter/trim_us` | I16 | 800..2200, default 1500 | live |
| `component/pwm_input/0/parameter/maximum_us` | I16 | 800..2200, default 2000 | live |
| `component/pwm_output/0/parameter/rate_hz` | I16 | exactly 50, 100, 200, or 330; default 50 | live |
| `component/pwm_output/0/parameter/minimum_us` | I16 | 800..2200, default 1000 | live |
| `component/pwm_output/0/parameter/trim_us` | I16 | 800..2200, default 1500 | live |
| `component/pwm_output/0/parameter/maximum_us` | I16 | 800..2200, default 2000 | live |
| `component/pwm_output/0/parameter/reverse_mask` | I16 | 0..4095, default 0 | live |
| `component/pwm_output/0/parameter/failsafe_us` | I16 | 0..2200, default 0 | live |

For each input/output calibration, firmware requires:

```text
minimum_us < trim_us < maximum_us
```

For output failsafe, zero disables the output when an assigned signal is stale
or invalid. A nonzero value is bounded to the active output calibration range.
Unassigned outputs remain disabled regardless of failsafe.

The receiver protocol mask is currently exposed as a numeric bitmask because
the firmware does not yet publish an ENUM_TABLE for it. Do not invent a wire
enum. A basic client may show the numeric/hex mask and explain that the compiled
autodetector includes SBUS, iBUS, DSM, CRSF/ELRS, FPort, SUMD, SRXL, and Ghost
where enabled by the board build.

## Current readable and realtime fields

The seven current components are:

```text
component/system/0       protocol service
component/system/1       runtime service
component/attitude/0     onboard attitude estimator
component/vsp/1          VSP skeleton
component/rcin/0         serial/receiver RC source
component/pwm_input/0    GPIO PWM input source
component/pwm_output/0   physical PWM output sink
```

Current observables:

```text
component/system/0/observable/rx_frames
component/system/0/observable/rx_drops
component/system/0/observable/tx_drops
component/system/1/observable/uptime_ms
component/system/1/observable/loop_count
component/system/1/observable/loop_dt_us
component/system/1/observable/loop_work_us
component/system/1/observable/loop_max_work_us
component/system/1/observable/loop_late
component/system/1/observable/loop_lateness_us
component/system/1/observable/loop_late_count
component/system/1/observable/service_rate_hz
component/attitude/0/observable/roll_deg
component/attitude/0/observable/pitch_deg
component/attitude/0/observable/yaw_deg
component/attitude/0/observable/quaternion
component/attitude/0/observable/body_rates_rad_s
component/rcin/0/observable/channel_1_us through channel_16_us
component/pwm_input/0/observable/channel_1_attach_status through channel_6_attach_status
component/pwm_input/0/observable/configuration_valid
component/pwm_output/0/observable/configuration_valid
component/pwm_output/0/observable/effective_rate_hz
component/pwm_output/0/observable/effective_failsafe_us
```

`loop_late` explicitly reports whether the preceding completed loop started
after its scheduled time or finished after the following deadline.
`loop_lateness_us` reports the larger overrun, and `loop_late_count` increments
once per late loop. A realtime monitor should alert on `loop_late` or a change
in the counter instead of inferring lateness from `loop_dt_us`.

Current routable/readable FLOAT32 fields:

```text
component/vsp/1/input/x
component/vsp/1/input/y
component/rcin/0/output/channel_1 through channel_16
component/pwm_input/0/output/channel_1 through channel_6
component/pwm_output/0/input/channel_1 through channel_12
component/pwm_output/0/output/channel_1 through channel_12
```

Routable RCIN, PWM input, and PWM output-command values are normalized FLOAT32
values, normally `-1.0..+1.0`. The RCIN observable channels and physical PWM
output fields are U16 pulse widths in microseconds. An inactive output reports
zero with INVALID quality. Attitude roll/pitch/yaw are FLOAT32 degrees;
quaternion is `QUATERNIONF` in `w,x,y,z` order; body rates are `VECTOR3F` in
`x,y,z` body axes and rad/s.

The VSP control-law update remains intentionally empty. Its output fields stay
invalid until that core is implemented manually and are not routable; do not
present them as a working mixer.

The schema is filtered by board capability. The reduced F405 descriptor surface
contains only its six physical PWM input/output channels; absent channels are
not returned by `DESCRIBE`, and direct operations on their stable IDs are
rejected with `NOT_AVAILABLE`.

## Realtime subscriptions

Only fields with both `READABLE` and `REALTIME` flags can be subscribed.
`SUBSCRIBE` requires `HELLO` first.

Request type `0x20`:

```text
requested_period_us  u32
field_count          u16
field_ids            u32[field_count]
```

Limits now:

- four simultaneous subscriptions;
- up to 40 unique fields per subscription;
- fastest period 10000 microseconds (100 Hz);
- a requested period of zero means fastest available;
- slower periods are rounded up to the 100 Hz scheduler tick;
- the complete encoded telemetry sample, including header and quality mask,
  must fit the negotiated maximum payload; an unsendable field set is rejected
  during `SUBSCRIBE`.

Successful response type `0x20`, `FLAG_RESPONSE`:

```text
subscription_id     u16
accepted_period_us  u32
field_count         u16
field_ids           u32[field_count]
```

Unsolicited `TELEMETRY` type `0x22` normally has `FLAG_VOLATILE` and uses its
own device-side frame sequence:

```text
subscription_id   u16
sample_sequence   u16
timestamp_us      u64
quality_mask_len  u8
quality_mask      u8[quality_mask_len]
values            typed bytes in subscription field order
```

There are two quality bits per value, packed from the least-significant bits in
field order:

```text
00 VALID
01 STALE
10 INVALID
11 reserved
```

Do not confuse those wire codes with any internal firmware enum numeric value.
Use the subscription's descriptor types to find each packed value boundary.
Keep the newest sample for visualization rather than building an old backlog.

`UNSUBSCRIBE` request type `0x21` contains one `u16 subscription_id`; its
successful response echoes that ID.

## Assignment matrix and PWM routing

Routes connect a routable OUTPUT field to a type-compatible routable INPUT
field. There is one source per destination. Setting another source for an
already assigned destination replaces the old route. Routes are persisted in
AP_Param storage and survive reset. The matrix currently holds at most 16
routes.

Each stored slot has a payload-derived commit marker. Firmware synchronously
invalidates the marker, writes source/destination/flags, then writes the final
marker. Startup rejects absent or mismatched markers, so losing power during a
replacement can leave the previous genuine route or no route but cannot create
a mixed, never-requested route. A successful response still means the route was
accepted into the live matrix; it is not a promise that an immediate power cut
will preserve the new route.

Route slots created by firmware predating commit markers have no trustworthy
transaction boundary and are deliberately ignored after upgrade. Recreate
those routes once; subsequently committed routes retain normal reset survival.

### ROUTE_LIST

Request type `0x40` after `HELLO`:

```text
cursor       u32
max_records  u16
```

Response type `0x43`:

```text
next_cursor  u32
count        u16
repeat count times:
    route_id             u32
    source_output_id     u32
    destination_input_id u32
    flags                u16
```

Continue until `next_cursor == 0`; `FLAG_MORE` is set when more remain.

### ROUTE_SET

Request type `0x41` after `HELLO`:

```text
source_output_id      u32
destination_input_id  u32
flags                 u16 = 0
```

The response is one `ROUTES` record containing the accepted route. Route flags
other than zero are not supported now.

### ROUTE_DELETE

Request type `0x42` after `HELLO`:

```text
route_id  u32
```

The successful response is an empty `ROUTES` page.

### Minimal working routes

Serial receiver channel 1 to physical PWM output 1:

```text
source path       component/rcin/0/output/channel_1
source ID         0x04D2A7EE
destination path  component/pwm_output/0/input/channel_1
destination ID    0xF2E2CD61
flags             0
```

GPIO-captured PWM input 1 to physical PWM output 2:

```text
source path       component/pwm_input/0/output/channel_1
source ID         0xEAB60715
destination path  component/pwm_output/0/input/channel_2
destination ID    0xEFE2C8A8
flags             0
```

For serial RC, configure `uart_port` and optionally `protocol_mask`, reboot if
the UART changed, then create the route. Default receiver UART index is 1.

For direct PWM input on `revo-mini`, set a channel pin and reboot before routing
it. GPIO 50..55 correspond to PWM connector positions 1..6. A connector pin
configured as PWM input is reserved and cannot also be a PWM output. For the
second example, setting input 1 to GPIO 50 consumes connector position 1, while
connector position 2 remains available as the destination.

The firmware rejects nonexistent outputs, input/output pin conflicts, wrong
directions, non-routable fields, type mismatches, nonzero flags, graph cycles,
and assignment-table overflow.

## Safety behavior the client must communicate

Vektor is power-on operational and currently has no vehicle arming layer. The
PWM module releases the HAL safety gate, then controls each channel locally:

- an unassigned output is disabled;
- an assigned output with a valid source produces PWM immediately;
- stale/invalid input disables the channel when `failsafe_us == 0`;
- stale/invalid input produces the configured failsafe pulse when nonzero;
- disconnecting the PC does not delete or suspend a persistent route.

Therefore the client must never create routes implicitly. Require an explicit
user action, show a warning before the first physical output route, and advise
testing with motors/actuators disconnected. Display whether each PWM output is
assigned and whether its source quality is valid, stale, or invalid.

## ERROR handling

`ERROR` type `0x04` is a correlated response:

```text
request_message_type  u8
error_code            u16
object_id             u32
detail                 str16
```

`str16` is a little-endian `u16` byte length followed by UTF-8 bytes. Branch on
the numeric code; use `detail` only for diagnostics.

Codes the initial client should handle explicitly:

```text
0x0001 UNKNOWN_MESSAGE
0x0002 BAD_VERSION
0x0003 BAD_LENGTH
0x0004 BAD_ID
0x0006 OUT_OF_RANGE
0x0007 READ_ONLY
0x0008 WRITE_ONLY
0x000A NOT_AVAILABLE
0x000D INVALID_STATE
0x000E INVALID_ROUTE
0x000F TYPE_MISMATCH
0x0010 TOO_MANY_ITEMS
0x0012 SEQUENCE_CONFLICT
0x0013 STORAGE_ERROR
0x001B SUBSCRIPTION_LIMIT
0x001F INTERNAL_ERROR
```

Malformed COBS, length, version, or CRC data is normally dropped silently
because no trustworthy request exists yet. A timeout must not automatically be
reported as a firmware error.

## Suggested client structure

Keep these layers separate:

1. `CdcTransport`: port enumeration, open/close, asynchronous bytes.
2. `FrameCodec`: incremental delimiter parser, COBS, length, CRC.
3. `RequestSession`: sequences, one outstanding request, timeout/retry,
   response correlation, reconnect.
4. `DescriptorStore`: components, fields, endpoints, timer groups, limits, and
   hash-keyed cache.
5. `ParameterService`: typed values, constraints, apply policy, persistence UI.
6. `RouteService`: routable field filtering, route paging, create/replace/delete.
7. `TelemetryService`: subscription state, packed values, quality, newest-sample
   delivery.

Do not put serial reads or request waits on the GUI thread. Do not hard-code a
board layout or field IDs into the main UI model.

## Acceptance checklist

The initial PC client is ready for current firmware when all of these pass:

- The canonical PING frame encodes and decodes byte-for-byte.
- Fragmented one-byte reads and several frames in one read both parse.
- Empty delimiters and a corrupt frame do not prevent the next valid frame.
- The client finds the CDC port, ignores a bootloader/non-Vektor port, and
  verifies the application through `HELLO`.
- `HELLO` parses the current reduced board and runtime limits.
- All implemented descriptor domains page to completion.
- The reduced board yields 7 components, 102 fields, and 21 readable
  parameters. `sys_protocol_baud` is omitted because this build uses USB.
- `GET_ALL_PARAMS` populates the editor using descriptor types and constraints.
- `SET` returns and displays the accepted value; reboot-policy fields are marked.
- Closing and reopening the client starts with a fresh `HELLO` and refreshes
  values/routes.
- `ROUTE_LIST` is correct when empty and when populated.
- With actuators disconnected, RCIN channel 1 can be deliberately routed to PWM
  output 1, replaced, listed, deleted, and shown after reconnect.
- A realtime subscription reports normalized RC/PWM values and their two-bit
  quality correctly.
- Known unimplemented services display `NOT_AVAILABLE` without crashing.

## Current limitations

- There is no file service, action service, or event stream yet.
- There are no enum-table descriptors yet.
- The VSP core is intentionally only a skeleton.
- Raw RC input and physical PWM output pulse widths are exposed as U16
  realtime fields; routable command fields remain normalized FLOAT32 values.
- The final H743 ChibiOS hardware definition is not available in this
  workspace, so no H743 capability record is compiled yet. Build and test the
  first PC integration against `revo-mini` and remain descriptor-driven for
  H743 later.
- USB access permissions/driver installation are an OS packaging concern. On
  Linux, provide an appropriate udev rule or user-group instructions rather
  than requiring the application to run as root.
