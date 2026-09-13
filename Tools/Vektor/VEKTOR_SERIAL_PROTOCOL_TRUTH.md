# Vektor Serial Protocol — One Truth

**Canonical working specification for Vektor Core Evo and Vektor Core Reduced**  
**Protocol family:** Vektor Protocol / VSP (Vektor Serial Protocol)  
**Status:** implementation baseline  
**Wire major version:** 1  
**Primary transport:** native USB CDC byte stream  
**Optional transport:** UART byte stream  
**Application relationship:** transport adapter over the common Vektor component model  
**Companion protocol:** MAVLink may coexist as a parallel adapter over the same application state

---

## 0. Authority and purpose

This document is the single working source of truth for the native Vektor serial protocol.

It supersedes the earlier `VEKTOR_PROTOCOL(1).md` draft for protocol implementation work. It also reconciles the protocol architecture against `VEKTOR_CORE_EVO_TRUTH_BASE.md`, the common firmware architecture notes, the two-platform project brief, and the verified hardware differences between the H743 full board and F405 reduced board.

This document intentionally distinguishes protocol truth from hardware truth:

- the protocol defines **how a client discovers and operates a Vektor device**;
- the board definition defines **what physical resources actually exist**;
- the Vektor application defines **what logical components and services exist**;
- the configurator must render what the device reports rather than infer capabilities from a board name.

A protocol capability is not proof that every board has the underlying hardware.

Examples:

- the protocol can describe CAN endpoints, but the reduced F405 board currently has no verified CAN endpoint;
- the protocol can describe file storage, but the current H743 schematic has no verified SD device;
- the protocol can describe Flex modes, but a physical Flex row may only advertise modes that its board definition genuinely supports.

### 0.1 Status words used in this document

- **CANONICAL** — frozen for protocol v1 implementation unless this document is deliberately revised.
- **RUNTIME** — value/capability supplied by the device at runtime and never assumed by the client.
- **OPTIONAL** — valid v1 functionality that a given firmware build or board may omit and therefore must not advertise.
- **RESERVED** — encoding space held for future use and required to be ignored or rejected as specified.
- **OPEN-PRODUCT** — product policy that remains outside the wire protocol itself.

The wire layout, CRC, message identifiers, primitive type identifiers, sequence rules, and framing rules below are **CANONICAL** for this working baseline.

---

# Part I — Design contract

## 1. What Vektor Protocol is

Vektor Protocol is a compact binary request/response and streaming protocol for:

- connection and version negotiation;
- board and firmware identification;
- hardware capability discovery;
- application component discovery;
- typed parameter reads and writes;
- logical signal routing;
- live observables;
- high-rate realtime subscriptions;
- one-shot actions;
- asynchronous events;
- bounded logical file access;
- diagnostics and service functions.

It is designed for a PC configurator talking to a Vektor controller over a byte stream.

The protocol must stay application-neutral. It may transport descriptors and values for VSP/VRS components, but the framing layer itself contains no VSP-specific rules.

## 2. What Vektor Protocol is not

It is not:

- the parameter database;
- a replacement for `AP_Param`;
- a PLC runtime;
- a hardware abstraction layer;
- a filesystem exposing arbitrary MCU paths;
- a bootloader protocol by definition;
- MAVLink with different message names;
- a board-name lookup table;
- a guaranteed reliable transport by itself;
- a place for control-law logic.

The Vektor application owns component behavior. The protocol only discovers, configures, observes, and invokes it.

## 3. Architectural invariants

The following are protocol-level invariants:

1. One protocol is used by both the H743 full board and the F405 reduced board.
2. Board differences are described at runtime.
3. Hardware capability and application component inventory are separate descriptor domains.
4. `AP_Param` remains the authoritative persistent parameter store where parameters are backed by ArduPilot parameter infrastructure.
5. A parameter changed over Vektor Protocol and the equivalent parameter changed over MAVLink must reach the same underlying application state.
6. Realtime visualization must never block control/configuration traffic.
7. Old realtime samples may be discarded when newer samples exist.
8. File traffic is background traffic.
9. Long-running actions are represented as actions plus events/observables, not as magic parameter writes.
10. The configurator must not expose hardware the device did not advertise.

---

# Part II — Transport and framing

## 4. Supported byte-stream transports

### 4.1 USB CDC

**CANONICAL primary transport.**

Native USB CDC is the preferred configurator/service connection on both current boards.

USB packet boundaries have no protocol meaning. The receiver must parse Vektor frames from an arbitrary byte stream.

### 4.2 UART

**OPTIONAL transport.**

The same framing can be carried on a UART endpoint if firmware exposes Vektor Protocol there.

UART electrical parameters such as:

- baud rate;
- inversion;
- parity;
- stop bits;
- half/full duplex;

are board/endpoint configuration and are not encoded in the generic packet header.

### 4.3 Transport independence

No message may rely on:

- USB transfer packet size;
- one read call containing exactly one frame;
- one UART DMA block containing exactly one frame;
- host OS serial buffering behavior.

The parser must work correctly when a frame arrives one byte at a time or when several frames arrive in one read.

---

## 5. Frame delimiter and COBS

### 5.1 Canonical stream representation

Each decoded Vektor frame is COBS encoded and terminated by `0x00`.

Canonical continuous stream form:

```text
COBS(frame_0) 00 COBS(frame_1) 00 COBS(frame_2) 00 ...
```

A sender **may** emit an additional leading `0x00` when a connection is opened or a parser is reset:

```text
00 COBS(frame) 00
```

Receivers must treat empty delimiter-to-delimiter spans as no-ops.

This preserves the original draft's zero-delimited concept while avoiding any dependency on a mandatory leading delimiter for every packet.

### 5.2 COBS rules

- The COBS encoded body contains no zero bytes.
- `0x00` is exclusively the stream frame delimiter.
- A malformed COBS block is discarded.
- Parsing resumes at the next `0x00` delimiter.
- A malformed frame must never poison parsing of later frames.

---

## 6. Decoded frame layout

**CANONICAL v1 frame:**

```text
Offset  Size  Field
0       1     version
1       1     message_type
2       1     flags
3       2     sequence
5       2     payload_len
7       N     payload
7+N     4     crc32
```

C definition equivalent:

```c
struct VektorFrameV1 {
    uint8_t  version;
    uint8_t  message_type;
    uint8_t  flags;
    uint16_t sequence_le;
    uint16_t payload_len_le;
    uint8_t  payload[payload_len];
    uint32_t crc32_le;
};
```

Header size before payload is **7 bytes**.

Total decoded frame size is:

```text
11 + payload_len
```

### 6.1 Byte order

All multi-byte integer and floating-point fields are little-endian.

IEEE-754 binary32/binary64 representation is used for `FLOAT32` and `FLOAT64`.

### 6.2 Payload length

`payload_len` is the number of decoded payload bytes only. It does not include:

- the 7-byte header;
- the CRC;
- COBS overhead;
- zero delimiters.

The protocol field permits a theoretical payload up to 65535 bytes, but a device is expected to advertise a much smaller runtime maximum.

A client must obey the device's advertised `max_payload` and `max_file_chunk` values.

---

## 7. CRC-32

### 7.1 Canonical algorithm

Vektor Protocol v1 uses **CRC-32/ISO-HDLC**, commonly called standard CRC-32 or Ethernet/zlib CRC-32.

Parameters:

```text
width      = 32
poly       = 0x04C11DB7
refin      = true
refout     = true
init       = 0xFFFFFFFF
xorout     = 0xFFFFFFFF
check      = 0xCBF43926 for ASCII "123456789"
```

The reflected implementation polynomial is commonly represented as `0xEDB88320`.

### 7.2 Covered bytes

CRC is calculated over the decoded bytes from `version` through the last payload byte.

The CRC field itself is excluded.

The 32-bit CRC is appended little-endian.

### 7.3 Bad CRC behavior

A frame with a bad CRC is discarded.

For a frame whose request sequence can be trusted only after CRC validation, the device must **not** attempt to send a request-correlated error response.

CRC errors may increment a diagnostic counter observable through the normal component/diagnostic system.

---

## 8. Protocol version

The frame-header `version` is the wire **major version**.

For this specification:

```text
version = 1
```

A receiver that cannot parse the major version must discard the frame.

Minor feature evolution inside major version 1 is negotiated through:

- `HELLO` metadata;
- descriptor contents;
- capability bits;
- message availability.

Breaking changes to the frame header or existing canonical field encodings require a new major version.

---

# Part III — Sequence, flags, reliability, and queues

## 9. Sequence number semantics

`sequence` is a 16-bit unsigned value and wraps modulo 65536.

### 9.1 Host requests

The host assigns a sequence value to every request for which a correlated response is expected.

The host should increment its request sequence monotonically, skipping no value requirement other than avoiding confusion with still-outstanding requests.

### 9.2 Responses

A direct response copies the request's sequence value and sets `FLAG_RESPONSE`.

### 9.3 Unsolicited device traffic

Unsolicited `TELEMETRY` and `EVENT` packets use a device-side transmit sequence counter.

Their sequence is useful for loss diagnostics but is not a request correlation token.

### 9.4 Wrap

Consumers must compare sequence values modulo 65536 and must not assume a process lifetime shorter than one wrap.

---

## 10. Flags byte

Canonical v1 flag bits:

```text
bit 0  FLAG_RESPONSE   = 0x01
bit 1  FLAG_MORE       = 0x02
bit 2  FLAG_URGENT     = 0x04
bit 3  FLAG_VOLATILE   = 0x08
bit 4  RESERVED        = 0x10
bit 5  RESERVED        = 0x20
bit 6  RESERVED        = 0x40
bit 7  RESERVED        = 0x80
```

### 10.1 `FLAG_RESPONSE`

Marks a direct response to a request.

The response sequence must equal the request sequence.

### 10.2 `FLAG_MORE`

The sender has more response chunks belonging to the same logical operation.

Used for paged or chunked response streams such as:

- `DESCRIBE`;
- `VALUES` bulk results;
- `ROUTES`;
- `FILE_LIST`.

Each physical frame remains independently CRC protected.

### 10.3 `FLAG_URGENT`

Marks an event whose transmit scheduling may preempt ordinary realtime/background traffic.

It does **not** bypass protocol validation or control-loop safety.

Normal examples are severe runtime fault events, not routine telemetry.

### 10.4 `FLAG_VOLATILE`

Marks information where freshness is more important than delivery and replacement is allowed before transmission.

`TELEMETRY` normally uses this flag.

A receiver need not treat the flag as a command. It is primarily a traffic-class hint and diagnostic signal.

### 10.5 Reserved bits

Senders must transmit reserved bits as zero.

Receivers must ignore unknown flag bits within a supported major version unless a future capability explicitly changes that rule.

---

## 11. Request/response reliability

### 11.1 USB CDC

USB supplies reliable ordered byte transport at the USB layer, but disconnects and process restarts still exist.

Vektor request semantics remain explicit so client code behaves the same on USB and UART.

### 11.2 UART

UART provides ordered bytes but no delivery guarantee.

For request/response control operations:

1. host sends request with a sequence number;
2. device validates and processes request;
3. device returns a response with the same sequence and `FLAG_RESPONSE`;
4. if the host times out, it may resend the **identical** request with the same sequence;
5. the device must detect recent duplicate control requests and replay the previous response without repeating a non-idempotent side effect.

### 11.3 Duplicate suppression

Device implementations must maintain a small recent-request cache sufficient for retransmission behavior.

Cache key:

```text
message_type
sequence
request_payload_crc32 or equivalent strong request fingerprint
```

If a duplicate key is received, replay the cached response.

If the same `(message_type, sequence)` arrives with a different payload fingerprint while the old entry is still in the duplicate window, respond with `SEQUENCE_CONFLICT`.

This is especially important for:

- `ACTION`;
- `SET`;
- `SET_MANY`;
- route changes;
- file writes.

### 11.4 Timeouts

Timeout policy is a client implementation detail and may differ by transport.

The protocol does not encode a universal millisecond timeout.

Long operations must acknowledge quickly and then progress asynchronously rather than forcing the request timeout to become very long.

---

## 12. Traffic classes and transmit priority

Canonical transmit priority:

```text
1. direct control responses and ERROR
2. urgent/important EVENT
3. fast realtime control/attitude TELEMETRY
4. ordinary TELEMETRY
5. descriptor/bulk responses that can be paged
6. file transfer/background traffic
```

### 12.1 Realtime queue rule

For realtime subscriptions, **newest value wins**.

If an unsent sample for a subscription is already queued and a newer sample is generated, firmware may replace the older sample instead of adding another packet.

The firmware must not build an ever-growing visualization backlog.

### 12.2 Control queue rule

Control replies are not replaceable.

They must either be transmitted or the transport/session must fail explicitly.

### 12.3 File queue rule

File operations must yield to control and realtime traffic.

Storage access itself must also be scheduled so that SD/dataflash latency cannot stall the control loop.

---

# Part IV — Primitive encodings

## 13. Primitive type identifiers

Canonical v1 type IDs:

```text
0x00  INVALID
0x01  BOOL
0x02  U8
0x03  I8
0x04  U16
0x05  I16
0x06  U32
0x07  I32
0x08  U64
0x09  I64
0x0A  FLOAT32
0x0B  FLOAT64
0x0C  ENUM
0x0D  STRING
0x0E  BLOB
0x0F  VECTOR2F
0x10  VECTOR3F
0x11  QUATERNIONF
```

Type IDs `0x12..0x7F` are reserved for future canonical types.

Type IDs `0x80..0xFF` are reserved for experimental/private builds and must not appear in a production schema unless explicitly negotiated.

## 14. Primitive binary representation

```text
BOOL         u8, 0=false, 1=true; other values invalid
U8           u8
I8           int8 two's complement
U16          little-endian u16
I16          little-endian int16
U32          little-endian u32
I32          little-endian int32
U64          little-endian u64
I64          little-endian int64
FLOAT32      IEEE-754 binary32 little-endian
FLOAT64      IEEE-754 binary64 little-endian
ENUM         little-endian i32
VECTOR2F     2 x FLOAT32 in declared axis order
VECTOR3F     3 x FLOAT32 in declared axis order
QUATERNIONF  4 x FLOAT32 in w,x,y,z order
```

### 14.1 String

Protocol strings are UTF-8 without a NUL terminator.

Generic `STRING` typed values use:

```text
length  u16
bytes   u8[length]
```

### 14.2 Blob

Generic `BLOB` typed values use:

```text
length  u16
bytes   u8[length]
```

Large file data does not use a nested `BLOB`; `FILE_READ` and `FILE_WRITE` carry their data directly in the message payload to avoid redundant length layers.

### 14.3 Length-prefixed protocol strings

Descriptor fields whose maximum is naturally small may use `str8`:

```text
length  u8
bytes   UTF-8[length]
```

Longer diagnostics, file names, and similar protocol text may use `str16`:

```text
length  u16
bytes   UTF-8[length]
```

Neither form includes a NUL terminator. This document states `str8` or `str16` explicitly wherever one of these structural string forms is used.

### 14.4 Typed value encoding

Where the type is **not already known from context**:

```text
type_id      u8
value        encoded according to type_id
```

Where a descriptor/subscription already fixes the type, the type byte is omitted.

This is important for compact realtime packets.

---

# Part V — Stable object IDs

## 15. Stable 32-bit IDs

Components, fields, ports, observables, actions, endpoint objects, and routes are identified by stable unsigned 32-bit IDs.

`0x00000000` is reserved as `INVALID_ID` and must never identify a real object.

## 16. Canonical generated-ID algorithm

Default generated IDs use **FNV-1a 32-bit** over a canonical UTF-8 path.

FNV parameters:

```text
offset basis = 0x811C9DC5
prime        = 0x01000193
```

### 16.1 Canonical paths

Examples:

```text
component/vsp/1
component/vsp/1/input/x
component/vsp/1/input/y
component/vsp/1/output/servo_a
component/vsp/1/parameter/vrs_gain
component/attitude/0/observable/quaternion
hw/pwm_bank/0/channel/1
hw/flex/3
hw/uart/1
hw/can/0
```

Canonical paths use:

- lowercase ASCII names;
- `/` separators;
- no spaces;
- no board display labels;
- stable semantic names rather than GPIO names.

### 16.2 Collision rule

The firmware build must detect duplicate generated IDs and fail the build/schema-generation step.

Silent collision resolution is forbidden.

### 16.3 Rename stability

Renaming a canonical path normally changes its generated ID.

If an object must retain identity across a user-facing/internal rename, its source descriptor may provide an explicit frozen numeric ID override.

This is preferable to inventing a runtime alias mechanism in the wire protocol.

---

# Part VI — Message namespace

## 17. Canonical message type IDs

```text
# Session / discovery
0x01  HELLO
0x02  DESCRIBE
0x03  PING
0x04  ERROR

# Parameters / generic values
0x10  GET
0x11  SET
0x12  VALUE
0x13  GET_MANY
0x14  SET_MANY
0x15  VALUES
0x16  GET_ALL_PARAMS

# Realtime subscriptions
0x20  SUBSCRIBE
0x21  UNSUBSCRIBE
0x22  TELEMETRY

# Actions and asynchronous events
0x30  ACTION
0x31  ACTION_ACK
0x32  EVENT

# Routing
0x40  ROUTE_LIST
0x41  ROUTE_SET
0x42  ROUTE_DELETE
0x43  ROUTES

# Logical file service
0x50  FILE_LIST
0x51  FILE_STAT
0x52  FILE_OPEN
0x53  FILE_READ
0x54  FILE_WRITE
0x55  FILE_CLOSE
0x56  FILE_DELETE
0x57  FILE_RENAME

0x58..0x7F  RESERVED canonical expansion
0x80..0xEF  RESERVED future protocol families
0xF0..0xFF  experimental/private; never in production release schemas
```

`FILE_DELETE` and `FILE_RENAME` are valid protocol messages but **OPTIONAL** services. A board/build that does not implement them must return `NOT_AVAILABLE` and must not advertise the corresponding capability.

---

# Part VII — Session and discovery

## 18. HELLO

`HELLO` is the first request on a new logical connection.

A client must not rely on cached board data until the current device has replied to `HELLO`.

### 18.1 HELLO request

Message type:

```text
HELLO = 0x01
flags = 0
```

Payload:

```text
client_min_major       u8
client_max_major       u8
client_proto_minor     u16
client_feature_flags   u32
client_nonce           u32
```

For the current client:

```text
client_min_major <= 1 <= client_max_major
```

`client_nonce` is not security. It is an opaque connection token useful for diagnostics and stale-response detection.

### 18.2 HELLO response

Same message type, same sequence, `FLAG_RESPONSE` set.

Payload:

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

### 18.3 Meaning of identity fields

`firmware_version`  
Human-readable build version. It is not used as a capability decision key.

`hardware_type`  
Human-readable family such as `Vektor Core Evo` or `Vektor Core Reduced`.

`hardware_revision`  
Physical board revision string defined by the board target.

`board_id`  
Stable numeric board-target/revision identity assigned by the Vektor project.

`schema_hash`  
Hash of application descriptor schema: components, fields, actions, types, and semantic descriptor content that affects client interpretation.

`capability_hash`  
Hash of hardware/runtime capability descriptors exposed by this build/board.

`device_id`  
Stable device identity where available. It may be derived from MCU unique ID through a project-defined deterministic hash. It is not an authentication credential.

### 18.4 Hash algorithm

Canonical v1 descriptor-cache hashes are **FNV-1a 64-bit** over the canonical serialized descriptor stream used by firmware schema generation.

Parameters:

```text
offset basis = 0xCBF29CE484222325
prime        = 0x100000001B3
```

The serializer must be deterministic. Descriptor records are sorted by numeric ID before hashing.

If an implementation cannot produce a stable cache hash, it must return zero. A zero hash means "do not cache by this hash."

---

## 19. Coarse capability flags

`capability_flags` is a cheap summary only. Structured descriptors remain authoritative.

Canonical v1 bits:

```text
bit 0   CAP_USB
bit 1   CAP_UART_VEKTOR
bit 2   CAP_PWM
bit 3   CAP_FLEX_IO
bit 4   CAP_CAN
bit 5   CAP_IMU
bit 6   CAP_COMPASS
bit 7   CAP_ADC
bit 8   CAP_STORAGE
bit 9   CAP_FILE_SERVICE
bit 10  CAP_MAVLINK
bit 11  CAP_ROUTING
bit 12  CAP_ACTIONS
bit 13  CAP_REALTIME
bit 14  CAP_SD
bit 15  CAP_DATAFLASH
bit 16  CAP_GNSS_ENDPOINT
bits 17..63 reserved
```

A coarse flag never expands a structured capability.

For example, `CAP_FLEX_IO` means Flex descriptors exist. It does not mean every Flex port supports every possible mode.

---

## 20. DESCRIBE

`DESCRIBE` retrieves runtime descriptors.

Descriptors are paged so neither side needs one very large packet.

### 20.1 Descriptor domains

Canonical domains:

```text
0x01 BOARD
0x02 COMPONENT
0x03 FIELD
0x04 ENDPOINT
0x05 TIMER_GROUP
0x06 STORAGE_AREA
0x07 RUNTIME_LIMIT
0x08 ENUM_TABLE
0x09 EVENT_TYPE
0x0A ACTION_SCHEMA
```

### 20.2 DESCRIBE request

```text
domain          u8
cursor          u32
max_records     u16
```

`cursor = 0` starts enumeration.

`max_records = 0` means "use device default page size."

### 20.3 DESCRIBE response envelope

```text
domain          u8
next_cursor     u32
record_count    u16
records[]
```

Each record is length prefixed:

```text
record_len      u16
record_bytes    u8[record_len]
```

If `next_cursor != 0`, more records remain.

`FLAG_MORE` should also be set when more records remain.

The client continues using `next_cursor` until the response returns `next_cursor = 0`.

### 20.4 Forward compatibility

Clients must ignore descriptor record bytes beyond the known minimum schema for a record version.

Every descriptor record starts with:

```text
record_version  u8
```

A client that does not understand a record version must skip that record rather than misparse it.

---

# Part VIII — Descriptor truth model

## 21. Board descriptor

Minimum BOARD record v1:

```text
record_version      u8 = 1
board_id            u32
hardware_type       str8
hardware_revision   str8
display_name        str8
```

Optional presentation metadata may be added later, but client functionality must not depend on artwork or a hard-coded board layout.

## 22. Component descriptor

Minimum COMPONENT record v1:

```text
record_version       u8 = 1
component_id         u32
component_type_id    u32
instance             u16
flags                u32
name                 str8
display_name         str8
```

`component_type_id` identifies semantic component class independent of instance.

## 23. Field descriptor

Canonical field kinds:

```text
0x01 INPUT
0x02 OUTPUT
0x03 PARAMETER
0x04 OBSERVABLE
```

Actions are described separately through ACTION_SCHEMA.

Minimum FIELD record v1:

```text
record_version       u8 = 1
field_id             u32
owner_component_id   u32
field_kind           u8
type_id              u8
flags                 u32
name                  str8
display_name          str8
units                 str8
```

If field flags indicate bounds/default metadata, those typed values follow in this fixed order:

```text
[min typed value]
[max typed value]
[default typed value]
```

The type tag is omitted because `type_id` is already known.

### 23.1 Field flags

Canonical v1 field flags:

```text
bit 0  FIELD_READABLE
bit 1  FIELD_WRITABLE
bit 2  FIELD_PERSISTENT
bit 3  FIELD_REALTIME
bit 4  FIELD_HAS_MIN
bit 5  FIELD_HAS_MAX
bit 6  FIELD_HAS_DEFAULT
bit 7  FIELD_APPLY_LIVE
bit 8  FIELD_APPLY_RECONFIGURE
bit 9  FIELD_APPLY_REBOOT
bit 10 FIELD_ROUTABLE
bit 11 FIELD_HIDDEN_NORMAL_UI
bit 12 FIELD_DEVELOPER_ONLY
bits 13..31 reserved
```

Exactly one parameter apply policy should normally be set for writable persistent parameters.

## 24. Hardware endpoint descriptor

Endpoint kind IDs:

```text
0x01 PWM_BANK
0x02 PWM_CHANNEL
0x03 FLEX_PORT
0x04 UART
0x05 CAN
0x06 ADC
0x07 RPM_CAPTURE
0x08 DIGITAL_IO
0x09 USB
0x0A SENSOR
0x0B STORAGE
0x0C GNSS
```

Minimum ENDPOINT record v1:

```text
record_version       u8 = 1
endpoint_id          u32
endpoint_kind        u8
parent_endpoint_id   u32
flags                u64
name                 str8
display_name         str8
```

Kind-specific bytes may follow the minimum record.

The `record_len` envelope allows clients to skip unknown extensions safely.

### 24.1 Flex mode flags

A Flex endpoint advertises only electrically and firmware-supported modes.

Canonical Flex mode bits:

```text
bit 0  FLEX_PWM_INPUT
bit 1  FLEX_PWM_OUTPUT
bit 2  FLEX_RPM_CAPTURE
bit 3  FLEX_ADC_INPUT
bit 4  FLEX_DIGITAL_INPUT
bit 5  FLEX_DIGITAL_OUTPUT
```

A client must not infer `FLEX_ADC_INPUT` merely because another board's Flex port supports ADC.

## 25. Timer group descriptor

Used so the configurator can explain shared PWM frame-rate constraints without exposing STM32 timer names.

Minimum TIMER_GROUP record v1:

```text
record_version       u8 = 1
group_id             u32
member_count         u8
member_endpoint_ids  u32[member_count]
rate_count           u8
supported_rates_hz   u16[rate_count]
current_rate_hz      u16
```

## 26. Storage area descriptor

Logical areas are not raw host filesystem paths.

Canonical storage area IDs:

```text
0x01 CONFIG
0x02 LOGS
0x03 CALIBRATION
0x04 USER
0x05 UPDATE
```

Minimum STORAGE_AREA record v1:

```text
record_version     u8 = 1
area_id            u8
flags              u32
capacity_bytes     u64
free_bytes         u64
name               str8
```

Area flags may include:

```text
READ
WRITE
APPEND
DELETE
RENAME
DIRECTORY_LIST
ATOMIC_REPLACE
```

A board without a verified SD card must not advertise an SD-backed storage capability simply because the protocol defines one.

## 27. Runtime limit descriptor

Runtime limits exist so the client never hard-codes H743 or F405 performance assumptions.

Minimum RUNTIME_LIMIT record v1:

```text
record_version       u8 = 1
limit_id             u32
capability_flags     u64
max_payload          u16
max_file_chunk       u16
max_subscriptions    u16
max_realtime_hz      u16
control_update_hz    u16
attitude_update_hz   u16
```

The protocol-service record uses the canonical path
`runtime_limit/protocol/0` for `limit_id`. A zero limit means that the
corresponding optional service is not currently available.

The values returned by `HELLO` are the fast connection summary. RUNTIME_LIMIT descriptors may carry additional service-specific details.

---

# Part IX — Parameters and generic values

## 28. GET

Request payload:

```text
field_id  u32
```

Valid only for readable parameters or observables that support explicit query.

Successful response is `VALUE` with matching request sequence.

## 29. VALUE

Response payload:

```text
field_id    u32
type_id     u8
value       typed value payload
```

The returned `type_id` must match the field descriptor.

For `SET`, `VALUE` returns the **accepted/applied value**, which may differ from the request if canonical firmware policy performs quantization or normalization.

Out-of-range values should normally be rejected rather than silently clamped unless the field descriptor/application semantics explicitly define clamping.

## 30. SET

Request payload:

```text
field_id    u32
type_id     u8
value       typed value payload
```

Firmware behavior:

1. resolve field;
2. verify writable type;
3. validate bounds/semantics;
4. apply through `ParameterService` or appropriate state owner;
5. persist when required;
6. notify/reconfigure component as required;
7. return `VALUE` with the accepted value.

If a change requires reboot or reconfiguration, the accepted value is still returned and an appropriate status may also be indicated by event/observable policy.

## 31. GET_MANY

Request:

```text
count       u16
field_id    u32[count]
```

Response uses `VALUES`.

## 32. SET_MANY

Request:

```text
count       u16
repeat count times:
    field_id    u32
    type_id     u8
    value       typed value
```

### 32.1 Atomicity

`SET_MANY` is **validation-atomic** by default:

- validate the complete request first;
- if any entry is invalid, apply none;
- return `ERROR` identifying the failing field when possible.

After validation, application/reconfiguration callbacks may occur sequentially but must observe the final accepted parameter set, not a partially rejected request.

This makes configurator profile updates predictable.

## 33. VALUES

Payload:

```text
count       u16
repeat count times:
    field_id    u32
    type_id     u8
    value       typed value
```

A large result may be paged with `FLAG_MORE`.

## 34. GET_ALL_PARAMS

Request payload may be empty.

Response is one or more `VALUES` frames containing fields whose descriptors are `PARAMETER` and `READABLE`.

The configurator should prefer cached descriptors plus bulk values instead of hundreds of individual `GET` round trips.

---

# Part X — Logical routing

## 35. Routing model

A route connects one routable output to one routable input:

```text
source_output_id
        ->
destination_input_id
```

Both IDs are field IDs whose descriptors include `FIELD_ROUTABLE` and appropriate INPUT/OUTPUT kinds.

The routing service is transport-independent.

## 36. Route ID

A stored route receives a stable route ID generated from:

```text
route/<source_output_id_hex>/<destination_input_id_hex>
```

using the canonical FNV-1a 32-bit algorithm, unless firmware provides an explicit route ID.

## 37. ROUTE_LIST

Request:

```text
cursor        u32
max_records   u16
```

Response message type `ROUTES`:

```text
next_cursor   u32
count         u16
repeat:
    route_id             u32
    source_output_id     u32
    destination_input_id u32
    flags                u16
```

`FLAG_MORE` is set if `next_cursor != 0`.

## 38. ROUTE_SET

Request:

```text
source_output_id       u32
destination_input_id   u32
flags                  u16
```

Firmware validates:

- source exists and is routable OUTPUT;
- destination exists and is routable INPUT;
- signal types are compatible;
- destination route multiplicity policy permits the route;
- board/application capability exists;
- the resulting graph does not violate a prohibited cycle rule.

Successful response is `ROUTES` containing the accepted single route.

## 39. ROUTE_DELETE

Request:

```text
route_id   u32
```

Successful response may be an empty `ROUTES` response with matching sequence.

### 39.1 User-interface complexity rule

The protocol can represent routes generically, but the configurator should preserve the product rule that normal user configurations should not require long PLC-style chains.

That is a UI/application policy, not a wire-format limitation.

---

# Part XI — Realtime subscriptions

## 40. SUBSCRIBE

A subscription is a compact ordered set of observable fields with a requested rate.

Request:

```text
requested_period_us   u32
field_count           u16
field_id              u32[field_count]
```

`requested_period_us = 0` means "fastest rate the firmware is willing to provide for this set."

### 40.1 Validation

Every requested field must:

- exist;
- be readable;
- be suitable for subscription;
- have a stable descriptor type.

If one field is invalid, the request is rejected as a whole.

### 40.2 SUBSCRIBE response

Same message type with `FLAG_RESPONSE`:

```text
subscription_id       u16
accepted_period_us    u32
field_count           u16
field_id              u32[field_count]
```

`subscription_id = 0` is invalid/reserved.

The accepted period may be slower than requested.

The client must use the accepted value rather than infer a rate from board identity.

## 41. TELEMETRY

Unsolicited device-to-host packet.

Typical flags:

```text
FLAG_VOLATILE
```

Payload:

```text
subscription_id    u16
sample_sequence    u16
timestamp_us       u64
quality_mask_len   u8
quality_mask       u8[quality_mask_len]
values             packed field values in subscription order
```

### 41.1 Timestamp

`timestamp_us` is monotonic device time in microseconds from the firmware monotonic clock epoch.

It is **sample/estimator time**, not USB transmit time.

It wraps only on `u64`, which is effectively irrelevant for product uptime.

No UTC meaning is implied.

### 41.2 Quality state

Every routed/live signal conceptually has:

```text
VALID
STALE
INVALID
```

Telemetry packs quality as two bits per field:

```text
00 VALID
01 STALE
10 INVALID
11 RESERVED
```

Fields are packed from least-significant bits upward in subscription field order.

`quality_mask_len` must be at least `ceil(field_count * 2 / 8)`.

### 41.3 Values

Types are known from descriptors and subscription order, so neither field IDs nor type IDs repeat in normal `TELEMETRY` samples.

This is the main bandwidth optimization for high-rate streams.

### 41.4 Fast attitude example

A typical attitude subscription requests:

```text
Attitude.quaternion  QUATERNIONF
Attitude.gyro        VECTOR3F
```

The device may accept, for example, 200 Hz on one board and 500 Hz on another.

The protocol does not promise either value globally.

### 41.5 Host behavior

For visualization:

- keep the newest sample;
- do not intentionally replay a stale backlog;
- render at the GUI's own display cadence;
- use `timestamp_us` when interpolation/prediction is implemented.

## 42. UNSUBSCRIBE

Request:

```text
subscription_id  u16
```

Successful response is the same message type with `FLAG_RESPONSE` and the same `subscription_id`.

A disconnect destroys all session subscriptions.

---

# Part XII — Actions and events

## 43. ACTION

Actions represent one-shot operations.

Examples include:

- VSP calibration;
- zero actuator feedback;
- reset statistics;
- start identification;
- compass calibration;
- reload/reconfigure;
- reboot;
- reboot to bootloader.

Request:

```text
component_id     u32
action_id        u32
argument_count   u8
repeat:
    argument_id  u16
    type_id      u8
    value        typed value
```

Arguments must match the ACTION_SCHEMA descriptor.

## 44. ACTION_ACK

Direct response:

```text
component_id     u32
action_id        u32
status           u8
operation_id     u32
```

Status:

```text
0 ACCEPTED_COMPLETE
1 ACCEPTED_RUNNING
2 REJECTED
```

`operation_id = 0` when no asynchronous operation follows.

For `ACCEPTED_RUNNING`, `operation_id` must be nonzero.

A long-running action must return `ACTION_ACK` quickly. It must not block the serial request waiting for calibration or identification to finish.

## 45. EVENT

Unsolicited device-to-host message.

Payload:

```text
event_type_id      u32
source_id          u32
operation_id       u32
timestamp_us       u64
severity           u8
argument_count     u8
arguments...
```

Severity:

```text
0 INFO
1 NOTICE
2 WARNING
3 ERROR
4 CRITICAL
```

Examples:

- component connected/disconnected;
- sensor became stale/invalid;
- CAN bus-off;
- calibration progress/completion/failure;
- configuration reloaded;
- parameter rejected by asynchronous reconfiguration;
- file transfer completed/failed.

Routine continuous values belong in observables/telemetry, not events.

---

# Part XIII — Logical file service

## 46. File-service principles

The file service exposes **logical areas**, not arbitrary MCU addresses and not unrestricted absolute filesystem paths.

Backends may be:

- SD card;
- SPI dataflash;
- internal flash;
- future external storage.

The same protocol is used regardless of backend.

A board advertises only the logical areas it actually implements.

## 47. Path encoding

Within a logical area, paths are UTF-8, relative, `/` separated, and encoded as:

```text
path_len  u16
path      u8[path_len]
```

Rules:

- no leading `/`;
- no `..` path element;
- no NUL;
- maximum path length is implementation-defined and must fit the advertised payload limit;
- paths are case-sensitive at protocol level even if a backend is not.

## 48. FILE_LIST

Request:

```text
area_id       u8
cursor        u32
max_entries   u16
path          path encoding; empty path means area root
```

Response:

```text
area_id        u8
next_cursor    u32
entry_count    u16
entries...
```

Entry:

```text
flags           u16
size            u64
modified_us     u64
name            str16
```

`modified_us = 0` when unavailable.

`FLAG_MORE` is used if `next_cursor != 0`.

## 49. FILE_STAT

Request:

```text
area_id    u8
path       path
```

Response:

```text
area_id       u8
flags         u16
size          u64
modified_us   u64
content_crc32 u32
path          path
```

`content_crc32 = 0` means unavailable/not calculated, not necessarily that file contents have CRC zero.

## 50. FILE_OPEN

Open modes:

```text
0 READ
1 WRITE_TRUNCATE
2 WRITE_CREATE_EXCLUSIVE
3 APPEND
4 STAGING_REPLACE
```

Request:

```text
area_id   u8
mode      u8
path      path
```

Response:

```text
handle    u16
size      u64
flags     u16
```

`handle = 0` is invalid.

Handles are session-scoped and are closed automatically on disconnect.

## 51. FILE_READ

Request:

```text
handle    u16
offset    u64
length    u16
```

Response:

```text
handle    u16
offset    u64
length    u16
data      u8[length]
```

Returned length may be less than requested at EOF or due to runtime chunk limits.

## 52. FILE_WRITE

Request:

```text
handle    u16
offset    u64
length    u16
data      u8[length]
```

Response:

```text
handle           u16
offset           u64
accepted_length  u16
```

A retry with the same request sequence and identical payload is covered by duplicate suppression and must not create duplicate append data.

For append mode, the client should use the offset reported by its own transfer state and firmware must reject impossible offsets rather than silently moving them unless append semantics are explicitly documented for that handle.

## 53. FILE_CLOSE

Request:

```text
handle           u16
expected_size    u64
expected_crc32   u32
flags            u16
```

`expected_crc32 = 0` means no end-to-end content CRC requested.

Response:

```text
handle         u16
final_size     u64
final_crc32    u32
status         u8
```

Staging/atomic replacement is committed only after validation succeeds.

## 54. FILE_DELETE

**OPTIONAL.**

Request:

```text
area_id  u8
path     path
```

A device must reject deletion in areas not advertised writable/deletable.

## 55. FILE_RENAME

**OPTIONAL.**

Request:

```text
area_id       u8
old_path      path
new_path      path
```

Cross-area rename is not supported in v1.

## 56. File-service scheduling

Storage operations are always lower priority than control and high-rate realtime traffic.

A large file operation must be naturally chunkable and resumable.

The host should pipeline conservatively. The v1 protocol does not require multiple outstanding writes to the same file handle.

---

# Part XIV — Ping and errors

## 57. PING

PING request:

```text
cookie       u32
host_time_us u64
```

PING response copies:

```text
cookie          u32
host_time_us    u64
device_time_us  u64
```

This supports:

- connection liveness;
- rough latency measurement;
- optional host/device timestamp alignment.

It is not a clock synchronization protocol.

## 58. ERROR

`ERROR` is a direct response to a validly parsed request that cannot be completed.

It uses the request sequence and sets `FLAG_RESPONSE`.

Payload:

```text
request_message_type  u8
error_code            u16
object_id             u32
detail                 str16
```

`object_id = 0` if not applicable.

`detail` should be short and diagnostic. Client logic must key on `error_code`, not parse human text.

## 59. Canonical error codes

```text
0x0001 UNKNOWN_MESSAGE
0x0002 BAD_VERSION
0x0003 BAD_LENGTH
0x0004 BAD_ID
0x0005 BAD_TYPE
0x0006 OUT_OF_RANGE
0x0007 READ_ONLY
0x0008 WRITE_ONLY
0x0009 BUSY
0x000A NOT_AVAILABLE
0x000B REQUIRES_RECONFIGURE
0x000C REQUIRES_REBOOT
0x000D INVALID_STATE
0x000E INVALID_ROUTE
0x000F TYPE_MISMATCH
0x0010 TOO_MANY_ITEMS
0x0011 PAYLOAD_TOO_LARGE
0x0012 SEQUENCE_CONFLICT
0x0013 STORAGE_ERROR
0x0014 FILE_NOT_FOUND
0x0015 FILE_ACCESS
0x0016 FILE_RANGE
0x0017 FILE_STORAGE_FULL
0x0018 FILE_BAD_HANDLE
0x0019 FILE_VERIFY_FAILED
0x001A ACTION_REJECTED
0x001B SUBSCRIPTION_LIMIT
0x001C UNSUPPORTED_RATE
0x001D DESCRIPTOR_VERSION
0x001E NOT_READY
0x001F INTERNAL_ERROR
```

### 59.1 Framing errors are not ERROR packets

The following occur before a trusted request exists and therefore do not generate request-correlated `ERROR` packets:

- malformed COBS;
- bad CRC;
- impossible decoded header length;
- unsupported header major version that cannot safely be parsed.

These are silently dropped and may update diagnostics.

---

# Part XV — Board capability truth

## 60. Full H743 board

The protocol must describe the actual full-board endpoint inventory from the final board definition.

Current hardware truth includes, at a high level:

- native USB FS;
- 12 primary PWM outputs;
- six timer-capable Flex channels;
- a separate dedicated UART7 RX row in the same physical lower header bank;
- onboard ICM-20602 IMU;
- onboard MMC5983MA compass;
- dual CAN physical interfaces using the currently selected transceiver implementation;
- multiple external serial connectors;
- VCC ADC sensing;
- no currently verified SD card in the supplied schematic.

Therefore the firmware/client must **not** infer:

- seven Flex ports from seven header rows;
- ADC capability on every Flex row;
- CAN FD data-phase capability from the STM32 FDCAN peripheral alone;
- SD capability from a configured but unpopulated/unused SPI peripheral;
- external UART capability merely because an MCU UART alternate function is enabled in CubeMX.

## 61. Reduced F405 board

Current reduced-board hwdef verifies, at a high level:

- native USB FS;
- USART1, USART3, USART6;
- external USART1 SBUS inversion control;
- six PWM outputs;
- BMI088 IMU;
- one I2C bus;
- battery voltage/current ADC inputs;
- USB-sense ADC;
- SPI SD card;
- SPI dataflash;
- internal parameter storage;
- no current CAN endpoint;
- no current generic Flex bank.

The reduced board runs the same Vektor protocol and common application model. It simply advertises fewer endpoints and may accept lower realtime rates.

## 62. Runtime beats board-name branching

A configurator may use `hardware_type` and `hardware_revision` for friendly artwork or labels.

It must use descriptors for functional decisions.

Bad client logic:

```text
if board == H743:
    show 12 PWM + 6 Flex + CAN + SD
```

Correct client logic:

```text
if descriptors contain PWM bank with 12 channels:
    show 12 PWM channels

if descriptors contain 6 Flex endpoints:
    show 6 Flex endpoints

if CAN endpoint exists:
    show CAN routing

if storage area exists:
    show the corresponding file UI
```

---

# Part XVI — Session lifecycle and configurator flow

## 63. Connection sequence

Canonical connection flow:

```text
transport open
    |
    v
optional 0x00 parser-reset delimiter
    |
    v
HELLO
    |
    +--> verify compatible major
    +--> record firmware/board identity
    +--> record schema_hash/capability_hash
    |
    v
load matching cached descriptors if both hashes match
    |
    +--> otherwise DESCRIBE required domains
    |
    v
build hardware endpoint model
    |
    v
build application component/field model
    |
    v
GET_ALL_PARAMS / required VALUES
    |
    v
ROUTE_LIST
    |
    v
create requested SUBSCRIBE streams
    |
    v
normal operation
```

## 64. Descriptor caching

Client descriptor cache key should include at least:

```text
selected_major
schema_hash
capability_hash
```

`device_id` is not required for schema cache sharing. Two identical boards/builds may safely use the same descriptor cache.

If either hash is zero, the client should re-describe the relevant domain rather than trust a persistent cache.

## 65. Disconnect behavior

On transport/session loss:

- subscriptions are destroyed;
- file handles are closed;
- incomplete noncommitted staging file operations remain subject to backend recovery policy;
- parameters already successfully persisted remain persisted;
- routes already successfully persisted remain persisted;
- running application control does not stop merely because the configurator disconnected;
- configurator-only actions do not imply an arming state.

Vektor is power-on operational. The service connection is not an autopilot arming link.

---

# Part XVII — Security and trust boundary

## 66. v1 trust model

Protocol v1 does **not** define cryptographic authentication, encryption, or authorization.

USB/UART access is therefore treated as trusted physical/service access unless the product layer adds another protection mechanism.

Consequences:

- `device_id` is identification, not authentication;
- CRC is corruption detection, not security;
- a hostile host with physical protocol access may attempt parameter changes/actions;
- firmware must still validate every value and bound every action regardless of trust.

If authenticated remote/network transport is introduced later, it must be layered deliberately rather than pretending CRC provides security.

---

# Part XVIII — Implementation rules

## 67. Firmware parser rules

Firmware must:

1. parse incrementally from a byte stream;
2. bound the encoded and decoded frame buffers;
3. reject oversized frames before unsafe allocation/copy;
4. validate COBS;
5. validate minimum decoded length;
6. validate `payload_len` against actual decoded size;
7. validate CRC before dispatch;
8. dispatch only supported message types;
9. avoid heap dependence in the fast path where practical;
10. never run file/storage work directly in a control-critical ISR;
11. avoid letting telemetry queue growth consume unbounded memory.

## 68. Host parser rules

The configurator must:

- parse arbitrary chunking;
- ignore empty zero-delimited spans;
- recover after corrupt frames;
- verify CRC before use;
- correlate direct responses by sequence;
- tolerate unsolicited EVENT/TELEMETRY at any time after HELLO;
- use descriptor types rather than guessed payload layouts;
- honor runtime limits;
- avoid blocking the UI thread on serial reads/writes.

## 69. Numeric validation

Firmware owns final semantic validation.

Descriptor min/max values are useful client guidance but do not replace server validation.

The host should reject obviously invalid user input early, then firmware validates again.

## 70. Unknown data handling

Within protocol major 1:

- unknown message type request to a device -> `UNKNOWN_MESSAGE` if header/CRC are valid;
- unknown descriptor record version -> client skips that record;
- unknown field/endpoint/action IDs -> `BAD_ID`;
- unknown enum numeric value received from device -> client should display numeric fallback rather than crash;
- unknown capability flag bit -> ignore;
- unknown frame flag bit -> ignore unless future negotiated behavior says otherwise.

---

# Part XIX — Canonical PING frame test vector

## 71. CRC check vector

Standard CRC check:

```text
ASCII: 31 32 33 34 35 36 37 38 39
CRC32: CB F4 39 26 as numeric 0xCBF43926
wire little-endian bytes: 26 39 F4 CB
```

## 72. Complete frame example

Example PING request fields:

```text
version       = 0x01
message_type  = 0x03  (PING)
flags         = 0x00
sequence      = 0x1234
payload_len   = 0x000C
cookie        = 0x12345678
host_time_us  = 0x0102030405060708
```

PING payload on the wire:

```text
78 56 34 12 08 07 06 05 04 03 02 01
```

Decoded bytes before CRC:

```text
01 03 00 34 12 0C 00 78 56 34 12 08 07 06 05 04 03 02 01
```

CRC-32/ISO-HDLC:

```text
numeric CRC = 0x95D9B856
wire CRC    = 56 B8 D9 95
```

Complete decoded frame:

```text
01 03 00 34 12 0C 00 78 56 34 12 08 07 06 05 04 03 02 01 56 B8 D9 95
```

COBS encoded body:

```text
03 01 03 04 34 12 0C 11 78 56 34 12 08 07 06 05 04 03 02 01 56 B8 D9 95
```

Canonical delimited stream example with an optional leading reset delimiter:

```text
00 03 01 03 04 34 12 0C 11 78 56 34 12 08 07 06 05 04 03 02 01 56 B8 D9 95 00
```

This vector should be used in firmware and PC unit tests.

---

# Part XX — MAVLink coexistence

## 73. Parallel adapters

Canonical architecture:

```text
                      Vektor application model
                       /                  \
                      /                    \
            Vektor Protocol              MAVLink
            native configurator          ecosystem tools
```

Vektor Protocol owns the richer native model:

- board capability discovery;
- hardware endpoint descriptors;
- typed component discovery;
- fast bulk configuration;
- routing;
- actions/events;
- high-rate compact subscriptions;
- logical file service.

MAVLink remains useful for:

- standard parameter access;
- standard attitude/status/sensor messages where applicable;
- compatibility with existing tooling.

## 74. Parameter consistency rule

If a Vektor parameter is exposed through both adapters:

```text
Vektor SET
     \
      -> common ParameterService -> AP_Param / component notification
     /
MAVLink PARAM_SET
```

Both transports must produce equivalent application-side results.

No second shadow parameter database is allowed.

---

# Part XXI — Performance guidance without false promises

## 75. No universal loop-rate promise

The protocol does not promise identical realtime rates on all boards.

The H743 may support higher accepted rates than the F405, but rates are runtime data.

Earlier design targets such as:

- 250–500 Hz attitude preview;
- roughly 1 kHz-class H743 control;
- roughly 500 Hz-class F405 control;

remain engineering targets to validate, not hard-coded wire guarantees.

The client requests a desired subscription period and accepts the device's negotiated result.

## 76. Control path isolation

The protocol scheduler must never make the control loop wait for:

- GUI reads;
- descriptor downloads;
- file reads/writes;
- log extraction;
- slow host rendering;
- a full telemetry queue.

A busy configurator is not a reason for actuator latency to change materially.

---

# Part XXII — Source reconciliation and superseded assumptions

## 77. Carried forward from the original protocol draft

The following original design directions are retained and are now canonicalized here:

- COBS zero-delimited byte-stream framing;
- little-endian decoded fields;
- CRC32 protection;
- typed fields;
- stable numeric IDs;
- `HELLO` and `DESCRIBE` discovery;
- `GET`/`SET` plus bulk parameter operations;
- push-based realtime subscriptions;
- compact telemetry based on known subscription field order;
- actions separate from parameters;
- asynchronous events;
- logical bounded file areas;
- file offset/chunk semantics;
- newest-value-wins realtime behavior;
- file traffic lower priority than control/realtime;
- Vektor and MAVLink as parallel adapters.

## 78. Resolved ambiguities from the original draft

This one-truth version freezes details the earlier draft left open:

- exact CRC algorithm;
- CRC byte order;
- message numeric IDs;
- flag bits;
- stable-ID generation algorithm;
- descriptor-cache hash algorithm;
- request/response duplicate suppression behavior;
- typed primitive numeric IDs;
- telemetry timestamp width and semantics;
- telemetry quality encoding;
- route-management messages;
- file path rules;
- core error numeric codes;
- a complete frame test vector.

## 79. Corrected capability assumptions

Protocol capability discovery must follow actual board truth.

In particular:

- H743 CubeMX peripheral enablement is not equivalent to connector exposure;
- the lower H743 header bank is six Flex channels plus one dedicated UART7 RX row, not seven identical Flex ports;
- the H743's FDCAN peripheral does not by itself prove physical CAN FD data-phase capability through the selected transceiver;
- the current H743 source set does not verify an SD card;
- the F405 reduced board does verify SD and SPI dataflash;
- the reduced F405 currently has no CAN or generic Flex bank to advertise.

---

# Part XXIII — What remains outside the protocol

## 80. OPEN-PRODUCT items

These are intentionally not solved by this wire specification:

- exact USB device class/product identifiers;
- whether customer firmware update uses a bootloader, application staging service, or another mechanism;
- which UART endpoint, if any, exposes Vektor Protocol by default;
- per-board default realtime rate profiles after profiling;
- exact board/revision numeric `board_id` allocation table;
- final user permission model if a future remote transport is added;
- GUI visual design;
- which optional file areas each board/build exposes;
- which optional action schemas ship in a particular application build.

They must be reflected through descriptors/capabilities where relevant, but they do not change the v1 packet grammar.

---

# Part XXIV — Minimal v1 implementation order

## 81. Firmware bring-up sequence

Recommended implementation order:

1. byte-stream RX/TX abstraction;
2. COBS codec;
3. canonical CRC32;
4. frame parser and bounded buffers;
5. `PING` plus frame test vector;
6. `HELLO`;
7. board/application capability registry;
8. `DESCRIBE`;
9. stable schema generation and hashes;
10. `GET`, `SET`, `VALUE`;
11. `GET_MANY`, `SET_MANY`, `VALUES`, `GET_ALL_PARAMS`;
12. route service messages;
13. realtime subscription scheduler;
14. `ACTION`, `ACTION_ACK`, `EVENT`;
15. file service on boards with real storage backends;
16. MAVLink coexistence validation.

## 82. Configurator bring-up sequence

1. serial transport abstraction;
2. streaming COBS parser;
3. CRC/frame tests using the canonical vector;
4. `PING`;
5. `HELLO`;
6. descriptor cache;
7. `DESCRIBE` object model;
8. parameter editor from descriptors;
9. hardware endpoint renderer;
10. routing model;
11. realtime subscriptions and attitude preview;
12. actions/events;
13. file browser/service where advertised.

---

# Part XXV — Short implementation reference

## 83. Frame

```text
COBS(
    version      u8
    message      u8
    flags        u8
    sequence     u16 LE
    payload_len  u16 LE
    payload      N bytes
    crc32        u32 LE
)
00
```

CRC:

```text
CRC-32/ISO-HDLC over decoded header + payload
```

## 84. Core connection

```text
HELLO
DESCRIBE as required
GET_ALL_PARAMS
ROUTE_LIST
SUBSCRIBE
```

## 85. Core object model

```text
board/hardware descriptors
        +
application component descriptors
        +
fields: INPUT / OUTPUT / PARAMETER / OBSERVABLE
        +
actions/events
        +
routes
```

## 86. Core scheduling rule

```text
control response > important event > realtime > bulk > file
```

Realtime:

```text
newest value wins
```

## 87. Core product rule

> The board defines physical reality. The Vektor application defines logical machine behavior. Vektor Protocol exposes both without requiring the client to know the PCB in advance.

---

# Part XXVI — Canonical v1 constants table

## 88. Header constants

```text
VEKTOR_PROTOCOL_MAJOR = 1
VEKTOR_HEADER_SIZE    = 7
VEKTOR_CRC_SIZE       = 4
VEKTOR_MIN_FRAME_SIZE = 11
VEKTOR_DELIMITER      = 0x00
```

## 89. Flag constants

```text
FLAG_RESPONSE = 0x01
FLAG_MORE     = 0x02
FLAG_URGENT   = 0x04
FLAG_VOLATILE = 0x08
```

## 90. Required v1 messages

A minimal compliant Vektor configurator-facing firmware must implement:

```text
HELLO
DESCRIBE
PING
ERROR
GET
SET
VALUE
GET_MANY
SET_MANY
VALUES
GET_ALL_PARAMS
SUBSCRIBE
UNSUBSCRIBE
TELEMETRY
ACTION
ACTION_ACK
EVENT
ROUTE_LIST
ROUTE_SET
ROUTE_DELETE
ROUTES
```

File messages are required only when `CAP_FILE_SERVICE` is advertised.

## 91. Final rule

The protocol must remain boring, explicit, bounded, and discoverable.

Do not move product intelligence into packet framing. Do not teach the configurator board-specific assumptions that the device can describe itself. Do not make high-rate visualization compete with control traffic. Do not expose a capability because the MCU could theoretically perform it.

The native Vektor serial protocol exists to make the same Vektor application understandable and controllable across different hardware targets without creating firmware forks or configurator forks.
