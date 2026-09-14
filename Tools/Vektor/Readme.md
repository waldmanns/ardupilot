# Vektor

Vektor is the initial ArduPilot/Waf firmware scaffold for the Vektor Core Evo
project family.

The canonical source for the current project facts is:

```text
Tools/Vektor/VEKTOR_CORE_EVO_TRUTH_BASE.md
```

PC configurator implementation instructions for the currently available USB
CDC protocol surface are in `Tools/Vektor/VEKTOR_PC_CLIENT_HANDOFF.md`.

This scaffold intentionally starts small. It provides:

- a `Vektor` application entry point;
- a folder-local waf program definition;
- a board capability model for the H743 Core Evo and F405 Reduced targets;
- a small runtime state service with loop timing and uptime observables;
- a minimal AP_Param-backed system parameter set;
- typed signal primitives with timestamp and quality;
- a 16-channel RC input source with AP_RCProtocol UART autodetection,
  per-channel calibration, receiver failsafe/freshness tracking, and normalized
  typed outputs;
- six GPIO edge-capture PWM inputs with independent freshness/quality state;
- a normalized PWM output bank that drives the board-advertised output count,
  owns pulse calibration and frame rate, and disables unrouted outputs;
- a common component/field schema registry that owns stable IDs and emits the
  protocol descriptor records;
- Vektor Serial Protocol v1 framing with COBS, CRC-32/ISO-HDLC, `PING`,
  `HELLO`, paged `DESCRIBE` for board/endpoints/timer groups/protocol/runtime
  diagnostics/system parameters, typed diagnostic and parameter `GET`/
  `GET_MANY`, persistent parameter `SET`/`SET_MANY`, `GET_ALL_PARAMS`, and
  protocol `ERROR` responses;
- stable schema/capability hashes over ID-sorted descriptor records, runtime
  limit discovery, and startup rejection of zero or colliding object IDs;
- bounded realtime `SUBSCRIBE`/`UNSUBSCRIBE` sessions and compact volatile
  `TELEMETRY` for the built-in protocol/runtime observables;
- a bounded persistent assignment matrix with type, direction, multiplicity,
  stable route-ID, and component-cycle validation, exposed through
  `ROUTE_LIST`, `ROUTE_SET`, and `ROUTE_DELETE`;
- a small duplicate-request cache that replays identical retries and rejects
  sequence reuse with changed payloads.

The first exposed persistent parameters are:

- `SYS_OPTIONS`: reserved system option bitmask;
- `SYS_DESC_PAGE`: default descriptor records per `DESCRIBE` page;
- `SYS_PROTO_BAUD`: protocol UART baud rate used on boot.
- `RCIN_PORT`: HAL serial index used for the receiver UART (`1` by default,
  `0` disables the added UART);
- `RCIN_TIMEOUT`: receiver freshness timeout in milliseconds.

PWM input parameters are `PWIN1_PIN` through `PWIN6_PIN`, `PWIN_TIMEOUT`,
`PWIN_MIN`, `PWIN_TRIM`, and `PWIN_MAX`. A pin value of `-1` disables that
input. On `revo-mini`, GPIO 50 through 55 correspond to PWM connector outputs
1 through 6. Configuring one of those pins as an input reserves that physical
channel, so it cannot simultaneously be targeted as a PWM output.

The output bank uses `PWM_RATE` (50, 100, 200, or 330 Hz), `PWM_MIN`,
`PWM_TRIM`, `PWM_MAX`, `PWM_REVERSE`, and `PWM_FAILSAFE`. A zero failsafe value
disables a channel whenever its assigned signal is stale or invalid; a nonzero
value sends that bounded pulse instead. Unassigned channels always remain
disabled.

The standard `RC1_*` through `RC16_*` calibration parameters and
`RC_PROTOCOLS` mask are also registered. The selected UART uses ArduPilot's
compiled-in receiver autodetector, including serial protocols such as iBUS,
SBUS, DSM, SUMD, SRXL, CRSF/ELRS, FPort, and Ghost where enabled for the board.
The native schema exposes that standard mask as
`component/rcin/0/parameter/protocol_mask`. The resulting normalized channels
appear as routable fields under
`component/rcin/0/output/channel_1` through `channel_16`.

Assignments are stored through `AP_Param` and allow one source per destination;
setting a new source for an assigned destination replaces the old route. Route
flags are reserved and must currently be zero. The VSP component consumes its
assigned X/Y samples, but its control-law `update()` remains intentionally
empty for manual implementation.

Minimal direct routes use these stable schema paths:

```text
# serial receiver channel 1 -> physical PWM output 1
component/rcin/0/output/channel_1
    -> component/pwm_output/0/input/channel_1

# GPIO-captured PWM input 1 -> physical PWM output 2
component/pwm_input/0/output/channel_1
    -> component/pwm_output/0/input/channel_2
```

`ROUTE_SET` carries the FNV-1a field IDs discovered from those descriptors.
PWM input and serial receiver samples therefore use the same normalized,
timestamped routing path. On the reduced F405 target only PWM output channels
1 through 6 are accepted; the full-board capability permits channels 1
through 12 once its H743 hwdef is available.

Build the current configured board with:

```text
./waf vektor
```

Run the host protocol tests with:

```text
./waf configure --board sitl
./waf --targets tests/test_vektor_protocol
build/sitl/tests/test_vektor_protocol
```

The current `revo-mini` hwdef represents the reduced F405 target facts captured
in the truth base. The full H743 target still needs its ChibiOS hwdef before it
can be built as hardware firmware.
