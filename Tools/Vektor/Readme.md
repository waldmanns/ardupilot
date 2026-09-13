# Vektor

Vektor is the initial ArduPilot/Waf firmware scaffold for the Vektor Core Evo
project family.

The canonical source for the current project facts is:

```text
Tools/Vektor/VEKTOR_CORE_EVO_TRUTH_BASE.md
```

This scaffold intentionally starts small. It provides:

- a `Vektor` application entry point;
- a folder-local waf program definition;
- a board capability model for the H743 Core Evo and F405 Reduced targets;
- a small runtime state service with loop timing and uptime observables;
- a minimal AP_Param-backed system parameter set;
- typed signal primitives with timestamp and quality;
- Vektor Serial Protocol v1 framing with COBS, CRC-32/ISO-HDLC, `PING`,
  `HELLO`, paged `DESCRIBE` for board/endpoints/timer groups/protocol/runtime
  diagnostics/system parameters, typed diagnostic and parameter `GET`/
  `GET_MANY`, persistent parameter `SET`/`SET_MANY`, `GET_ALL_PARAMS`, and
  protocol `ERROR` responses;
- a small duplicate-request cache that replays identical retries and rejects
  sequence reuse with changed payloads.

The first exposed persistent parameters are:

- `SYS_OPTIONS`: reserved system option bitmask;
- `SYS_DESC_PAGE`: default descriptor records per `DESCRIBE` page;
- `SYS_PROTO_BAUD`: protocol UART baud rate used on boot.

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
