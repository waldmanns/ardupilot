# VTEL: configurable i-BUS and Graupner HoTT telemetry

This is the Vektor2 **Telemetry Builder** reference and wiring/topology README. It
tracks the implementation in `VTelemetryCore.*`, `VTelemetry.*` and
`VTelemetryParameters.*` in this archive. VTEL is optional and separate from
MAVLink and the 100 Hz propulsion router. It is **not** a replacement for RC
input, failsafe logic or a physical battery monitor.

## Architecture: one source catalog, protocol-specific endpoints

```text
  GPS / AHRS / RC / VSP / thrusters / RPM / ADC / navigation / VRS / VFN / VSEQ / PWM
                                     |
                       numeric source ID + type + unit
                                     |
                      VTEL source catalog and reader
                                     |
                  10 Hz snapshot + validated slot routing
                   /                                 \
       i-BUS virtual sensors                  HoTT virtual modules
       addresses 1..15                         EAM / GPS / VARIO
                   |                                 |
       i-BUS serial task                      HoTT serial task
          115200 baud                           19200 baud
                   |                                 |
    receiver telemetry socket               receiver HoTT socket
                   |                                 |
             transmitter                          transmitter
```

`VTEL_SLTn` is an `AP_Int16` holding **only the source ID**. `0` means
unassigned and is also a valid saved override to the factory default. Source
IDs do not depend on the slot or protocol, so the same source can feed several
fields or both protocols. There are no per-slot scaling, unit or type
parameters. The source catalog has stable IDs and a data shape (`Number` or
`CoordinateE7`), while the destination catalog provides the protocol encoding.

Mappings are rechecked at the 10 Hz snapshot rate when parameters change. An
unknown ID or incompatible structure is ignored only for that destination.
**Physical meaning is deliberately not checked**: a voltage number can feed a
HoTT temperature or RPM field if you choose. Numerical values are encoded
according to the destination's fixed scale, offset and range, not implicitly
converted according to the source's unit. Non-finite/unavailable samples are
not substituted with invented readings. i-BUS suppresses the measurement
reply, while HoTT retains the field's neutral/empty representation within a
valid fixed-layout module packet. Numeric overflow clamps to the destination
range.

## Physical and logical bus topology

### FlySky i-BUS sensor telemetry

```text
                 one bidirectional telemetry line, 115200 baud
   Vektor2 UART (hardware half-duplex) <----------------> i-BUS receiver SENSOR port
                                                 |
                                          RF link to transmitter
```

* Set a dedicated UART to `SERIALx_PROTOCOL=49` (i-BUS telemetry). This is the
  **sensor telemetry** interface, not the i-BUS RC-channel input stream.
* The receiver is the bus master. It discovers virtual sensor addresses in
  ascending order: `0x81..0x8F` for discovery, `0x91..0x9F` for measurement
  type, `0xA1..0xAF` for the measurement. Address zero is reserved by the
  receiver; the firmware offers addresses **1–15**.
* A request has four bytes: length `04`, command/address, checksum low,
  checksum high. Responses have 4, 6 or 8 bytes, according to command and
  sensor-width. Every frame uses little-endian data and the subtractive 16-bit
  checksum `0xFFFF - sum(frame bytes except checksum)`.
* The receiver can stop discovery **at the first missing address**. If you
  disable or invalidate `VTEL_SLT3`, addresses 4–15 might not be discovered
  even if correctly assigned. Keep enabled address assignments contiguous
  from slot 1 whenever possible. A currently unavailable sample does not
  disable discovery of an otherwise valid source.
* i-BUS sensor *type* is selected by the **assigned source**, including wire
  width and scaling. These are virtual addresses in one Vektor2 device: you do
  **not** need 15 physical sensor modules or separate UARTs.
* Receivers and transmitter firmware vary in which sensor types and how many
  addresses they can display. Validate unusual assignments on the actual
  receiver.

### Graupner HoTT binary telemetry

```text
                 one receiver-polled HoTT telemetry bus, 19200 baud
   Vektor2 UART (bidirectional) <---------------> HoTT receiver telemetry port
             |
             +-- virtual EAM  (0x8E) -- fixed fields, VTEL_SLT16..43
             +-- virtual GPS  (0x8A) -- fixed fields, VTEL_SLT44..58
             +-- virtual VARIO(0x89) -- fixed fields, VTEL_SLT59..65
                                                 |
                                          RF link to transmitter
```

* Set a UART to `SERIALx_PROTOCOL=27`. The receiver polls a **module ID**:
  binary request `[0x80, module_id]`. Vektor2 responds **only** when at least
  one valid source is configured in that virtual module.
* Each reply contains a **44-byte fixed-format module packet** followed by a
  one-byte additive checksum. The packet begins `0x7C`, contains the requested
  module ID in byte 2 (zero-based offset 1), and ends with `0x7D` in byte 44.
  The byte after that is the checksum (`sum(44 packet bytes) mod 256`).
* The worker waits about **4 ms** before the first byte and spaces each packet
  byte by approximately **1.2 ms**. That work is isolated from the main control
  loop. The field layout is **not** dynamically advertised to the receiver.
* The three virtual modules coexist on **one HoTT UART and telemetry wire**;
  do not assign one UART per module. Each field has a fixed meaning in the
  radio UI, even if you intentionally wire it to an unrelated number.
* The current backend supports HoTT **binary** requests (`0x80`). HoTT text
  menus (`0x7F`) and alarm/beep user configuration are **not emulated**.
  Reserved flags, text, warnings and headers are not assignable `VTEL_SLTn`
  parameters.

**Wiring caution:** Both protocols use receiver-specific bidirectional
telemetry wiring and voltage levels. Verify the receiver's *sensor/telemetry*
connector, board UART half-duplex support and signal voltage before attaching
wires. Do not connect a 5 V receiver signal to a 3.3 V-only MCU pin without
suitable protection/level shifting. i-BUS and HoTT are distinct protocols,
so **use separate appropriately configured UARTs** if both are needed at once.
VTEL opens the first serial instance of each configured role. Factory slot
values alone do not activate a port or force a UART onto the telemetry bus.

## HoTT virtual module emulation: supported vs unsupported

| HoTT module | Binary poll ID | Module identifier in response | VTEL slots | Status | Displayed content |
|---|---|---|---|---|---|
| **EAM** (Electric Air Module) | `0x8E` | `0xE0` | `16–43` | **Emulated** | cell voltages, battery voltage, temperature, altitude, current, capacity, climb, RPM and speed |
| **GPS** | `0x8A` | `0xA0` | `44–58` | **Emulated** | course, speed, coordinates, fix, satellites, altitude, climb, velocity and accuracy |
| **VARIO** | `0x89` | `0x90` | `59–65` | **Emulated** | altitude, min/max altitude, climb rates and yaw |
| **GAM** (General Air Module) | `0x8D` | `0xD0` | none | **Not emulated** | fuel, voltage, current, RPM, altitude, speed, etc. in a different fixed packet |
| **ESC / other HoTT modules** | depends on module | depends | none | **Not emulated** | device-specific data and screen layouts |

### How the receiver sees the emulated modules

For example, on the **same HoTT wire**, the receiver can request different
screens in successive transactions:

```text
HoTT receiver -> Vektor2: 80 8E   (ask for Electric Air Module)
Vektor2      -> receiver: 7C 8E ... E0 ... 7D CRC
HoTT receiver -> Vektor2: 80 8A   (ask for GPS module)
Vektor2      -> receiver: 7C 8A ... A0 ... 7D CRC
HoTT receiver -> Vektor2: 80 89   (ask for VARIO module)
Vektor2      -> receiver: 7C 89 ... 90 ... 7D CRC
HoTT receiver -> Vektor2: 80 8D   (ask for GAM)
Vektor2      -> receiver: (no reply; unsupported)
```

`EAM` exposes seven low and seven high cell-voltage fields, not fourteen
independent cells, followed by battery, temperature and other numeric fields.
`GPS` has **two five-byte coordinate structures**, so those two slots accept
only coordinate sources. `VARIO` has fixed numeric fields plus reserved
human-readable text bytes. VTEL leaves the text bytes blank instead of
pretending that numeric slot assignments are character strings. No
configuration parameter changes the announced *kind* of HoTT field: the
receiver's built-in EAM/GPS/VARIO screen determines its label and units.

EAM and GAM are **different** packet layouts even where the displayed concepts
overlap. Calling Vektor2's EAM emulator `GAM` or asking it to reply to `0x8D`
would make the receiver decode fields incorrectly. GAM needs its own field
catalog and slot range before it can be enabled. An unsupported module poll is
ignored, not answered with data from another module.

The standard-like **factory defaults** are selected when the controller has an
actually relevant measurement. A configured slot with an unavailable runtime
source still behaves as unavailable. Examples: `MP1_VOLT` and `MP2_VOLT`
feed EAM battery voltages, `VRPM1_RPM` feeds EAM RPM, GPS ground speed feeds GPS
speed, and GPS altitude/estimator climb populate altitude/climb. i-BUS defaults
use ascending addresses with standard sensor types. Cell voltages, battery
current/capacity, historical extrema and home distance remain zero by default
because Vektor2 does not natively calculate them here. GPS altitude is GPS MSL
height, not home-relative height; the HoTT field offset is still applied.

## Configuration examples

```text
# FlySky address 1: measured MP1 voltage, source #100, i-BUS sensor type voltage
VTEL_SLT1 = 100
# FlySky address 2: shaft RPM sensor, source #90
VTEL_SLT2 = 90

# HoTT EAM temperature 1: deliberately show VSP1 rod-X PWM (arbitrary numeric mapping)
VTEL_SLT32 = 40
# HoTT GPS latitude and longitude: structure-compatible e7 coordinates
VTEL_SLT46 = 6
VTEL_SLT47 = 7
# Disable a field and preserve that choice across reboots
VTEL_SLT35 = 0
```

## Slot/field catalog (exact firmware order)

Slot numbers are **persistent**. Adding new fields later must append them,
never move these existing assignments. For HoTT, `Offset` is zero-based within
its 44-byte binary packet. `Scale`/`Bias` means
`encoded_integer = clamp(round(source_number * scale + bias))`, except for
coordinate packing and GPS fix text. `Bytes=coord` means a 5-byte signed
DMM/decimal-minute coordinate field, not a floating-point scalar. Signed
numeric fields are encoded using two's complement.

| Slot | Endpoint / destination field | Offset | Format | Scale / bias | Factory source (ID: name) |
|---:|---|---:|---|---|---|
| `VTEL_SLT1` | i-BUS sensor address 1 | — | from assigned source | from assigned source | `18: IBUS_GPS_STATUS` |
| `VTEL_SLT2` | i-BUS sensor address 2 | — | from assigned source | from assigned source | `100: MP1_VOLT` |
| `VTEL_SLT3` | i-BUS sensor address 3 | — | from assigned source | from assigned source | `1: GPS_SPEED_MS` |
| `VTEL_SLT4` | i-BUS sensor address 4 | — | from assigned source | from assigned source | `10: AHRS_ROLL` |
| `VTEL_SLT5` | i-BUS sensor address 5 | — | from assigned source | from assigned source | `11: AHRS_PITCH` |
| `VTEL_SLT6` | i-BUS sensor address 6 | — | from assigned source | from assigned source | `12: AHRS_HEADING` |
| `VTEL_SLT7` | i-BUS sensor address 7 | — | from assigned source | from assigned source | `9: AHRS_CLIMB_MS` |
| `VTEL_SLT8` | i-BUS sensor address 8 | — | from assigned source | from assigned source | `3: GPS_COURSE` |
| `VTEL_SLT9` | i-BUS sensor address 9 | — | from assigned source | from assigned source | `8: GPS_ALT_M` |
| `VTEL_SLT10` | i-BUS sensor address 10 | — | from assigned source | from assigned source | `6: GPS_LAT` |
| `VTEL_SLT11` | i-BUS sensor address 11 | — | from assigned source | from assigned source | `7: GPS_LON` |
| `VTEL_SLT12` | i-BUS sensor address 12 | — | from assigned source | from assigned source | `90: VRPM1_RPM` |
| `VTEL_SLT13` | i-BUS sensor address 13 | — | from assigned source | from assigned source | `91: VRPM2_RPM` |
| `VTEL_SLT14` | i-BUS sensor address 14 | — | from assigned source | from assigned source | `2: GPS_SPEED_KMH` |
| `VTEL_SLT15` | i-BUS sensor address 15 | — | from assigned source | from assigned source | `101: MP2_VOLT` |
| `VTEL_SLT16` | EAM / EAM_CELL1_LOW | 6 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT17` | EAM / EAM_CELL2_LOW | 7 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT18` | EAM / EAM_CELL3_LOW | 8 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT19` | EAM / EAM_CELL4_LOW | 9 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT20` | EAM / EAM_CELL5_LOW | 10 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT21` | EAM / EAM_CELL6_LOW | 11 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT22` | EAM / EAM_CELL7_LOW | 12 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT23` | EAM / EAM_CELL1_HIGH | 13 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT24` | EAM / EAM_CELL2_HIGH | 14 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT25` | EAM / EAM_CELL3_HIGH | 15 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT26` | EAM / EAM_CELL4_HIGH | 16 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT27` | EAM / EAM_CELL5_HIGH | 17 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT28` | EAM / EAM_CELL6_HIGH | 18 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT29` | EAM / EAM_CELL7_HIGH | 19 | unsigned 1B | `×50 + 0` | — (`0`) |
| `VTEL_SLT30` | EAM / EAM_BATT1 | 20 | unsigned 2B | `×10 + 0` | `100: MP1_VOLT` |
| `VTEL_SLT31` | EAM / EAM_BATT2 | 22 | unsigned 2B | `×10 + 0` | `101: MP2_VOLT` |
| `VTEL_SLT32` | EAM / EAM_TEMP1 | 24 | unsigned 1B | `×1 + 20` | — (`0`) |
| `VTEL_SLT33` | EAM / EAM_TEMP2 | 25 | unsigned 1B | `×1 + 20` | — (`0`) |
| `VTEL_SLT34` | EAM / EAM_ALT | 26 | unsigned 2B | `×1 + 500` | `8: GPS_ALT_M` |
| `VTEL_SLT35` | EAM / EAM_CURRENT | 28 | unsigned 2B | `×10 + 0` | — (`0`) |
| `VTEL_SLT36` | EAM / EAM_VOLT | 30 | unsigned 2B | `×10 + 0` | `100: MP1_VOLT` |
| `VTEL_SLT37` | EAM / EAM_CAPACITY | 32 | unsigned 2B | `×0.1f + 0` | — (`0`) |
| `VTEL_SLT38` | EAM / EAM_CLIMB | 34 | unsigned 2B | `×100 + 30000` | `9: AHRS_CLIMB_MS` |
| `VTEL_SLT39` | EAM / EAM_CLIMB3 | 36 | unsigned 1B | `×3 + 120` | — (`0`) |
| `VTEL_SLT40` | EAM / EAM_RPM | 37 | unsigned 2B | `×0.1f + 0` | `90: VRPM1_RPM` |
| `VTEL_SLT41` | EAM / EAM_MIN | 39 | unsigned 1B | `×1 + 0` | — (`0`) |
| `VTEL_SLT42` | EAM / EAM_SEC | 40 | unsigned 1B | `×1 + 0` | — (`0`) |
| `VTEL_SLT43` | EAM / EAM_SPEED | 41 | unsigned 2B | `×1 + 0` | `2: GPS_SPEED_KMH` |
| `VTEL_SLT44` | GPS / GPS_DIR | 6 | unsigned 1B | `×0.5f + 0` | `3: GPS_COURSE` |
| `VTEL_SLT45` | GPS / GPS_SPEED | 7 | unsigned 2B | `×1 + 0` | `2: GPS_SPEED_KMH` |
| `VTEL_SLT46` | GPS / GPS_LAT | 9 | coordinate (5 bytes) | DMM encoding | `6: GPS_LAT` |
| `VTEL_SLT47` | GPS / GPS_LON | 14 | coordinate (5 bytes) | DMM encoding | `7: GPS_LON` |
| `VTEL_SLT48` | GPS / GPS_DISTANCE | 19 | unsigned 2B | `×1 + 0` | — (`0`) |
| `VTEL_SLT49` | GPS / GPS_ALT | 21 | unsigned 2B | `×1 + 500` | `8: GPS_ALT_M` |
| `VTEL_SLT50` | GPS / GPS_CLIMB | 23 | unsigned 2B | `×100 + 30000` | `9: AHRS_CLIMB_MS` |
| `VTEL_SLT51` | GPS / GPS_CLIMB3 | 25 | unsigned 1B | `×3 + 120` | — (`0`) |
| `VTEL_SLT52` | GPS / GPS_SATS | 26 | unsigned 1B | `×1 + 0` | `4: GPS_SATS` |
| `VTEL_SLT53` | GPS / GPS_FIX | 27 | unsigned 1B | GPS fix text (`2`/`3`/`-`) | `5: GPS_FIX` |
| `VTEL_SLT54` | GPS / GPS_HOME_DIR | 28 | unsigned 1B | `×0.5f + 0` | — (`0`) |
| `VTEL_SLT55` | GPS / GPS_VN | 29 | signed 2B | `×1000 + 0` | `13: AHRS_VN_MS` |
| `VTEL_SLT56` | GPS / GPS_SACC | 31 | unsigned 1B | `×100 + 0` | `15: GPS_SACC_MS` |
| `VTEL_SLT57` | GPS / GPS_VE | 36 | signed 2B | `×1000 + 0` | `14: AHRS_VE_MS` |
| `VTEL_SLT58` | GPS / GPS_HACC | 38 | unsigned 1B | `×100 + 0` | `16: GPS_HACC_M` |
| `VTEL_SLT59` | VARIO / VARIO_ALT | 5 | unsigned 2B | `×1 + 500` | `8: GPS_ALT_M` |
| `VTEL_SLT60` | VARIO / VARIO_MAX | 7 | unsigned 2B | `×1 + 500` | — (`0`) |
| `VTEL_SLT61` | VARIO / VARIO_MIN | 9 | unsigned 2B | `×1 + 500` | — (`0`) |
| `VTEL_SLT62` | VARIO / VARIO_CLIMB | 11 | unsigned 2B | `×100 + 30000` | `9: AHRS_CLIMB_MS` |
| `VTEL_SLT63` | VARIO / VARIO_CLIMB3 | 13 | unsigned 2B | `×300 + 30000` | — (`0`) |
| `VTEL_SLT64` | VARIO / VARIO_CLIMB10 | 15 | unsigned 2B | `×1000 + 30000` | — (`0`) |
| `VTEL_SLT65` | VARIO / VARIO_YAW | 41 | unsigned 1B | `×0.5f + 0` | `12: AHRS_HEADING` |

## Source ID catalog

IDs are permanent parameter values and must not be renumbered. These are the
supported *readable values*, not a promise that all backing hardware exists.
For example, measured shaft RPM requires the board RPM input feature,
measurement points require configured analog inputs, and GPS positions require
GPS fix. The i-BUS type/width columns describe the virtual i-BUS sensor when
that source is assigned to any i-BUS address. The `Unit` is documentation only;
it **does not** block intentionally nonsemantic HoTT mappings.

| ID | Source name | Type | Native unit | i-BUS type | i-BUS width |
|---:|---|---|---|---|---|
| 1 | `GPS_SPEED_MS` | Number | `m/s` | `0x13` | 2 B |
| 2 | `GPS_SPEED_KMH` | Number | `km/h` | `0x7E` | 2 B |
| 3 | `GPS_COURSE` | Number | `deg` | `0x0A` | 2 B |
| 4 | `GPS_SATS` | Number | `count` | `0x02` | 2 B |
| 5 | `GPS_FIX` | Number | `enum` | `0x02` | 2 B |
| 6 | `GPS_LAT` | CoordinateE7 | `degE7` | `0x80` | 4 B |
| 7 | `GPS_LON` | CoordinateE7 | `degE7` | `0x81` | 4 B |
| 8 | `GPS_ALT_M` | Number | `m` | `0xF9` | 2 B |
| 9 | `AHRS_CLIMB_MS` | Number | `m/s` | `0x09` | 2 B |
| 10 | `AHRS_ROLL` | Number | `deg` | `0x0F` | 2 B |
| 11 | `AHRS_PITCH` | Number | `deg` | `0x10` | 2 B |
| 12 | `AHRS_HEADING` | Number | `deg` | `0x08` | 2 B |
| 13 | `AHRS_VN_MS` | Number | `m/s` | `0x02` | 2 B |
| 14 | `AHRS_VE_MS` | Number | `m/s` | `0x02` | 2 B |
| 15 | `GPS_SACC_MS` | Number | `m/s` | `0x02` | 2 B |
| 16 | `GPS_HACC_M` | Number | `m` | `0x02` | 2 B |
| 18 | `IBUS_GPS_STATUS` | Number | `packed` | `0x0B` | 2 B |
| 20 | `RC1` | Number | `us` | `0x02` | 2 B |
| 21 | `RC2` | Number | `us` | `0x02` | 2 B |
| 22 | `RC3` | Number | `us` | `0x02` | 2 B |
| 23 | `RC4` | Number | `us` | `0x02` | 2 B |
| 24 | `RC5` | Number | `us` | `0x02` | 2 B |
| 25 | `RC6` | Number | `us` | `0x02` | 2 B |
| 26 | `RC7` | Number | `us` | `0x02` | 2 B |
| 27 | `RC8` | Number | `us` | `0x02` | 2 B |
| 28 | `RC9` | Number | `us` | `0x02` | 2 B |
| 29 | `RC10` | Number | `us` | `0x02` | 2 B |
| 30 | `RC11` | Number | `us` | `0x02` | 2 B |
| 31 | `RC12` | Number | `us` | `0x02` | 2 B |
| 32 | `RC13` | Number | `us` | `0x02` | 2 B |
| 33 | `RC14` | Number | `us` | `0x02` | 2 B |
| 34 | `RC15` | Number | `us` | `0x02` | 2 B |
| 35 | `RC16` | Number | `us` | `0x02` | 2 B |
| 36 | `RC17` | Number | `us` | `0x02` | 2 B |
| 37 | `RC18` | Number | `us` | `0x02` | 2 B |
| 40 | `VSP1_ROD_X` | Number | `us` | `0x02` | 2 B |
| 41 | `VSP1_ROD_Y` | Number | `us` | `0x02` | 2 B |
| 42 | `VSP1_RPM_OUT` | Number | `us` | `0x02` | 2 B |
| 43 | `VSP2_ROD_X` | Number | `us` | `0x02` | 2 B |
| 44 | `VSP2_ROD_Y` | Number | `us` | `0x02` | 2 B |
| 45 | `VSP2_RPM_OUT` | Number | `us` | `0x02` | 2 B |
| 50 | `BOW_PWM` | Number | `us` | `0x02` | 2 B |
| 51 | `STERN_PWM` | Number | `us` | `0x02` | 2 B |
| 60 | `VSP1_TDIR` | Number | `deg` | `0x08` | 2 B |
| 61 | `VSP1_TMAG` | Number | `norm` | `0x07` | 2 B |
| 62 | `VSP2_TDIR` | Number | `deg` | `0x08` | 2 B |
| 63 | `VSP2_TMAG` | Number | `norm` | `0x07` | 2 B |
| 64 | `VSP1_VST` | Number | `enum` | `0x02` | 2 B |
| 65 | `VSP2_VST` | Number | `enum` | `0x02` | 2 B |
| 66 | `VSP1_AUTH` | Number | `mask` | `0x02` | 2 B |
| 67 | `VSP2_AUTH` | Number | `mask` | `0x02` | 2 B |
| 80 | `VRS_STATE` | Number | `enum` | `0x02` | 2 B |
| 81 | `VRS_DEMAND` | Number | `norm` | `0x07` | 2 B |
| 82 | `VRS_INHIB` | Number | `mask` | `0x02` | 2 B |
| 90 | `VRPM1_RPM` | Number | `rpm` | `0x02` | 2 B |
| 91 | `VRPM2_RPM` | Number | `rpm` | `0x02` | 2 B |
| 100 | `MP1_VOLT` | Number | `V` | `0x03` | 2 B |
| 101 | `MP2_VOLT` | Number | `V` | `0x03` | 2 B |
| 102 | `MP3_VOLT` | Number | `V` | `0x03` | 2 B |
| 103 | `MP4_VOLT` | Number | `V` | `0x03` | 2 B |
| 110 | `VFN1_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 111 | `VFN2_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 112 | `VFN3_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 113 | `VFN4_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 114 | `VFN5_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 115 | `VFN6_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 116 | `VFN7_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 117 | `VFN8_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 118 | `VFN9_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 119 | `VFN10_FLAGS` | Number | `mask` | `0x02` | 2 B |
| 120 | `VFN1_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 121 | `VFN2_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 122 | `VFN3_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 123 | `VFN4_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 124 | `VFN5_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 125 | `VFN6_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 126 | `VFN7_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 127 | `VFN8_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 128 | `VFN9_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 129 | `VFN10_ERRORS` | Number | `mask` | `0x02` | 2 B |
| 130 | `VFN1_PULSE` | Number | `us` | `0x02` | 2 B |
| 131 | `VFN2_PULSE` | Number | `us` | `0x02` | 2 B |
| 132 | `VFN3_PULSE` | Number | `us` | `0x02` | 2 B |
| 133 | `VFN4_PULSE` | Number | `us` | `0x02` | 2 B |
| 134 | `VFN5_PULSE` | Number | `us` | `0x02` | 2 B |
| 135 | `VFN6_PULSE` | Number | `us` | `0x02` | 2 B |
| 136 | `VFN7_PULSE` | Number | `us` | `0x02` | 2 B |
| 137 | `VFN8_PULSE` | Number | `us` | `0x02` | 2 B |
| 138 | `VFN9_PULSE` | Number | `us` | `0x02` | 2 B |
| 139 | `VFN10_PULSE` | Number | `us` | `0x02` | 2 B |
| 140 | `NAV_STATE` | Number | `enum` | `0x02` | 2 B |
| 141 | `NAV_RBITS` | Number | `mask` | `0x02` | 2 B |
| 240 | `DP_TGT_LAT` | CoordinateE7 | `degE7` | `0x80` | 4 B |
| 241 | `DP_TGT_LON` | CoordinateE7 | `degE7` | `0x81` | 4 B |
| 242 | `DP_ACT_LAT` | CoordinateE7 | `degE7` | `0x80` | 4 B |
| 243 | `DP_ACT_LON` | CoordinateE7 | `degE7` | `0x81` | 4 B |
| 244 | `DP_DIST` | Number | `m` | `0x02` | 2 B |
| 245 | `DP_BRG` | Number | `deg` | `0x08` | 2 B |
| 246 | `HDG_TGT` | Number | `deg` | `0x08` | 2 B |
| 247 | `HDG_ACT` | Number | `deg` | `0x08` | 2 B |
| 248 | `DP_ERR_N` | Number | `m` | `0x02` | 2 B |
| 249 | `DP_ERR_E` | Number | `m` | `0x02` | 2 B |
| 250 | `DP_CMD_X` | Number | `norm` | `0x02` | 2 B |
| 251 | `DP_CMD_Y` | Number | `norm` | `0x02` | 2 B |
| 252 | `HDG_CMD` | Number | `norm` | `0x02` | 2 B |
| 253 | `DP_VN` | Number | `m/s` | `0x02` | 2 B |
| 254 | `DP_VE` | Number | `m/s` | `0x02` | 2 B |
| 255 | `DP_CMD_MAG` | Number | `norm` | `0x02` | 2 B |
| 256 | `DP_CMD_BRG` | Number | `deg` | `0x08` | 2 B |
| 257 | `HDG_ERR` | Number | `deg` | `0x02` | 2 B |
| 258 | `DP_CMD_N` | Number | `norm` | `0x02` | 2 B |
| 259 | `DP_CMD_E` | Number | `norm` | `0x02` | 2 B |
| 260 | `DP_CALC` | Number | `bool` | `0x02` | 2 B |
| 261 | `HDG_CALC` | Number | `bool` | `0x02` | 2 B |
| 150 | `SEQ1_STATE` | Number | `enum` | `0x02` | 2 B |
| 151 | `SEQ2_STATE` | Number | `enum` | `0x02` | 2 B |
| 152 | `SEQ3_STATE` | Number | `enum` | `0x02` | 2 B |
| 153 | `SEQ4_STATE` | Number | `enum` | `0x02` | 2 B |
| 154 | `SEQ5_STATE` | Number | `enum` | `0x02` | 2 B |
| 155 | `SEQ6_STATE` | Number | `enum` | `0x02` | 2 B |
| 156 | `SEQ7_STATE` | Number | `enum` | `0x02` | 2 B |
| 157 | `SEQ8_STATE` | Number | `enum` | `0x02` | 2 B |
| 158 | `SEQ9_STATE` | Number | `enum` | `0x02` | 2 B |
| 159 | `SEQ10_STATE` | Number | `enum` | `0x02` | 2 B |
| 160 | `SEQ11_STATE` | Number | `enum` | `0x02` | 2 B |
| 161 | `SEQ12_STATE` | Number | `enum` | `0x02` | 2 B |
| 162 | `SEQ13_STATE` | Number | `enum` | `0x02` | 2 B |
| 163 | `SEQ14_STATE` | Number | `enum` | `0x02` | 2 B |
| 164 | `SEQ15_STATE` | Number | `enum` | `0x02` | 2 B |
| 165 | `SEQ16_STATE` | Number | `enum` | `0x02` | 2 B |
| 170 | `SEQ1_PULSE` | Number | `us` | `0x02` | 2 B |
| 171 | `SEQ2_PULSE` | Number | `us` | `0x02` | 2 B |
| 172 | `SEQ3_PULSE` | Number | `us` | `0x02` | 2 B |
| 173 | `SEQ4_PULSE` | Number | `us` | `0x02` | 2 B |
| 174 | `SEQ5_PULSE` | Number | `us` | `0x02` | 2 B |
| 175 | `SEQ6_PULSE` | Number | `us` | `0x02` | 2 B |
| 176 | `SEQ7_PULSE` | Number | `us` | `0x02` | 2 B |
| 177 | `SEQ8_PULSE` | Number | `us` | `0x02` | 2 B |
| 178 | `SEQ9_PULSE` | Number | `us` | `0x02` | 2 B |
| 179 | `SEQ10_PULSE` | Number | `us` | `0x02` | 2 B |
| 180 | `SEQ11_PULSE` | Number | `us` | `0x02` | 2 B |
| 181 | `SEQ12_PULSE` | Number | `us` | `0x02` | 2 B |
| 182 | `SEQ13_PULSE` | Number | `us` | `0x02` | 2 B |
| 183 | `SEQ14_PULSE` | Number | `us` | `0x02` | 2 B |
| 184 | `SEQ15_PULSE` | Number | `us` | `0x02` | 2 B |
| 185 | `SEQ16_PULSE` | Number | `us` | `0x02` | 2 B |
| 200 | `PWM1_OUT` | Number | `us` | `0x02` | 2 B |
| 201 | `PWM2_OUT` | Number | `us` | `0x02` | 2 B |
| 202 | `PWM3_OUT` | Number | `us` | `0x02` | 2 B |
| 203 | `PWM4_OUT` | Number | `us` | `0x02` | 2 B |
| 204 | `PWM5_OUT` | Number | `us` | `0x02` | 2 B |
| 205 | `PWM6_OUT` | Number | `us` | `0x02` | 2 B |
| 206 | `PWM7_OUT` | Number | `us` | `0x02` | 2 B |
| 207 | `PWM8_OUT` | Number | `us` | `0x02` | 2 B |
| 208 | `PWM9_OUT` | Number | `us` | `0x02` | 2 B |
| 209 | `PWM10_OUT` | Number | `us` | `0x02` | 2 B |
| 210 | `PWM11_OUT` | Number | `us` | `0x02` | 2 B |
| 211 | `PWM12_OUT` | Number | `us` | `0x02` | 2 B |
| 212 | `PWM13_OUT` | Number | `us` | `0x02` | 2 B |
| 213 | `PWM14_OUT` | Number | `us` | `0x02` | 2 B |
| 214 | `PWM15_OUT` | Number | `us` | `0x02` | 2 B |
| 215 | `PWM16_OUT` | Number | `us` | `0x02` | 2 B |
| 216 | `PWM17_OUT` | Number | `us` | `0x02` | 2 B |
| 217 | `PWM18_OUT` | Number | `us` | `0x02` | 2 B |
| 218 | `PWM19_OUT` | Number | `us` | `0x02` | 2 B |
| 219 | `PWM20_OUT` | Number | `us` | `0x02` | 2 B |
| 220 | `PWM21_OUT` | Number | `us` | `0x02` | 2 B |
| 221 | `PWM22_OUT` | Number | `us` | `0x02` | 2 B |
| 222 | `PWM23_OUT` | Number | `us` | `0x02` | 2 B |
| 223 | `PWM24_OUT` | Number | `us` | `0x02` | 2 B |
| 224 | `PWM25_OUT` | Number | `us` | `0x02` | 2 B |
| 225 | `PWM26_OUT` | Number | `us` | `0x02` | 2 B |
| 226 | `PWM27_OUT` | Number | `us` | `0x02` | 2 B |
| 227 | `PWM28_OUT` | Number | `us` | `0x02` | 2 B |
| 228 | `PWM29_OUT` | Number | `us` | `0x02` | 2 B |
| 229 | `PWM30_OUT` | Number | `us` | `0x02` | 2 B |
| 230 | `PWM31_OUT` | Number | `us` | `0x02` | 2 B |
| 231 | `PWM32_OUT` | Number | `us` | `0x02` | 2 B |

## Implementation and verification

- `VTelemetryCore.cpp` owns source/slot metadata, compatibility, clipping,
  checksums, coordinate conversion and the two packet encoders.
- `VTelemetryParameters.cpp` registers `VTEL_SLT1..VTEL_SLT65` as normal
  ArduPilot parameters. Slots `62..65` use an `AP_SUBGROUPEXTENSION` because
  the main AP_Param group has a bounded index range.
- `VTelemetry.cpp` reads Vektor2 subsystem state, refreshes a locked snapshot
  every 100 ms, and serves receiver polls from separate UART tasks. It does
  not alter the 100 Hz propulsion routing or the MAVLink telemetry publisher.
- `tests/test_vtelemetry.py` compiles the host-only core and exercises slot
  rejection, deliberate numeric mismatches, checksums, GPS coordinate
  encoding, EAM/GPS/VARIO packets and stale sensor behavior.

Host tests can be run with `python3 tests/test_vtelemetry.py`. **No full
ArduPilot build or live i-BUS/HoTT receiver test is included in this ZIP**.
Before operating, verify the board's serial options, module discovery,
receiver display behavior, valid/invalid signal transitions and byte timings
on real hardware, with propulsion outputs disconnected.

## Wire format references

- [ArduPilot i-BUS telemetry documentation](https://ardupilot.org/plane/docs/common-ibus-telemetry.html)
- [i-BUS single-wire telemetry polling and discovery](https://github.com/skotekar/wiki/blob/master/Single-wire-FlySky-(IBus)-telemetry.md)
- [ArduPilot HoTT binary implementation](https://firmware.ardupilot.org/coverage/AP_Hott_Telem/AP_Hott_Telem.cpp.gcov.html)
- [Paparazzi HoTT EAM definition](https://docs.paparazziuav.org/latest/hott__eam_8h_source.html)
- [Paparazzi HoTT GAM definition](https://docs.paparazziuav.org/latest/hott__gam_8h_source.html)

## M3.5/M3.8 DP sources and timing

`VTEL_SLTn` can select DP and shared-heading sources `240..261` in the same
way as other VTEL values. Existing IDs `240..256` retain their original units
and meaning. Sources `240..243` carry full-precision E7 coordinates and
require a coordinate-compatible field for HoTT. `257` is signed heading error
in degrees. `258` and `259` are the normalized **earth-frame** north and east
correction requests; `250` and `251` remain body-frame surge and sway.
`260` (`DP_CALC`) and `261` (`HDG_CALC`) are **calculation valid** flags, not
actuator authority or mode states. M3.8 can report `DP_CALC=1` even while
`NAV_STATE=HDG_WAIT` and `OUTPUT_NOT_READY` is set.

The source is unavailable (and thus not disguised as zero) when its required
EKF estimate/target/controller output is invalid. `NAV_RBITS` remains source
`141`; do not create a second DP status mask. Estimator resets clear captured
targets instead of attempting automatic re-engagement.

Receiver-driven VTEL takes 10 Hz data snapshots in the existing application
schedule. `DP_TEL_HZ` (0..50 Hz) controls the separate MAVLink
`NAMED_VALUE_FLOAT` diagnostic stream, not the physical receiver poll rate.
At high DP telemetry rates the available MAVLink link bandwidth must be
checked; output is best effort. **Neither transport conveys thrust actually
produced by the vessel.**
