# Vektor2 baseline

Vektor2 is a deliberately small ArduPilot-library application. It uses the
ArduPilot hardware/sensor ecosystem, but it is **not** an ArduPilot vehicle.
There are no flight modes, mission system, vehicle arming architecture, or
vehicle failsafe state machine.

This baseline intentionally uses **standard MAVLink only**. There is no Vektor
MAVLink dialect, no private extension payload, no descriptor protocol, and no
second serial protocol.

## Included

- AP_HAL / AP_BoardConfig
- AP_InertialSensor
- optional AP_Compass, with standard `COMPASS_*` parameters
- AP_AHRS with EKF3 selected
- AP_GPS receiver detection and GPS-aided EKF3 position estimation
- AP_SerialManager
- AP_RCProtocol + `hal.rcin`
- direct `hal.rcout` PWM
- standard ArduPilot MAVLink/GCS parameter and telemetry infrastructure
- a fixed 32-slot microsecond routing table
- two fixed logic components: VSP1 and VSP2

## Deliberately absent

- AP_Vehicle
- AP_Arming
- flight modes
- missions
- vehicle failsafe architecture
- RC_Channel application logic
- SRV_Channels servo-function model
- custom MAVLink messages
- the original Vektor serial protocol

A private `SRV_Channels` object still exists only because current ChibiOS
`RCOutput::init()` consults that registry during hardware initialization.
Vektor2 does not initialize or use the servo-function architecture.

## Runtime loop

```text
wait for IMU sample
      |
      v
update INS
update GPS and look for a receiver
update RC input
      |
      v
update AHRS / EKF3
      |
      v
apply changed RTn_SRC / RTn_DST parameters
      |
      v
run router + VSP components
      |
      v
write PWM
      |
      v
receive/send standard MAVLink
```

The IMU clocks the application at 100 Hz by default.

## GPS and EKF3

On Revo Mini, GPS1 uses `SERIAL3` / USART3 by default. Connect both signal
wires: GPS TX -> PB11 (FC RX), GPS RX <- PB10 (FC TX), plus power and ground.
Vektor2 defaults to `SERIAL3_PROTOCOL=5`, `GPS1_TYPE=1` (AUTO),
`GPS_AUTO_CONFIG=1`, `GPS_DRV_OPTIONS=0`, and `GPS1_DELAY_MS=120`.
Receiver detection and baud selection use the same AP_GPS defaults as Rover:
the driver probes the available baud rates and configures u-blox at 230400
baud. `SERIAL3_BAUD` is the initial probe rate, not a fixed operating rate.
Other receiver types and ports remain configurable with the standard
parameters; reboot after changing the serial role or GPS type. The nonzero
GPS delay lets EKF3 allocate its observation buffer before a receiver is
detected, so attitude can initialize from the IMU alone. A saved zero delay
from older firmware is treated as 120 ms at boot; another nonzero value is
preserved.

The M10 SPG 5.10 UART1 factory default is 38400 baud with NAV-PVT output
disabled; a module may have different saved settings. The shared AP_GPS u-blox
startup sequence now asks older receivers for NAV-SOL and M9/M10 receivers
for NAV-PVT on UART1 in RAM, then sends `PUBX,41` to select UBX output and
the target baud. The RAM request does not write receiver flash or BBR. The
legacy NAV-SOL request stays for M8 and older receivers. AUTO (`GPS1_TYPE=1`)
does not automatically fall back to NMEA; select `GPS1_TYPE=5` for a pure
NMEA receiver. AP_GPS detects and reports a receiver independently of a
position fix.

Saved parameter values survive reflashing and override these compiled
defaults. When upgrading from the forced u-blox/115200 setup, set
`GPS1_TYPE=1`, `GPS_AUTO_CONFIG=1`, and `GPS_DRV_OPTIONS=0`, then reboot.
Check these effective values in the startup `STATUSTEXT` or parameter list.
The connected port must have `SERIALx_PROTOCOL=5`; GPS1 uses the first port
configured for GPS in serial-number order.

EKF3 uses GPS for horizontal position and velocity and vertical velocity.
Height is synthetic because this firmware has no barometer and does not
require altitude:

```text
EK3_SRC1_POSXY = 3  (GPS)
EK3_SRC1_VELXY = 3  (GPS)
EK3_SRC1_POSZ  = 0  (synthetic zero height)
EK3_SRC1_VELZ  = 3  (GPS)
EK3_SRC1_YAW   = 1  (compass yaw, gyro propagation)
```

With an installed compass, EKF3 keeps source set 1 and its compass yaw.
If no compass is detected at startup, Vektor2 selects source set 2 with the
the same GPS position and velocity sources, synthetic height, and
`EK3_SRC2_YAW=8` (GSF).
Source set 2 is reserved for this fallback; its active values are restored
at boot if older firmware saved the unaided values. Both height sources are
set to synthetic height in RAM at boot, even if older firmware saved GPS
height. Saved parameters and compass yaw settings stay unchanged. Because
Vektor2 has no arming state, it signals expected movement to EKF3 while a
compass-less vehicle has a 3D GPS fix and moves at least 1 m/s. EKF3 decides
when the GPS/IMU GSF yaw estimate is accurate enough to align yaw and fuse
GPS position. Without a compass or GPS motion, EKF3 can report a valid
roll/pitch attitude after tilt alignment, but absolute yaw and position
cannot be observed. A receiver can report a raw 3D fix before EKF3 has a
fused position.

Compass calibration uses ArduPilot's standard `MAV_CMD_DO_START_MAG_CAL`,
`MAV_CMD_DO_ACCEPT_MAG_CAL`, and `MAV_CMD_DO_CANCEL_MAG_CAL` handlers. The
compass is sampled and the calibration task is updated at 10 Hz. Calibration
requires a detected, healthy compass but does not require a GPS fix. The DCM
attitude component is included because ArduPilot's compass calibrator uses it
for attitude samples.

Vektor2 reports the first five GPS1 baud changes and the effective GPS
settings in `STATUSTEXT`. While no backend is detected, a warning every 15
seconds shows the current baud and copied UART traffic as `no RX`, `NMEA`,
`UBX`, `NMEA+UBX`, or `unknown`. The UART tap observes bytes after AP_GPS reads
them; it never consumes parser input. `RX=NMEA` with no backend suggests a
protocol/configuration issue or an unconnected FC TX wire. `no RX` suggests
wiring, power, UART assignment, or baud problems. `GPS_RAW_INT` reports the
receiver and fix independently of EKF validity. If EKF3 remains
uninitialized, a periodic `EKF3 waiting` message reports whether GPS timing
is known and how much free memory remains. If EKF3 initializes but attitude
is still invalid, `EKF3 attitude pending` reports the detected compass count
and GPS status.

### GPS bench checks

- **A — factory M10:** Confirm initial NMEA at its UART1 baud; verify the
  legacy CFG-MSG and RAM-only NAV-PVT VALSET precede PUBX,41. After the switch,
  expect periodic UBX-NAV-PVT and a detected u-blox backend.
- **B — M8:** Confirm the legacy NAV-SOL request still produces a detectable
  stream; the extra VALSET should be harmless if unsupported.
- **C — RX-only wiring:** Disconnect FC TX (PB10) while retaining GPS TX to
  PB11. Expect an RX classification, but AP_GPS may fail to configure and
  detect the receiver. Restore both wires for normal operation.
- **D — pure NMEA:** Set `GPS1_TYPE=5` and `GPS_AUTO_CONFIG=0` on a factory
  receiver. Expect NMEA detection; this checks FC RX and port assignment
  independently of the u-blox wake-up sequence. Restore the u-blox settings
  afterward.

GPS and EKF settings remain ordinary ArduPilot parameters. Saved compass
yaw settings are left intact.

## Serial ports

UART roles and baud rates are normal ArduPilot parameters:

```text
SERIAL0_PROTOCOL
SERIAL0_BAUD
SERIAL1_PROTOCOL
SERIAL1_BAUD
...
```

Typical protocol values used by this baseline are:

```text
2   MAVLink2
5   GPS
23  RC input
```

Vektor2 does not invent a generic UART-connected state. Runtime health belongs
to the protocol using the port.

## RC input

`RcInput` reads the HAL RC input snapshot. On ChibiOS, the dedicated RC input
thread runs AP_RCProtocol and decodes the serial receiver. A serial receiver
can be attached by setting the relevant `SERIALx_PROTOCOL` to `23` (RCIN),
then rebooting. For the Revo Mini board definition in this tree, the mapping is:

| Parameter | Physical interface |
| --- | --- |
| `SERIAL0` | USB |
| `SERIAL1` | USART1, PA10 RX / PA9 TX |
| `SERIAL2` | Unused slot |
| `SERIAL3` | USART3, PB11 RX / PB10 TX |
| `SERIAL4` | USART6, PC7 RX / PC6 TX |

`SERIAL1_PROTOCOL` defaults to `23` to receive RC on USART1 RX.
`SERIAL0_PROTOCOL` defaults to `2` for USB MAVLink. Only one serial port can
have RCIN enabled at a time. AP_RCProtocol scans the supported receiver baud
rates and framing, so `SERIAL1_BAUD` does not select the RC wire speed.

Every five seconds, Vektor2 sends ArduPilot's MAVLink
`NAMED_VALUE_STRING` with key `RC_PROTO` and the decoded protocol name as
its value (for example, `CRSF`). The value is `NONE` when there is no fresh
valid RC input. This is an ArduPilot MAVLink dialect message, so the receiver
must support `NAMED_VALUE_STRING`.

ArduPilot's AP_RCProtocol also produces its own debug detection announcement.
Realtime channel values use the standard `RC_CHANNELS` message. Vektor2 sends
`UINT16_MAX` for unused channels and reports zero channels once the local RC
snapshot has timed out.

No RC_Channel mapping, aux-switch logic or vehicle RC failsafe is used.

## PWM output

`PwmOut` talks directly to `hal.rcout` and retains only the most recently
written pulse width for telemetry. Vektor2 has no safety or arming workflow;
startup releases the HAL RCOutput safety gate so configured routes can produce
physical PWM pulses.

The standard MAVLink `SERVO_OUTPUT_RAW` message reports the first 16 output
values in microseconds. Disabled outputs are reported as zero. The application
router supports up to 32 PWM endpoint numbers, but the selected board remains
authoritative about how many outputs physically exist.

Each physical output has persistent `PWMn_MIN`, `PWMn_MAX`, and `PWMn_INV`
parameters (`n` is 1..32). Defaults are 1000 us, 2000 us, and 0. The final
output stage clamps its routed input to 1000..2000 us and scales that range to
`PWMn_MIN`..`PWMn_MAX`; `PWMn_INV=1` swaps the output endpoints. These settings
apply to the physical output regardless of its route. Both endpoints must be
within 500..2500 us, `MIN` must be below `MAX`, and `INV` must be 0 or 1.
Invalid settings disable that output until corrected. `SERVO_OUTPUT_RAW`
reports the scaled pulse width.

## Standard MAVLink surface

The intended desktop interface is deliberately small:

```text
HEARTBEAT          device presence, reported as a surface boat
RAW_IMU            accelerometer, gyro, and optional compass samples
ATTITUDE           EKF attitude
RC_CHANNELS        RC values in us
SERVO_OUTPUT_RAW   PWM values in us
GPS_RAW_INT        receiver status and raw GPS position
GLOBAL_POSITION_INT EKF3 position when available
STATUSTEXT         status/events such as RC protocol detection
PARAM_VALUE/SET    all configuration
```

The raw sensor and attitude streams default to 10 Hz. The extended-status
and position streams default to 1 Hz, including `GPS_RAW_INT` and
`GLOBAL_POSITION_INT`. Normal ArduPilot message scheduling and
`MAV_CMD_SET_MESSAGE_INTERVAL` remain authoritative.
IMU health and sensor counts are reported through `STATUSTEXT` after startup
and when health changes. There is no Vektor telemetry scheduler.

A GCS can send `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` in `COMMAND_LONG` or
`COMMAND_INT` to reboot Vektor2. Set parameter 1 to `1` for a normal reboot or
`3` to reboot and hold in the bootloader. Other parameter 1 values receive
`MAV_RESULT_UNSUPPORTED`. Accepted requests receive `COMMAND_ACK` before the
reset. Vektor2 has no arming state, so an accepted command also interrupts
active PWM routing; the ChibiOS reboot path engages output safety before reset.

Vektor2 enables GCS and defaults `SERIAL0_PROTOCOL` to MAVLink2, including
on AP_Periph-style boards whose generic defaults disable both. The selected
board must expose the intended serial port; saved `SERIALx_PROTOCOL` values
can override this default.

## Logic components

Two fixed components are present:

```text
VSP1: 3 inputs (IN1, IN2, RPM_IN), 3 outputs (OUT1, OUT2, RPM_OUT), prefix VSP1
VSP2: 3 inputs (IN1, IN2, RPM_IN), 3 outputs (OUT1, OUT2, RPM_OUT), prefix VSP2
ThrusterBow: 1 input, 1 output, prefix THR_BOW
ThrusterStern: 1 input, 1 output, prefix THR_STN
```

All routed values are `uint16_t` microseconds.
Both thruster components currently pass their inputs through unchanged; their parameters are reserved for future control logic.

Each VSP builds a small stack-allocated `Limiter` from its current
parameters on every cycle, then uses `mapCoordinates()` for the first two
outputs. `RPM_IN` passes directly to `RPM_OUT`. Coordinate results are clamped
to the `uint16_t` output range before routing. The default `LIM=25` constrains
coordinates to a radius of 25 microseconds around `(1500, 1500)`.

The component may later read existing ArduPilot services such as `AP::ahrs()`,
`AP::gps()` or `AP::ins()` when it needs read-only system data. Routing,
persistence, MAVLink and hardware writes stay outside the component function.

### VSP parameters

Both components expose normal persistent AP_Param values. The direction and configuration values range from 0 to 10:

```text
VSP1_LIM = 25
VSP1_X_C = 1500
VSP1_Y_C = 1500
VSP1_DIR = 0
VSP1_THR_ANG = 0

VSP2_LIM = 25
VSP2_X_C = 1500
VSP2_Y_C = 1500
VSP2_DIR = 0
VSP2_THR_ANG = 0
VSP_CONF = 0
VSP_TEL_HZ = 5
THR_BOW_MID = 1500
THR_BOW_DST = 0
THR_STN_MID = 1500
THR_STN_DST = 0
THR_TEL_HZ = 0
```

`VSP1_THR_ANG` and `VSP2_THR_ANG` default to 0 degrees. Their values
are clamped to 0..359 before being supplied as signed 16-bit values to their
respective component loops. They do not change the current VSP output
algorithm until the loops use them.

`VSP_TEL_HZ` controls each VSP named-value telemetry key in Hz. Set it to
`0` to disable those messages. Values above 50 are limited to 50 Hz.

Thruster parameters share the `THR_` prefix: `THR_BOW_*`, `THR_STN_*`,
and `THR_TEL_HZ`. The bow and stern parameter storage IDs are unchanged,
so existing saved values survive the prefix rename.

`THR_TEL_HZ` controls both thruster named floats independently of VSP telemetry.
It defaults to `0` (disabled), with rates above 50 limited to 50 Hz.
After routing, `THRBOW_PWM` and `THRSTN_PWM` report the bow and stern
component output pulse widths in microseconds, before physical-output scaling.
These wire names fit MAVLink's 10-character named-value limit.

Each VSP emits two MAVLink `NAMED_VALUE_FLOAT` values after the routing cycle:
`VSP1_XA` and `VSP1_YA` for VSP1, and `VSP2_XA` and `VSP2_YA` for VSP2.
These are the component's first two output pulse widths in microseconds.
`Telemetry.cpp` owns the shared rate limit and transport; the named values
for each component are declared in its own `.cpp` file. Sending is skipped
when a MAVLink link has no payload space, without delaying a component loop.

Inside `VSP1.cpp` they are simply:

```cpp
params.lim
params.x_c
params.y_c
```

VSP2 is identical with its own independent parameter store.

## Routing

The runtime has 32 route slots. A source may fan out to any number of
consumers. A consumer may have only one producer.

Valid examples:

```text
RC1 --------> PWM1
   +--------> PWM2

RC2 --------> VSP1.IN1
VSP1.OUT1 --> PWM3
VSP1.OUT1 --> VSP2.IN1
```

Invalid:

```text
RC1 --+
      +--> PWM1
RC2 --+
```

Component dependency cycles are rejected, so this is invalid:

```text
VSP1 -> VSP2 -> VSP1
```

The router computes component execution order when configuration changes, not
on every realtime cycle.

### Route parameters

Routing is configured through ordinary MAVLink parameters only:

```text
RT1_SRC
RT1_DST
...
RT32_SRC
RT32_DST
```

A route is inactive if either value is `0`.

Source encoding:

```text
1..18    RC1..RC18
101      VSP1 output 1
102      VSP1 output 2
103      VSP1 RPM_OUT
111      VSP2 output 1
112      VSP2 output 2
113      VSP2 RPM_OUT
121      ThrusterBow output
131      ThrusterStern output
```

Destination encoding:

```text
1..32    PWM1..PWM32
101      VSP1 input 1
102      VSP1 input 2
103      VSP1 RPM_IN
111      VSP2 input 1
112      VSP2 input 2
113      VSP2 RPM_IN
121      ThrusterBow input
131      ThrusterStern input
```

Examples:

```text
RT1_SRC = 1
RT1_DST = 101
```

means:

```text
RC1 -> VSP1.IN1
```

and:

```text
RT2_SRC = 101
RT2_DST = 3
```

means:

```text
VSP1.OUT1 -> PWM3
```

The standard parameter-backed routes use direct microsecond mapping with the
existing route defaults of 1000..2000 us. Programmatic routes still retain the
small clamp/map/reverse capability already present in `Route` if it is ever
needed locally.

Route parameters are checked every 100 ms. A changed configuration rebuilds
the tiny runtime table and releases old PWM consumers. Duplicate consumers,
invalid endpoints and component cycles are rejected. Vektor2 emits a standard
`STATUSTEXT` warning while the parameter configuration is invalid.

For safe live editing, clear a route by setting either endpoint to `0`, update
the other endpoint, then set the final endpoint.

## Application structure

```text
Vektor2.cpp/.h             composition root + main loop
Parameters.cpp/.h          top-level AP_Param registration
GCS_MAVLink.cpp/.h         minimal standard-MAVLink backend
RcInput.cpp/.h             RC snapshot
PwmOut.cpp/.h              direct PWM output
Routing.cpp/.h             realtime route engine
RouteParameters.cpp/.h     RT1..RT32 standard parameter binding
Logic.cpp/.h               component registry/runtime
ComponentParameters.*      AP_Param-backed VSP parameters
VSP1.cpp/.h                VSP1 algorithm and telemetry values
VSP2.cpp/.h                VSP2 algorithm and telemetry values
Telemetry.cpp/.h           shared rate limit and MAVLink publisher
Config.h                   small application constants
wscript                    ArduPilot build target
```

## Build integration

Place `Vektor2/` into the ArduPilot tree using the same custom-application
integration point used by your existing Vektor target. The provided `wscript`
builds the `Vektor2` program.

Conceptually:

```sh
./waf configure --board <board>
./waf --targets bin/Vektor2
```

Validate a clean target build for the chosen board, then verify boat heartbeat,
RAW_IMU, ATTITUDE, optional compass data, RCIN, and safe bench PWM behavior.

## Navigation parameters

ArduPilot limits parameter names to 16 characters. The navigation parameter
group uses the `VNAV_` prefix; long requested names are exposed as follows:

| Requested name | Firmware name |
| --- | --- |
| `VNAV_HDG_DEADBAND` | `VNAV_HDG_DBAND` |
| `VNAV_HDG_STICK_RATE` | `VNAV_HDG_STK_RT` |
| `VNAV_DP_CAPTURE_POS` | `VNAV_DP_CAP_POS` |
| `VNAV_DP_CAPTURE_HDG` | `VNAV_DP_CAP_HDG` |
| `VNAV_DP_MAX_SPEED` | `VNAV_DP_MAX_SPD` |
| `VNAV_DP_ACCEL_LIMIT` | `VNAV_DP_ACC_LIM` |
| `VNAV_DP_POS_FILTER` | `VNAV_DP_POS_FILT` |
| `VNAV_DP_VEL_FILTER` | `VNAV_DP_VEL_FILT` |
| `VNAV_DP_YAW_PRIORITY` | `VNAV_DP_YAW_PRI` |
| `VNAV_ALLOC_W_SURGE` | `VNAV_AL_W_SURGE` |
| `VNAV_ALLOC_W_SWAY` | `VNAV_AL_W_SWAY` |
| `VNAV_ALLOC_MAX_SURGE` | `VNAV_AL_MAX_SRG` |
| `VNAV_ALLOC_MAX_SWAY` | `VNAV_AL_MAX_SWAY` |
| `VNAV_ALLOC_MAX_YAW` | `VNAV_AL_MAX_YAW` |
| `VNAV_ALLOC_SURGE_SLEW` | `VNAV_AL_SURGE_SL` |
| `VNAV_ALLOC_SWAY_SLEW` | `VNAV_AL_SWAY_SL` |
| `VNAV_ALLOC_YAW_SLEW` | `VNAV_AL_YAW_SL` |
| `VNAV_MANUAL_OVERRIDE` | `VNAV_MAN_OVR` |
| `VNAV_MANUAL_DEADBAND` | `VNAV_MAN_DBAND` |
| `VNAV_OVERRIDE_MODE` | `VNAV_OVR_MODE` |
| `VNAV_FAILSAFE_MODE` | `VNAV_FAIL_MODE` |
