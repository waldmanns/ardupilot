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
- AP_GPS compiled in for later use, disabled by configuration initially
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
update GPS (inactive until configured)
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

## EKF / future GPS

EKF3 is selected from the start. The default EKF source configuration is
explicitly unaided:

```text
EK3_SRC1_POSXY = 0
EK3_SRC1_VELXY = 0
EK3_SRC1_POSZ  = 0
EK3_SRC1_VELZ  = 0
EK3_SRC1_YAW   = 0
```

GPS is already part of the object graph. Later, configure a serial port for
GPS, select the receiver type, and set the appropriate EKF source parameters.
No application architecture change is required.

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

For example, set `SERIAL1_PROTOCOL=23` to receive RC on USART1 RX.
Keep `SERIAL0_PROTOCOL=2` for USB MAVLink. Only one serial port can have
RCIN enabled at a time. AP_RCProtocol scans the supported receiver baud rates
and framing, so `SERIAL1_BAUD` does not select the RC wire speed.

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
GPS_RAW_INT        GPS when enabled
STATUSTEXT         status/events such as RC protocol detection
PARAM_VALUE/SET    all configuration
```

The raw sensor and attitude streams default to 10 Hz. Normal ArduPilot
message scheduling and `MAV_CMD_SET_MESSAGE_INTERVAL` remain authoritative.
IMU health and sensor counts are reported through `STATUSTEXT` after startup
and when health changes. There is no Vektor telemetry scheduler.

Vektor2 enables GCS and defaults `SERIAL0_PROTOCOL` to MAVLink2, including
on AP_Periph-style boards whose generic defaults disable both. The selected
board must expose the intended serial port; saved `SERIALx_PROTOCOL` values
can override this default.

## Logic components

Two fixed components are present:

```text
VSP1: 3 inputs (IN1, IN2, RPM_IN), 3 outputs (OUT1, OUT2, RPM_OUT), prefix VSP1
VSP2: 3 inputs (IN1, IN2, RPM_IN), 3 outputs (OUT1, OUT2, RPM_OUT), prefix VSP2
```

All routed values are `uint16_t` microseconds.

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

VSP2_LIM = 25
VSP2_X_C = 1500
VSP2_Y_C = 1500
VSP2_DIR = 0
VSP_CONF = 0
```

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
VSP1.cpp/.h                VSP1 algorithm only
VSP2.cpp/.h                VSP2 algorithm only
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
