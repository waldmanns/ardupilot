# Vektor2 baseline

Vektor2 is a deliberately small ArduPilot-library application. It uses the
ArduPilot hardware/sensor ecosystem, but it is **not** an ArduPilot vehicle.
There are no flight modes, mission system, vehicle arming architecture, or
vehicle failsafe state machine.

This baseline keeps **standard ArduPilot MAVLink** for ground-control
communications. It adds optional receiver-facing **i-BUS and HoTT** telemetry
through VTEL. There is no custom Vektor MAVLink dialect, private extension
payload, descriptor protocol or proprietary Vektor serial protocol.

## Included

- AP_HAL / AP_BoardConfig
- AP_InertialSensor
- optional AP_Compass, with standard `COMPASS_*` parameters
- AP_AHRS with EKF3 selected
- AP_GPS receiver detection and GPS-aided EKF3 position estimation
- AP_SerialManager
- AP_RCProtocol + `hal.rcin`, with standard RC_Channels MAVLink overrides
- direct `hal.rcout` PWM
- standard ArduPilot MAVLink/GCS parameter and telemetry infrastructure
- SD/filesystem binary logging, MAVFTP and native MAVLink log transfers
- a fixed 32-slot microsecond routing table
- routed VSP, AZI and tunnel thruster components
- a [sequencer](VSEQ.md) with onboard parameter storage and optional boot playback
- configurable [VTEL telemetry builder](VTEL.md): typed source catalog, 65
  numeric `VTEL_SLTn` assignments, i-BUS virtual sensors and HoTT EAM/GPS/VARIO
  virtual modules

## Custom receiver telemetry: VTEL

VTEL presents the firmware's existing measurements as stable numeric source IDs.
Each `VTEL_SLTn` parameter assigns one source ID to an i-BUS virtual sensor
address or a fixed HoTT module field. Values default to the closest standard
measurement available (GPS speed, course, position and altitude, AHRS attitude,
measured RPM, ADC voltages). There is no physical-quantity restriction on
numeric reassignments: a rod position can deliberately occupy a HoTT
`temperature` display field. Structurally invalid assignments are ignored.

The **[VTEL README](VTEL.md)** documents physical bus topology and wiring,
i-BUS discovery and sensor addresses, HoTT polling and **EAM/GPS/VARIO module
emulation**, exact slot/source catalogs, default mappings, and unsupported HoTT
module types such as GAM. Configure `SERIALx_PROTOCOL=49` for i-BUS telemetry
or `SERIALx_PROTOCOL=27` for HoTT, on a suitable receiver-connected UART.
The catalog includes the active propulsion family, its layout and AZI commissioning
commands and reason masks; inactive propulsion sources are unavailable.
The backend reads a 10 Hz snapshot and handles serial polling outside the
100 Hz control path. Hardware verification is still required.

## Deliberately absent

- AP_Vehicle
- AP_Arming
- flight modes
- missions
- vehicle failsafe architecture
- periodic RC auxiliary-switch processing
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
apply changed RTn_SRC / RTn_DST parameters and sequencer commands
      |
      v
advance the sequencer and run the router and logic components
      |
      v
write PWM
      |
      v
receive/send standard MAVLink
      |
      v
bounded logger maintenance and sample buffering
```

The IMU clocks the application at 100 Hz by default.
SD writes, capacity polling and recording-stop file closure run on background
IO threads. Logging maintenance has one small scheduler task after control/PWM.

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

With a detected compass enabled for yaw, EKF3 starts with source set 1 and
its compass yaw. When no compass is enabled for yaw at startup, Vektor2
selects source set 2 with the same GPS position and velocity sources,
synthetic height, and `EK3_SRC2_YAW=8` (GSF).
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

## Compass setup on Vektor Core EVO

The `MatekH743` Vektor2 build uses ArduPilot's `AP_Compass` drivers:

| Compass | Connection | Detection |
| --- | --- | --- |
| Onboard MMC5983MA (U3) | I²C1, PB8 SCL / PB9 SDA, address `0x30` | Explicit `MMC5XX3` probe, device ID `1257473` |
| External I²C compass (for example IST8310 or QMC5883P) | GNSS/I²C connector, PB10 SCL / PB11 SDA, plus module power and ground | Automatic ArduPilot probing on I²C2 |
| External DroneCAN compass | Configured [DroneCAN port](#can-and-routed-pwm) | Standard ArduPilot DroneCAN discovery |

For the onboard sensor, check continuity from U3 ground pins 9 and 11 to
board ground. The schematic referenced by the board definition shows those
pins without a ground connection. The onboard rotation
(`ROTATION_ROLL_180_YAW_90`) assumes U3 is top-side up with the edge above
the `5983` marking (pins 1–4) toward the bow. Recalibrate the onboard compass
after updating from a build that used `ROTATION_ROLL_180_YAW_270`. For an
external compass, use the connector's I²C lines even if the compass is part
of a GPS module that also connects to the GPS UART.

1. Set `COMPASS_ENABLE=1`, `COMPASS_DISBLMSK=0` and `EK3_SRC1_YAW=1`, then
   reboot with the compasses connected. The ground station lists detected
   compasses by device ID (`COMPASS_DEV_ID`, `COMPASS_DEV_ID2`,
   `COMPASS_DEV_ID3`). The corresponding `COMPASS_EXTERNAL`,
   `COMPASS_EXTERN2` and `COMPASS_EXTERN3` fields identify external
   compasses. If the onboard ID is absent, check U3 wiring or use an external
   compass.
2. In the ground station's compass setup, move the chosen compass to
   priority 1 (`COMPASS_PRIO1_ID`), set `COMPASS_USE=1` and, for a single
   heading source, set `COMPASS_USE2=0` and `COMPASS_USE3=0`. Reboot after
   changing priority. Set the external compass orientation to match its
   installation if it differs from the detected default. Compasses disabled
   for yaw remain visible and continue reporting samples.
3. Run onboard compass calibration through the ground station, accept the
   result and reboot. On the bench, turn the vessel through the cardinal
   headings and confirm that the indicated heading follows the bow. Check
   that heading stays consistent when rolling or pitching the vessel.

Vektor2 updates the compass frontend and its calibration task at 10 Hz.
Calibration uses ArduPilot's `MAV_CMD_DO_START_MAG_CAL`,
`MAV_CMD_DO_ACCEPT_MAG_CAL` and `MAV_CMD_DO_CANCEL_MAG_CAL` handlers and
requires a healthy compass. With the default `COMPASS_OPTIONS=0`, you can
calibrate before GPS acquires a fix. In the MAVLink inspector, the
magnetometer fields in `RAW_IMU`, `SCALED_IMU2` and `SCALED_IMU3` show
samples for compass priorities 1, 2 and 3, respectively.

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

## Board status LED

On boards declaring `AP_NOTIFY_GPIO_LED_1_PIN`, Vektor2 lights the status LED
during startup, then blinks it once per second (500 ms on, 500 ms off) from
the control loop. This indicates that the application loop is running.
On the BMI088 Vektor Core EVO board this is the active-high PA10 LED,
HAL GPIO 90. The LED does not depend on a GPS fix or a ground-station link.

## ADC measurement points

On the BMI088 Vektor Core EVO definition stored under `MatekH743`,
`MP_TEL_HZ` controls measurement-point telemetry independently of other
streams. The default `0` disables it. Set `1` through `50` for the requested
rate per point in Hz; changes take effect without rebooting. Negative values
disable it and values above 50 are limited to 50 Hz. Boards without measurement
point declarations send no MP values.

Each point is a standard MAVLink `NAMED_VALUE_FLOAT`, with the measured rail
voltage in volts after divider correction. Values are verified against
`vektor-core-evo(20261111-182425).kicad_sch`:

| Key | Measured rail | MCU input | Divider (top/bottom) | Scale |
| --- | --- | --- | --- | --- |
| `MP_V_VCC` | Servo VCC | PC4 / ADC1_INP4 | R8/R7: 56k/10k | 6.6 |
| `MP_V_V5` | +5 V | PC5 / ADC1_INP8 | R23/R22: 56k/10k | 6.6 |
| `MP_V_VCAN` | CAN power | PA4 / ADC1_INP18 | R29/R28: 56k/10k | 6.6 |
| `MP_V_BAT` | Battery | PC2_C / ADC3_INP0 | R27/R26: 100k/10k | 11 |

All four fields in a snapshot share `time_boot_ms`. A full link retries
unsent fields between snapshots; telemetry never blocks control/PWM. A point
whose analog source could not be allocated reports NaN. These are voltage
measurement points; they do not enable ArduPilot battery monitoring.

## Shaft RPM inputs

On the Vektor Core EVO (`MatekH743`), PA2 and PA3 use the board's external
pull-ups. The firmware measures the interval between falling edges, assuming
one magnet pulse per revolution. It reports zero until it measures a full
revolution and after pulses stop.

`VRPM_TEL_HZ` defaults to `0`, which disables transmission. Set it to 1–50 Hz
to publish each RPM, magnet and governor status value at that rate as a MAVLink
`NAMED_VALUE_FLOAT` message.
Changes take effect without rebooting.

| MAVLink key | MCU pin | Value |
| --- | --- | --- |
| `VRPM1_RPM` | PA2 | RPM |
| `VRPM2_RPM` | PA3 | RPM |
| `VRPM1_IRQ` | PA2 | Magnet detected, 0 or 1 |
| `VRPM2_IRQ` | PA3 | Magnet detected, 0 or 1 |

Each `VRPMx_IRQ` value is `1` while its input is low (magnet detected) and `0`
while it is high. The input level is sampled at the telemetry rate.

### VSP RPM governor

VSP1 uses RPM1 (PA2) and VSP2 uses RPM2 (PA3). Route the knob or switch to
`RPM_IN` (destination `103` for VSP1 or `113` for VSP2). Both drives use the
same RPM range and tuning. An RC channel can feed both routes for a shared
command, or each drive can have its own command.

| Parameter | Setting |
| --- | --- |
| `VRPM_ENABLE` | `0` (default) passes each `RPM_IN` directly to `RPM_OUT`. `1` uses that input as the governor command. |
| `VRPM_RPM_LOW` | Target shaft RPM at 1500 µs. Set a positive value. |
| `VRPM_RPM_HIGH` | Target shaft RPM at 2000 µs. Set a value at least as high as `VRPM_RPM_LOW`. |
| `VRPM_DB` | Deadband on either side of the target in RPM. The default is ±2 RPM. |
| `VRPM_SLEW_FAST` | Throttle rate at large RPM errors. Default 30 µs/s. |
| `VRPM_SLEW_SLOW` | Throttle rate at the near-target boundary. Default 10 µs/s. Set it no higher than `VRPM_SLEW_FAST`. |
| `VRPM_NEAR_RPM` | Error where the slow rate applies. Default 50 RPM. Set it above `VRPM_DB`. |
| `VRPM_START_MAX` | Fixed spin-up pulse applied immediately. Default 1200 µs. |
| `VRPM_START_TMO` | Maximum time to hold the spin-up pulse, in seconds. Default 10 s. |

In governor mode, 1480 µs and below outputs 1000 µs to stop the motor.
1481–1500 µs selects the low RPM target. From 1500 to 2000 µs the target
changes linearly from low to high. With endpoints of 200 and 400 RPM,
1750 µs requests 300 RPM.

The throttle rate tapers linearly from the slow rate at `VRPM_NEAR_RPM` to
zero at the deadband. Farther out it blends to the fast rate at twice
`VRPM_NEAR_RPM`. Inside the deadband the drive holds its throttle pulse.
Higher logical throttle pulses must increase shaft speed.

If the pot passes 1480 µs before valid shaft RPM is available, the drive sends
`VRPM_START_MAX` immediately. It holds that pulse while waiting for feedback.
Once a valid reading arrives, the governor starts slewing from the current
output toward the target RPM. If `VRPM_START_TMO` expires first, or if feedback
is lost while regulating, the drive commands 1000 µs. Lower the knob to 1480 µs
or below to rearm. A missing RPM input route or an unavailable RPM sensor also
commands a stop. Shaft RPM measurements continue with `VRPM_TEL_HZ=0`.

With VSP propulsion selected, `VRPM1_STAT` and `VRPM2_STAT` report the
governor bitmask for VSP1 and VSP2 at `VRPM_TEL_HZ`. Decode the float value
as an unsigned integer.

| Bit | Value | Meaning when set |
| --- | --- | --- |
| 0 | 1 | Governor enabled by `VRPM_ENABLE`. |
| 1 | 2 | This drive has a valid routed `RPM_IN` control pulse. |
| 2 | 4 | The control pulse requests RPM hold (above 1480 µs). |
| 3 | 8 | This drive has valid shaft RPM feedback. |
| 4 | 16 | This drive has a valid routed `RPM_IN` pulse. |
| 5 | 32 | The governor is controlling this drive's `RPM_OUT`. |
| 6 | 64 | Active and within the RPM deadband. |
| 7 | 128 | Active and the output pulse is at a throttle endpoint (1000 or 2000 µs). |
| 8 | 256 | The governor settings are valid. |
| 9 | 512 | Starting the shaft while waiting for RPM feedback. |
| 10 | 1024 | Stopped after a fault. Lower `RPM_IN` to 1480 µs or below to rearm. |

## RC input

`RcInput` uses ArduPilot’s `RC_Channels` to read the HAL receiver and apply
standard MAVLink RC overrides. On ChibiOS, the dedicated RC input
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

Routing consumes raw channel pulse widths; RC calibration does not remap routes.
No flight-mode switch or periodic auxiliary-switch processing is used.

### Native ExpressLRS MAVLink

The standard ArduPilot MAVLink handler accepts `RC_CHANNELS_OVERRIDE` and
applies `MAV_GCS_SYSID` filtering, channel ignore/release semantics,
`RC_OPTIONS`, and `RC_OVERRIDE_TIME`. The latter defaults to 3 seconds;
`0` disables overrides and `-1` disables their timeout. Direct receiver
input keeps Vektor2's 500 ms timeout. `RC_PROTO` reports `MAVLink` while an
accepted override is active. The upstream handler currently accepts channels
1–16. Existing routing IDs work for both serial RC and MAVLink RC input.

Use [ExpressLRS native MAVLink mode](https://www.expresslrs.org/software/mavlink/)
on the transmitter and receiver. On the connected Vektor2 UART set:

```text
SERIALx_PROTOCOL = 2
SERIALx_BAUD = 460
```

Reboot after changing the port settings. `460` selects 460800 baud. Connect
receiver TX to board RX, receiver RX to board TX, and common ground. For a
receiver on Revo Mini USART1, `x` is `1`; USB MAVLink remains on `SERIAL0`.
The default port roles are unchanged, so this setup is opt-in.

The receiver's MAVLink source system ID must match `MAV_GCS_SYSID` (default
255, or its configured range), and its target must match `MAV_SYSID` (default
1). Start with low telemetry rates for the radio link; `VSP_TEL_HZ`,
`THR_TEL_HZ` also consume its bandwidth. Receiver RSSI integration is not
added by this change.

Automated SITL coverage (requires `pymavlink`):

```sh
./waf configure --board sitl
./waf --targets bin/Vektor2
python3 Tools/Vektor2/tests/test_mavlink_rc.py
```

The test uses temporary parameter storage and a second MAVLink serial port.
It checks parameter exchange, RC-to-PWM routing, channel ignore/release,
source/target filtering, timeout with continuing GCS heartbeats, and disabling
overrides using the standard RC parameters.

Bench validation: confirm RC channel values and routed PWM over the link,
read/write a parameter, then stop RC packets and verify that the override
expires after `RC_OVERRIDE_TIME`. A connected GCS heartbeat must not keep
RC overrides alive.

Direct RC-to-PWM routes use the router's existing `HoldLast` policy when their
source becomes unavailable. The retained PWM does not keep RC input valid for
controllers such as heading hold.

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

On the BMI088 Vektor Core EVO board (`MatekH743` build target), the twelve
main outputs follow the connector's mirrored physical order. `1_2`
(`TIM1_CH2`) is PWM1 and `2_1` (`TIM2_CH1`) is PWM12:

| PWM | MCU pin | Timer channel |
| --- | --- | --- |
| 1 | PA9 | TIM1_CH2 |
| 2 | PA8 | TIM1_CH1 |
| 3 | PC9 | TIM8_CH4 |
| 4 | PC8 | TIM8_CH3 |
| 5 | PC7 | TIM8_CH2 |
| 6 | PC6 | TIM8_CH1 |
| 7 | PD15 | TIM4_CH4 |
| 8 | PD14 | TIM4_CH3 |
| 9 | PD13 | TIM4_CH2 |
| 10 | PD12 | TIM4_CH1 |
| 11 | PA1 | TIM2_CH2 |
| 12 | PA0 | TIM2_CH1 |

`BRD_ALT_CONFIG=0` selects PWM12 on PA0. To use that pin as a TX-only UART4,
set `BRD_ALT_CONFIG=1`, set `SERIAL7_PROTOCOL` and `SERIAL7_BAUD` for the
connected device, and reboot. PWM12 is unavailable on the connector in this
mode.

On the FLEX header, PWM13 through PWM16 are on PA6, PA7, PB0 and PB1.
PA2 and PA3 are shaft RPM inputs.
For direct RC passthrough, set `RTn_SRC=n` and `RTn_DST=n` for each channel.
Each RC source must supply valid input before its output starts.

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
STORAGE_INFORMATION SD availability and capacity
FILE_TRANSFER_PROTOCOL standard MAVFTP file access
LOG_ENTRY/LOG_DATA native binary log listing/download
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

Six stable component IDs are registered. `MAIN_PROP_TYPE` selects one main
propulsion family at boot (0 = VSP, default; 1 = AZI). Only the selected
runtime backend is constructed; both tunnel thrusters remain independent.
`AZI_CONF=1` selects twin aft azipods (AZI1 port, AZI2 starboard). AZ2 adds
manual independent azipod control using limited-angle position servos and
forward-only or reversible ESCs. Outputs require valid boot calibration,
a supported layout and paired local PWM routes. The layout alone selects
installed azipods; there are no per-pod enable parameters. Read
[AZIPODS_AZ2_HANDOVER.md](AZIPODS_AZ2_HANDOVER.md) before commissioning;
[AZIPODS_AZ1_HANDOVER.md](AZIPODS_AZ1_HANDOVER.md) records the original architecture.
AZ2.3 adds a temporary visual steering/propeller test through `MAV_CMD_USER_3`.
See [AZIPODS_AZ2_3_HANDOVER.md](AZIPODS_AZ2_3_HANDOVER.md) and the exact
[client widget specification](AZIPODS_AZ2_3_CLIENT.md). AZ2.4 adds per-drive
reason bits and periodic MAVLink diagnostics (`AZI_TEL_HZ`, default 5 Hz,
0 disables, capped at 10 Hz). See [AZ2.4 firmware and client diagnostic contract](AZIPODS_AZ2_4_STATUS.md).

```text
VSP1: 3 inputs (IN1, IN2, RPM_IN), 3 outputs (OUT1, OUT2, RPM_OUT), prefix VSP1
VSP2: 3 inputs (IN1, IN2, RPM_IN), 3 outputs (OUT1, OUT2, RPM_OUT), prefix VSP2
ThrusterBow: 1 input, 1 output, prefix THR_BOW
ThrusterStern: 1 input, 1 output, prefix THR_STN
AZI1: 2 inputs (X_IN, Y_IN), 2 outputs (AZ_OUT, MOT_OUT), prefix AZI1
AZI2: 2 inputs (X_IN, Y_IN), 2 outputs (AZ_OUT, MOT_OUT), prefix AZI2
```

All routed values are `uint16_t` microseconds.
Bow and stern now have independent master/VFN enables, neutral interlocks, manual
conditioning and optional heading assistance. They default disabled and send their
configured neutral when stopped. See [Thrusters.md](Thrusters.md) for configuration
and migration from the old passthrough behavior.

Each VSP builds a small stack-allocated `Limiter` from its current
parameters on every cycle, then uses `mapCoordinates()` for the first two
outputs. The RPM command follows the [VSP RPM governor settings](#vsp-rpm-governor).
Coordinate results are clamped
to the `uint16_t` output range before routing. The default `LIM=25` constrains
coordinates to a radius of 25 microseconds around `(1500, 1500)`.

The component may later read existing ArduPilot services such as `AP::ahrs()`,
`AP::gps()` or `AP::ins()` when it needs read-only system data. Routing,
persistence, MAVLink and hardware writes stay outside the component function.

### VSP parameters

Both components expose normal persistent AP_Param values. Rotation uses
`VSPn_DIR=0` for CW and `1` for CCW, viewed from above.
Layout uses `VSP_CONF=1` for AFT and `2` for VWT:

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
THR_BOW_ENABLE = 0
THR_BOW_MID = 1500
THR_BOW_DST = 0
THR_STN_ENABLE = 0
THR_STN_MID = 1500
THR_STN_DST = 0
THR_TEL_HZ = 0
```

`VSP1_THR_ANG` and `VSP2_THR_ANG` default to 0 degrees. Their values
are clamped to 0..359 before being supplied as signed 16-bit values to their
respective component loops. The current loops apply them as a rod-coordinate angle correction before
heading hold and roll stabilization.

`VSP_TEL_HZ` controls each VSP named-value telemetry key in Hz. Set it to
`0` to disable those messages. Values above 50 are limited to 50 Hz.

Thruster parameters share the `THR_` prefix: `THR_BOW_*`, `THR_STN_*`,
and `THR_TEL_HZ`. The bow and stern parameter storage IDs are unchanged,
so existing saved values survive the prefix rename.

`THR_TEL_HZ` controls thruster output, enable and state named floats independently of VSP telemetry.
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

## Roll stabilization

Optional additive roll bursts are applied after each VSP ring limiter.
`VRS_ENABLE=0` leaves the original outputs and telemetry unchanged. See
[VRS.md](VRS.md) for the controller, 17 parameters, telemetry and tests.

## Heading hold

The retired `VHDG_*` heading mode and telemetry are removed in the
navigation-base revision. `VNAV_MODE=0/1/2` now selects Free, Heading Hold, or
Full DP as **requested modes**. M1 introduced observation-only supervision; M2 adds a host-tested geometry-aware
propulsion allocator; M3 adds the vessel-agnostic `NavigationDP` position/velocity
controller and shared `NavigationHeading` yaw controller with internal reason bits.
M3.5 adds AHRS/EKF yaw, horizontal position and velocity reset handling and
selectable VTEL DP/heading sources 257–259 (`HDG_ERR`, `DP_CMD_N/E`).
M3.8 adds selectable VFN navigation actions 5=Heading/6=DP for 2/3-position
RC switches, plus VTEL sources 260/261 (`DP_CALC`, `HDG_CALC`) to distinguish
valid onboard calculations from real actuator authority.
**Automatic propulsion is still disabled.** `DP_TEL_HZ=0..50` controls MAVLink
DP/heading diagnostic reports. DP-specific reasons merge into the single
`NAV_RBITS` in Full DP mode, while `NAV_STATE` continues to reflect actual
actuator ownership. See [NAVIGATION.md](NAVIGATION.md),
[ALLOCATION_M2.md](ALLOCATION_M2.md), and [NAVIGATION_M2_REVIEW.md](NAVIGATION_M2_REVIEW.md).

## Routed functions

Ten `VFN` slots accept any router source through destinations 201..210.
Each has persistent enable/type/mode and an inclusive 800..2200 us activation
interval. Level or toggle requests gate VRS and navigation heading readiness
on the following control cycle, in addition to their master enables. Multiple requests for
one feature combine with OR. M3.8 navigation selector types 5 and 6 work
as exclusive level requests (Free / Heading / DP) and fail to Free on loss;
they do not command propulsion. Slots default unused/disabled. See [VFN.md](VFN.md)
for the parameter, routing and telemetry contract and verification commands.

## SD card and recording

The current Revo Mini definition supports an SPI SD card. Native binary logs
record sensor data, routed commands, physical PWM, navigation state/reason bits, roll
stabilization and storage/input status. `LOG_DISARMED=1` starts continuous
recording; zero stops and closes the file in the background. `VLOG_HZ` controls
Vektor sample rate independently of telemetry. MAVFTP, native `LOG_*` transfer,
optional remote block logging, storage information/interval requests and SD
format commands use standard ArduPilot MAVLink services. See
[Storage.md](Storage.md) for wiring, parameters, record groups, protocols and tests.

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

A route is inactive if either value is `0`. Routes to an unselected main
propulsion component are not executed; routes from it have invalid sources.
Dormant routes retain their destination claims and participate in graph
validation. There is one common RT table, not separate saved route profiles.

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
141/142  AZI1 azimuth/motor output (direct PWM only in AZ2)
151/152  AZI2 azimuth/motor output (direct PWM only in AZ2)
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
141/142  AZI1 X/Y input
151/152  AZI2 X/Y input
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
Logging.cpp                native binary records and bounded logger maintenance
Storage.cpp/.h             background SD status and recording-stop file closure
VSP1.cpp/.h                VSP1 algorithm and telemetry values
VSP2.cpp/.h                VSP2 algorithm and telemetry values
Telemetry.cpp/.h           shared rate limit and MAVLink publisher
VTelemetryCore.cpp/.h       protocol-independent VTEL mapping and codecs
VTelemetryParameters.*      VTEL_SLT1..VTEL_SLT65 AP_Param group
VTelemetry.cpp/.h           i-BUS/HoTT UART workers and source snapshots
VTEL.md                     telemetry topology, slot/module catalog and defaults
Config.h                   small application constants
wscript                    ArduPilot build target
```

## Build integration

### CAN and routed PWM

CAN-capable boards include the standard `AP_CANManager` and DroneCAN drivers.
Boards without CAN omit these libraries, the manager object, the output bridge,
and the `CAN_*` parameters at build time. Existing GPS and compass frontends
discover DroneCAN devices through their normal ArduPilot backends. Other sensor
families disabled by Vektor2 remain disabled.

Routing still targets PWM channels. After each routing cycle, the final
`PWMn_MIN/MAX/INV`-adjusted values are also supplied to the standard DroneCAN
servo backend. Channel 1 maps to actuator ID 1, and so on, up to the board's
`NUM_SERVO_CHANNELS` limit. Local PWM output continues normally; no additional
route or `SERVOx_FUNCTION` configuration is needed.

Example for a DroneCAN PWM actuator on CAN port 1, output channel 1:

1. Set `CAN_P1_DRIVER=1`, `CAN_P1_BITRATE` to the bus bitrate, and
   `CAN_D1_PROTOCOL=1` (DroneCAN), then reboot.
2. Set `CAN_D1_UC_SRV_BM=1` to publish channel 1 (bit 0; add bits for other
   channels), and `CAN_D1_UC_SRV_RT` to the desired update rate.
3. Enable bit 4 (`SendServoAsPWM`, value 16) in `CAN_D1_UC_OPTION`, preserving
   other option bits, for exact microsecond commands. Without this option the
   standard backend sends normalized commands instead. The receiving device
   must support DroneCAN PWM actuator commands.
4. Configure the usual `RTn_SRC` / `RTn_DST` PWM route. Verify the actuator ID
   and compare received CAN commands with the local PWM pulse width at low,
   middle, and high input, including output scaling and inversion.

Unrouted, disabled, or invalid outputs are supplied as zero to the backend,
which removes them from its servo active mask. This stops commands; it does
not send a physical disable command. Configure and verify the actuator's
command timeout. Test route removal and RC loss with
`DisableRoutedOutputs`; `HoldLast` intentionally continues the last command.

This bridge uses DroneCAN servo/PWM commands. DroneCAN raw ESC commands retain
the library's soft-arming gate; Vektor2 has no arming state machine and does not
bypass it. PiccoloCAN and KDECAN function-based motor outputs are not enabled.
Standard MAVLink CAN forwarding and SLCAN diagnostics remain available where
the board enables them.

For a SITL build, run `python3 Tools/Vektor2/tests/test_mavlink_can.py` to
compare local PWM telemetry with actual DroneCAN PWM commands over the standard
SITL multicast transport. Run this test without other CAN SITL instances on
the shared multicast bus. Physical bus timing and actuator timeout behavior
still need a hardware bench check.

### Building

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

## Shared thrust mapper

See [VspMapper.md](VspMapper.md) for the shared CW/CCW vector-to-rods API,
installation conventions and `VSPn_TDIR` / `VSPn_TMAG` named-float estimates.
`VSP_CONF=1` means AFT; `VSP_CONF=2` means VWT (Voith Water Tractor).

## Manual vector control

`VSP1_VMODE=1` and `VSP2_VMODE=1` independently enable body-frame manual thrust
vector inputs through the shared mapper. Both default to legacy mode (0). See
[VspVectorControl.md](VspVectorControl.md) for input semantics, neutral transitions,
controller ownership, slew parameters and state telemetry.

## VSP authority telemetry

`VSP1_AUTH` / `VSP2_AUTH` report the current rod command sources as an integer
bitmask in NAMED_VALUE_FLOAT. See [VspAuthority.md](VspAuthority.md) for manual,
heading, VRS and neutral-guard flags and their combinations.

## Coordinated VSP navigation (opt-in)

See [VSP_ALLOCATOR.md](VSP_ALLOCATOR.md) for AFT/VWT Heading Hold, Full DP,
geometry, readiness, status bits, output rollback and commissioning.
With `VNAV_VSP_EN=0`, manual control and tunnel Heading Hold remain available.
In VWT layout, only the stern tunnel can participate in tunnel Heading Hold.
