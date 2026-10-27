# SD card, binary logging and MAVLink file services

Vektor2 uses the existing AP_Filesystem/FATFS SD driver, AP_Logger binary
recording, and ArduPilot's MAVFTP and log-transfer protocols. There is no new
serial protocol or private MAVLink message.

## Board and filesystem

The **current `revo-mini` definition in this checkout** already enables FATFS
and an SPI SD card on **SPI3: PC10 SCK, PC11 MISO, PC12 MOSI, PA13 CS**. This
is the customized BMI088 board definition used by Vektor2. The separate
`revo-mini-sd` target has different sensor/wiring definitions; it is not a
drop-in firmware substitute. No board pin assignments are changed here.

SD support depends on building for hardware with a writable filesystem. A
missing/unmounted card does not prevent RC/PWM, estimation or stabilization
from running. Card mount/capacity checks and recording-stop buffer drain,
sync and closure run on the HAL IO thread, once per second. Normal log writes use AP_Logger's
bounded RAM buffer and its dedicated IO thread. Card availability/capacity is
copied from a cached snapshot when sending status; no capacity scan runs in
the control loop. Mount retries allow a later inserted/recovered card to be
discovered; the native file logger retries failed opens after its backoff.

Card contents are retained at boot. No automatic format is performed. The
native logger may remove **old log files** to maintain `LOG_FILE_MB_FREE` or
the configured log-file limit; unrelated files are outside that cleanup.

On the Revo Mini, binary files are under **`/APM/LOGS/NNNNNNNN.BIN`**. SITL
uses **`logs/NNNNNNNN.BIN`** inside its working directory and emulates storage
capacity with that host filesystem. `LASTLOG.TXT` is maintained by AP_Logger.

## Recording controls

Vektor2 has no arming state. It keeps the native logger disarmed, enabling
continuous recording through `LOG_DISARMED=1`. Downloads/erase therefore
remain available without introducing vehicle modes or fake arming.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `LOG_BACKEND_TYPE` | 1 | Native bitmap: 1 file/SD, 2 remote MAVLink, 3 both; changes require reboot |
| `LOG_DISARMED` | 1 | 1 records continuously; 0 stops recording and closes the current file in the background |
| `LOG_BITMASK` | 63 | Vektor record groups, listed below |
| `VLOG_HZ` | 10 | Vektor sample rate, 0..50 Hz, independent of telemetry rates |
| `LOG_FILE_BUFSIZE` | 8 | File logger RAM buffer in KiB; native allocation may reduce it if memory is limited |
| `LOG_FILE_MB_FREE` | 16 | Native minimum free-space target, maintained by deleting oldest logs |
| `LOG_MAX_FILES` | 500 | Native log count/number rotation limit |

`VLOG_HZ=0` stops Vektor samples, while native metadata/events may still be
recorded. `LOG_BITMASK` selects Vektor groups; use `LOG_DISARMED=0` to stop
the recording session. After setting it to zero, the IO thread drains pending
records, syncs and closes the file. This normally starts within one second;
completion depends on card latency. Returning to 1 starts a new log.
Saved parameter values override these compiled defaults.

Other standard logger parameters remain available. `LOG_DISARMED=2` follows
native USB-connection gating; mode 3 discards never-armed logs on a later boot,
so it is unsuitable for retaining Vektor sessions because Vektor never arms.
No capability is implied by the unrelated reserved `VNAV_LOG_*` parameters.

Each log contains native self-describing FMT/FMTU records, parameter snapshots
and firmware/system metadata, followed by selected Vektor samples:

| Bit value | Records | Contents |
| --- | --- | --- |
| 1 | `VATT`, `VIMU` | Roll/pitch/yaw in degrees, attitude validity, acceleration in m/s², body gyro in rad/s, INS health |
| 2 | `VGPS`, `VPOS` | Receiver fix/satellites/position/speed/course/HDOP; fused position and NED velocity with validity flags |
| 4 | `VVSP`, `VRC`, `VPWM` | Both VSP raw X/Y/RPM inputs and final component outputs; RC pulse widths/validity; enabled physical PWM outputs |
| 8 | `VHDG`, `VHST` | Heading hold enable/state/inhibition, heading/target/error/rate, PID request/integral, travel scale, idle wait and both applied Y additions; slow status snapshot with readable reason |
| 16 | `VROL`, `VRSD` | Enabled roll controller state/inhibition/roll/rate/demand and each drive's correction/headroom/duty/limiting/burst count |
| 32 | `VSTA` | Storage state, control-input validity, total/free bytes and dropped logger blocks |

Raw/input/output pulse widths and heading additions use microseconds.
VLOG_HZ sample groups share a monotonic `TimeUS` timestamp. `VHST` follows the
separate 1/2 Hz heading status clock and records the reason even when measurement
telemetry is disabled. HDG targets
and errors are NaN before HOLD; unavailable fused position/velocity is marked
invalid with NaN floating fields. Native library events and named telemetry
can also appear at their own rates. `LOG_REPLAY` remains a native advanced
option; these Vektor records alone do not claim complete EKF replay support.

A small, low-priority scheduler task gives the native logger's metadata writer
a real time budget after control/PWM and MAVLink. Vektor's sensor/control loop
still runs directly from the IMU, without an AP_Vehicle scheduler framework.

## MAVLink transports

| Interface | Supported behavior |
| --- | --- |
| `AUTOPILOT_VERSION.capabilities` | Native FTP capability advertisement, subject to the existing board FTP-disable option |
| `FILE_TRANSFER_PROTOCOL` / MAVFTP | Directory listing/create/remove; upload/download, burst reads, file rename/remove/truncate and CRC32; native session/retry/error handling |
| `LOG_REQUEST_LIST` → `LOG_ENTRY` | List binary recording sessions |
| `LOG_REQUEST_DATA` → `LOG_DATA` | Download binary logs by ID and offset |
| `LOG_REQUEST_END` | End a log transfer; safe with no logger backend configured |
| `LOG_ERASE` | Delete native log recordings, retaining unrelated card files |
| `REMOTE_LOG_BLOCK_STATUS` / `REMOTE_LOG_DATA_BLOCK` | Native acknowledged live binary logging when the MAVLink backend is selected |
| `STORAGE_INFORMATION` | Storage ID/count/status, total/used/available capacity in MiB, medium type and name |
| `STATUSTEXT` | Card ready/unavailable transitions, format progress/results and native logger errors |

MAVFTP runs its file operations in the native FTP worker. The native `LOG_*`
download path retains ArduPilot's existing behavior: a download pauses log
recording, and `LOG_REQUEST_END` releases the read session. File-backend
downloads are available only when a file backend is selected. Remote block
logging is a separate live stream; `MAV_CMD_LOGGING_START/STOP` and the common
`LOGGING_DATA` protocol are not aliases for it.

Set `LOG_BACKEND_TYPE=3` and reboot for both SD and remote block logging; use
the normal ArduPilot remote-log client to start/ACK/stop its blocks. Backend
selection is independent of telemetry sample rates.

### Storage information requests

Use `COMMAND_LONG` or `COMMAND_INT`:

```text
MAV_CMD_REQUEST_MESSAGE:
  param1 = 261  (STORAGE_INFORMATION)
  param2 = 0    (all storage) or 1 (this SD/filesystem)
```

The deprecated `MAV_CMD_REQUEST_STORAGE_INFORMATION` also works with storage
ID 0/1 in parameter 1 and request flag 1 in parameter 2. Other storage IDs are
unsupported. The single supported medium is ID 1; missing/unmounted media
report EMPTY, usable media READY, and builds without writable storage report
NOT_SUPPORTED with count zero. While formatting, capacity is unavailable and
status is UNFORMATTED. Read/write speed fields remain zero; no benchmark is run.

Status is sent on availability transitions and on request. Optional per-link
periodic reporting uses `MAV_CMD_SET_MESSAGE_INTERVAL`, message ID 261, interval
in microseconds. Positive intervals are bounded to 100 ms..60 seconds. Zero
restores the default (on-change/on-request); -1 disables the periodic stream.
`MAV_CMD_GET_MESSAGE_INTERVAL` returns the effective interval, or -1 when no
periodic stream is selected. `MAV_CMD_REQUEST_MESSAGE` for MESSAGE_INTERVAL
(244), with parameter 2 = 261, is equivalent.

### Formatting

Formatting is available on the native FATFS SD backend. It erases the medium
and runs asynchronously, using the native command progress/acknowledgement:

1. Set `LOG_DISARMED=0`, finish log downloads with `LOG_REQUEST_END`, and allow
   the background buffer drain/sync/close to complete.
2. Send `MAV_CMD_STORAGE_FORMAT` with **parameter 1 = 1** (storage ID),
   **parameter 2 = 1** (explicit format request).
3. Receive `COMMAND_ACK` IN_PROGRESS, followed by the native final ACCEPTED or
   FAILED acknowledgement and updated `STORAGE_INFORMATION`.
4. Restore `LOG_DISARMED=1` when recording should resume.

An active recording, pending background close, active log download or ongoing
format returns TEMPORARILY_REJECTED. Unsupported IDs/format flags do not start
an operation. A formatting allocation failure now reaches FAILED rather than
leaving the native progress task stuck. The POSIX SITL backend has no format
operation and returns FAILED without erasing its working/host filesystem.

## Verification

```sh
./waf configure --board sitl
./waf --targets bin/Vektor2
python3 Tools/Vektor2/tests/test_mavlink_storage.py
python3 Tools/Vektor2/tests/test_mavlink_vhdg.py
python3 Tools/Vektor2/tests/test_mavlink_vrs.py
./waf configure --board revo-mini
./waf --targets bin/Vektor2
```

The storage integration test uses temporary parameter/card directories and
localhost ports 6012/6013. It checks capacity and interval commands, MAVFTP file
operations/CRC, complete byte-exact native log downloads, live remote logging,
binary log decoding and rate changes, final buffered data on clean stop,
recording restart, record/format guards, log erase, and a
second boot with no logger backend. Run the simulator tests sequentially;
their additional simulated serial ports can overlap. Physical SD removal,
formatting and electrical/card behavior require a board test.
