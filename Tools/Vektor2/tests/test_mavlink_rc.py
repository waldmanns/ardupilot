#!/usr/bin/env python3
"""Exercise Vektor2's standard MAVLink RC path in SITL (requires pymavlink).

Run after a SITL build: python3 Tools/Vektor2/tests/test_mavlink_rc.py
The simulator and its parameter storage are isolated in a temporary directory.
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil


def run(binary):
    with tempfile.TemporaryDirectory(prefix='vektor2-mavlink-rc-') as directory:
        defaults = Path(directory) / 'defaults.parm'
        defaults.write_text('SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\n'
                            'RC_OPTIONS 1\nGPS1_TYPE 0\nVSP_TEL_HZ 0\n')
        with open(Path(directory) / 'sitl.log', 'w+') as log:
            process = subprocess.Popen(
                [str(binary), '--model', 'rover', '--speedup', '1',
                 '--instance', '23', '--defaults', str(defaults),
                 '--serial0', 'tcp:5990', '--serial1', 'tcp:5991'],
                cwd=directory, stdout=log, stderr=subprocess.STDOUT)
            link = None
            try:
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    try:
                        link = mavutil.mavlink_connection('tcp:127.0.0.1:5991',
                                                         source_system=255)
                        break
                    except OSError:
                        if process.poll() is not None:
                            raise RuntimeError('SITL exited during startup')
                        time.sleep(0.1)
                assert link is not None, 'SITL connection timeout'
                assert link.wait_heartbeat(timeout=30), 'No heartbeat'
                system, component = link.target_system, link.target_component

                def wait_for(kind, predicate, timeout=5):
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        msg = link.recv_match(type=kind, blocking=True, timeout=0.1)
                        if msg is not None and predicate(msg):
                            return msg
                    raise AssertionError(f'Timed out waiting for {kind}')

                def parameter(name, value=None):
                    if value is None:
                        link.mav.param_request_read_send(system, component, name.encode(), -1)
                    else:
                        link.mav.param_set_send(system, component, name.encode(), value,
                                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    msg = wait_for('PARAM_VALUE', lambda m: m.param_id == name)
                    if value is not None:
                        assert abs(msg.param_value - value) < 0.001, (name, msg.param_value)
                    return msg.param_value

                def override(ch1=65535, ch9=65535, ch16=65535, target=None):
                    channels = [65535] * 18
                    channels[0], channels[8], channels[15] = ch1, ch9, ch16
                    link.mav.rc_channels_override_send(
                        system if target is None else target, component, *channels)

                def drain():
                    while link.recv_match(blocking=False) is not None:
                        pass

                def assert_absent_value(value, duration=0.35):
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        msg = link.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.1)
                        assert msg is None or msg.chan1_raw != value, msg

                assert parameter('RC_OVERRIDE_TIME') == 3.0
                assert parameter('MAV_GCS_SYSID') == 255
                parameter('RC_OVERRIDE_TIME', 0.7)
                parameter('RT1_SRC', 1)
                parameter('RT1_DST', 1)
                for msgid in (65, 36):  # RC_CHANNELS, SERVO_OUTPUT_RAW
                    link.mav.command_long_send(system, component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        msgid, 50000, 0, 0, 0, 0, 0)
                wait_for('RC_CHANNELS', lambda m: m.chancount == 0)

                override(ch1=1630, ch9=1740, ch16=1850)
                wait_for('RC_CHANNELS', lambda m: m.chan1_raw == 1630 and
                         m.chan9_raw == 1740 and m.chan16_raw == 1850)
                wait_for('SERVO_OUTPUT_RAW', lambda m: m.servo1_raw == 1630)
                override(ch9=0)  # 0 ignores extension channels; UINT16_MAX ignores others
                wait_for('RC_CHANNELS', lambda m: m.chan1_raw == 1630 and m.chan9_raw == 1740)
                override(ch1=0, ch9=65534, ch16=65534)
                wait_for('RC_CHANNELS', lambda m: m.chancount == 0)
                print('PASS: parameter round trip, RC channels, PWM routing, ignore/release')

                drain()
                link.mav.srcSystem = 254
                override(ch1=1800)
                assert_absent_value(1800)
                link.mav.srcSystem = 255
                override(ch1=1800, target=system + 1)
                assert_absent_value(1800)
                print('PASS: source and target system filtering')

                override(ch1=1690)
                wait_for('RC_CHANNELS', lambda m: m.chan1_raw == 1690)
                deadline = time.monotonic() + 1.3
                expired = False
                while time.monotonic() < deadline:
                    link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    msg = link.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.1)
                    expired |= msg is not None and msg.chancount == 0
                assert expired, 'Heartbeat incorrectly kept RC input alive'
                wait_for('SERVO_OUTPUT_RAW', lambda m: m.servo1_raw == 0)
                print('PASS: override timeout and output disable while GCS remains connected')

                parameter('RC_OPTIONS', 3)  # Ignore receiver and MAVLink overrides
                drain()
                override(ch1=1800)
                assert_absent_value(1800)
                parameter('RC_OVERRIDE_TIME', 0)
                parameter('RC_OPTIONS', 1)
                override(ch1=1800)
                assert_absent_value(1800)
                print('PASS: RC_OPTIONS and disabled overrides')
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read()[-6000:])
                raise
            finally:
                if link is not None:
                    link.close()
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path,
                        default=Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2')
    run(parser.parse_args().binary.resolve())
