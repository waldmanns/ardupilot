#!/usr/bin/env python3
"""VRS parameter, EKF, routing and telemetry integration test in isolated SITL.

Requires pymavlink and build/sitl/bin/Vektor2. PWM9..14 avoid the simulated
rover's propulsion channels; a roll trim requests deterministic corrections
without pretending the rover simulator models a VSP-equipped hull.
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
    with tempfile.TemporaryDirectory(prefix='vektor2-vrs-sitl-') as directory:
        defaults = Path(directory) / 'defaults.parm'
        defaults.write_text('SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\n'
                            'RC_OPTIONS 1\nRC_OVERRIDE_TIME -1\nGPS1_TYPE 0\n'
                            'VSP_TEL_HZ 10\n')
        with open(Path(directory) / 'sitl.log', 'w+') as log:
            process = subprocess.Popen(
                [str(binary), '--model', 'rover', '--speedup', '1',
                 '--instance', '24', '--defaults', str(defaults),
                 '--serial0', 'tcp:5992', '--serial1', 'tcp:5993'],
                cwd=directory, stdout=log, stderr=subprocess.STDOUT)
            link = None
            try:
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    try:
                        link = mavutil.mavlink_connection('tcp:127.0.0.1:5993', source_system=255)
                        break
                    except OSError:
                        if process.poll() is not None:
                            raise RuntimeError('SITL exited during startup')
                        time.sleep(0.1)
                assert link is not None, 'SITL connection timeout'
                assert link.wait_heartbeat(timeout=30), 'No heartbeat'
                system, component = link.target_system, link.target_component

                def wait_for(kind, predicate, timeout=8):
                    deadline = time.monotonic() + timeout
                    last = None
                    while time.monotonic() < deadline:
                        if process.poll() is not None:
                            raise RuntimeError(f'SITL exited with status {process.returncode}')
                        msg = link.recv_match(type=kind, blocking=True, timeout=0.1)
                        if msg is not None:
                            last = msg
                            if predicate(msg):
                                return msg
                    raise AssertionError(f'Timed out waiting for {kind}; last={last}')

                def parameter(name, value=None):
                    if value is None:
                        link.mav.param_request_read_send(system, component, name.encode(), -1)
                    else:
                        link.mav.param_set_send(system, component, name.encode(), value,
                                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    msg = wait_for('PARAM_VALUE', lambda m: m.param_id == name)
                    if value is not None:
                        assert abs(msg.param_value-value) < 0.0001, (name, msg.param_value)
                    return msg.param_value

                def override(channels):
                    link.mav.rc_channels_override_send(system, component, *channels, *([65535]*12))

                def drain():
                    while link.recv_match(blocking=False) is not None:
                        pass

                def outputs(msg):
                    return tuple(getattr(msg, f'servo{i}_raw') for i in range(9, 15))

                def vrs_name(name):
                    return name.startswith(('RS_', 'V1_RS_', 'V2_RS_'))

                # All sixteen parameters must be accessible under their intended
                # public names, with the feature and both gains off by default.
                expected = {'VRS_ENABLE': 0, 'VRS_P': 0, 'VRS_D': 0,
                            'VRS_TRIM': 0, 'VRS_FILT_HZ': 2, 'VRS_DB': 0.02,
                            'VRS_ON_MS': 250, 'VRS_OFF_MS': 250,
                            'VRS_SLEW': 1, 'VRS_MAN_LIM': 0.8,
                            'VSP1_RS_ANG': 0, 'VSP1_RS_SIGN': 0, 'VSP1_RS_MAX': 0.2,
                            'VSP2_RS_ANG': 0, 'VSP2_RS_SIGN': 0, 'VSP2_RS_MAX': 0.2}
                for name, value in expected.items():
                    assert abs(parameter(name)-value) < 0.0001, name
                print('PASS: all 16 VRS parameters/defaults through MAVLink', flush=True)

                routes = [(1, 101), (2, 102), (3, 103), (4, 111), (5, 112), (6, 113),
                          (101, 9), (102, 10), (103, 11), (111, 12), (112, 13), (113, 14)]
                for i, (source, destination) in enumerate(routes, 1):
                    parameter(f'RT{i}_SRC', source)
                    parameter(f'RT{i}_DST', destination)
                for i in (1, 2):
                    parameter(f'VSP{i}_LIM', 100)
                link.mav.command_long_send(system, component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    36, 50000, 0, 0, 0, 0, 0)
                ack = wait_for('COMMAND_ACK', lambda m:
                               m.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)
                assert ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED, ack
                # The unchanged legacy limiter divides by X offset. Keep X nonzero
                # here: SITL traps floating divide-by-zero unlike the MCU. The
                # host geometry sweep covers every VRS angle independently.
                override([1550, 1500, 1700, 1550, 1450, 1800])
                baseline = (1510, 1500, 1700, 1510, 1490, 1800)
                wait_for('SERVO_OUTPUT_RAW', lambda m: outputs(m) == baseline)
                drain()
                deadline = time.monotonic() + 0.6
                while time.monotonic() < deadline:
                    msg = link.recv_match(blocking=True, timeout=0.1)
                    if msg is not None and msg.get_type() == 'NAMED_VALUE_FLOAT':
                        assert not vrs_name(msg.name), msg
                print('PASS: disabled baseline PWM and no added VRS telemetry', flush=True)

                for name, value in {'VSP1_RS_ANG': 90, 'VSP1_RS_SIGN': 1,
                                    'VSP2_RS_ANG': 270, 'VSP2_RS_SIGN': -1,
                                    'VRS_P': 0.1, 'VRS_TRIM': 5, 'VRS_SLEW': 10}.items():
                    parameter(name, value)
                parameter('VRS_ENABLE', 1)
                wait_for('NAMED_VALUE_FLOAT', lambda m: m.name == 'RS_ROLL')
                # EKF tilt alignment may still be in progress on a fresh startup.
                wait_for('SERVO_OUTPUT_RAW', lambda m:
                         m.servo9_raw == 1510 and m.servo10_raw > 1500 and
                         m.servo11_raw == 1700 and m.servo12_raw == 1510 and
                         m.servo13_raw == 1490 and m.servo14_raw == 1800, timeout=45)
                wait_for('NAMED_VALUE_FLOAT', lambda m: m.name == 'V1_RS_DUT' and m.value > 0)

                # The main command can move while bursts continue; +Y correction
                # must preserve the newly requested X coordinate and RPM command.
                override([1600, 1500, 1750, 1550, 1450, 1800])
                wait_for('SERVO_OUTPUT_RAW', lambda m:
                         m.servo9_raw == 1520 and m.servo10_raw > 1500 and m.servo11_raw == 1750)
                parameter('VRS_TRIM', -5)
                wait_for('SERVO_OUTPUT_RAW', lambda m:
                         m.servo9_raw == 1520 and m.servo10_raw == 1500 and
                         m.servo12_raw == 1510 and m.servo13_raw < 1490 and m.servo14_raw == 1800)
                print('PASS: EKF-gated bursts, both effect signs, simultaneous maneuvering and RPM', flush=True)

                parameter('VRS_ENABLE', 0)
                baseline = (1520, 1500, 1750, 1510, 1490, 1800)
                wait_for('SERVO_OUTPUT_RAW', lambda m: outputs(m) == baseline)
                drain()
                deadline = time.monotonic() + 0.8
                while time.monotonic() < deadline:
                    msg = link.recv_match(blocking=True, timeout=0.1)
                    if msg is None:
                        continue
                    if msg.get_type() == 'NAMED_VALUE_FLOAT':
                        assert not vrs_name(msg.name), msg
                    if msg.get_type() == 'SERVO_OUTPUT_RAW':
                        assert outputs(msg) == baseline, msg
                parameter('VRS_ENABLE', 1)
                override([0]*6)  # release every source; no receiver is enabled
                wait_for('NAMED_VALUE_FLOAT', lambda m:
                         m.name == 'RS_INHIB' and int(m.value) & 4)
                wait_for('NAMED_VALUE_FLOAT', lambda m: m.name == 'V2_RS_CMD' and m.value == 0)
                print('PASS: immediate live disable, silent VRS telemetry, RC-loss inhibition', flush=True)
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
