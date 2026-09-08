#!/usr/bin/env python3
"""MiniDP MAVLink application regressions; requires built SITL and pymavlink.

Uses an isolated parameter store and loopback TCP. The generic rover model is
only a sensor/transport fixture; test_dynamics.cpp checks MiniDP hull forces.
"""
import argparse
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil


class Probe:
    def __init__(self, binary, directory, port):
        self.directory = Path(directory)
        defaults = self.directory / 'defaults.parm'
        defaults.write_text('''FRAME_TYPE 902
AUTH_MAV_TGT 1
AUTH_TEST 1
MAN_ENABLE 0
AUTH_TGT_LOSS 1
BRD_SAFETY_DEFLT 0
EK3_ENABLE 1
AHRS_EKF_TYPE 3
LOG_DISARMED 0
''')
        self.log = (self.directory / 'console.log').open('w')
        self.process = subprocess.Popen([
            str(binary), '--model', 'rover', '--speedup', '3', '--instance', '41',
            '--serial0', f'tcp:{port}', '--defaults', str(defaults)],
            cwd=directory, stdout=self.log, stderr=subprocess.STDOUT)
        self.endpoint = f'tcp:127.0.0.1:{port}'
        self.connection = None
        self.last_hb = 0
        self.latest = {}
        self.named = {}
        self.overrides = None
        for _ in range(100):
            self.check_alive()
            try:
                self.connection = mavutil.mavlink_connection(
                    f'tcp:127.0.0.1:{port}', source_system=255)
                break
            except (ConnectionRefusedError, OSError):
                time.sleep(0.1)
        assert self.connection is not None, 'SITL connection timeout'

    def check_alive(self):
        assert self.process.poll() is None, f'SITL exited: {self.process.returncode}; see {self.directory}'

    def pump(self, duration):
        messages = []
        until = time.monotonic() + duration
        while time.monotonic() < until:
            self.check_alive()
            now = time.monotonic()
            if now - self.last_hb > 0.15:
                self.connection.mav.heartbeat_send(6, 8, 0, 0, 0)
                if self.overrides is not None:
                    self.connection.mav.rc_channels_override_send(1, 1, *self.overrides)
                self.last_hb = now
            m = self.connection.recv_match(blocking=False)
            if m is None:
                time.sleep(0.005)
                continue
            messages.append(m)
            self.latest[m.get_type()] = m
            if m.get_type() == 'NAMED_VALUE_FLOAT':
                self.named[m.name] = m.value
        return messages

    def command(self, command, *values):
        self.pump(0.05)
        self.connection.mav.command_long_send(1, 1, command, 0,
            *(list(values) + [0] * (7 - len(values))))
        for _ in range(20):
            for m in self.pump(0.05):
                if m.get_type() == 'COMMAND_ACK' and m.command == command:
                    return m.result
        raise AssertionError(f'No ACK for {command}')

    def param(self, name, value=None):
        if value is None:
            self.connection.mav.param_request_read_send(1, 1, name.encode(), -1)
        else:
            self.connection.mav.param_set_send(1, 1, name.encode(), value, 9)
        for _ in range(20):
            for m in self.pump(0.05):
                if m.get_type() == 'PARAM_VALUE' and m.param_id == name:
                    if value is not None:
                        assert abs(m.param_value - value) < 0.01, (name, m.param_value, value)
                    return m.param_value
        raise AssertionError(f'No parameter {name}')

    def target(self, frame, x, y, yaw=0, mask=4 | 8 | 16 | 32 | 64 | 128 | 256 | 2048):
        self.connection.mav.set_position_target_local_ned_send(
            0, 1, 1, frame, mask, x, y, 0, 0, 0, 0, 0, 0, 0, yaw, 0)
        self.pump(0.4)

    def assert_named(self, name, value):
        self.pump(0.5)
        assert self.named.get(name) == value, (name, self.named.get(name), value)

    def servo(self, channel):
        self.pump(0.2)
        return getattr(self.latest['SERVO_OUTPUT_RAW'], f'servo{channel}_raw')

    def close(self):
        if self.connection:
            self.connection.close()
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.log.close()


def run(p):
    for _ in range(100):
        p.pump(0.4)
        if p.named.get('DP_READY') == 1:
            break
    assert p.named.get('DP_READY') == 1, ('no-baro EKF not ready', p.named)
    assert p.param('EK3_SRC1_POSZ') == 3
    assert p.param('EK3_SRC2_POSZ') == 3
    assert p.param('EK3_SRC3_POSZ') == 3
    assert p.param('EK2_ALT_SOURCE') == 2
    print('PASS: no-baro defaults produce DP-ready EKF', flush=True)
    assert p.command(401) == 0  # MiniDP pre-arm policy, no barometer/mission requirement
    assert p.command(179, 1) != 0  # unsupported home setter must not dereference AP_Vehicle
    assert p.command(511, 36, 50000) == 0

    p.param('SERVO8_FUNCTION', 51)
    p.overrides = [1700, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
    p.pump(0.5)
    assert p.servo(8) == 1700, p.servo(8)
    p.overrides[0] = 1300
    p.pump(0.5)
    assert p.servo(8) == 1300, p.servo(8)
    p.param('SERVO8_FUNCTION', 140)
    p.overrides[0] = 2000
    p.pump(0.5)
    assert p.servo(8) == p.param('SERVO8_MAX'), p.servo(8)
    p.param('SERVO2_FUNCTION', 51)
    p.overrides[0] = 1700
    p.pump(0.5)
    assert p.servo(2) == 1700, p.servo(2)
    assert p.command(176, 1, 2) != 0  # incomplete frame cannot enter DP
    p.assert_named('DP_MODE', 0)
    p.param('SERVO2_FUNCTION', 34)
    p.overrides[0] = 1500
    print('PASS: raw/scaled RCIN on arbitrary output and former motor output', flush=True)

    p.connection.mav.set_mode_send(1, 1, 2)
    p.assert_named('DP_MODE', 2)
    p.connection.mav.set_mode_send(1, 1, 0)
    p.assert_named('DP_MODE', 0)
    print('PASS: SET_MODE message path', flush=True)

    p.target(1, 12, -4)
    p.assert_named('DP_TGT_N', 12)
    target_id = p.named['DP_TGT_ID']
    p.target(8, 12, -4)  # BODY_NED positions are absolute local NED.
    p.assert_named('DP_TGT_N', 12)
    p.assert_named('DP_TGT_E', -4)
    p.assert_named('DP_TGT_ID', target_id)
    p.target(1, 999, 999, mask=4 | 16 | 32 | 64 | 128 | 256 | 2048)
    p.assert_named('DP_TGT_ID', target_id)  # velocity requested: reject atomically
    p.target(1, 12, -4, yaw=1e20)
    p.assert_named('DP_MODE', 2)
    assert p.command(176, 1, 0) == 0
    p.target(1, math.nan, 0)
    p.assert_named('DP_OWN', 0)
    p.param('AUTH_MAV_TGT', 0)
    assert p.command(176, 1, 2) != 0
    p.assert_named('DP_MODE', 0)
    p.assert_named('DP_OWN', 0)
    p.param('AUTH_MAV_TGT', 1)
    print('PASS: target frames, stable identity, huge yaw, atomic rejection', flush=True)

    p.param('MAN_ENABLE', 1)
    p.param('AUTH_RC_TAKE', 1)
    assert p.command(176, 1, 2) == 0
    p.assert_named('DP_MODE', 2)
    p.overrides[1] = 1700
    p.assert_named('DP_MODE', 0)
    p.overrides[1] = 1500
    p.param('MAN_ENABLE', 0)
    print('PASS: centered RC preserves DP; deflection takes over', flush=True)

    p.param('SIM_RC_FAIL', 1)
    p.param('MAN_ENABLE', 1)
    p.overrides[1] = 1700
    p.pump(1)
    p.assert_named('RC_OK', 1)
    p.assert_named('DP_OWN', 1)
    # Release yaw override while receiver is absent. Continued overrides on
    # other channels must not make the stale physical yaw input usable.
    p.overrides[3] = 0
    p.pump(1)
    p.assert_named('DP_OWN', 0)
    p.overrides[3] = 1500
    p.assert_named('DP_OWN', 1)
    p.overrides[1] = 1500
    p.param('MAN_ENABLE', 0)
    p.param('SIM_RC_FAIL', 0)
    print('PASS: receiver-free overrides work; missing channels reject stale data', flush=True)

    assert p.command(400, 1, 21196) == 0
    assert p.command(209, 1, 0, 10, 5, 1, 0) == 0
    assert p.command(209, 3, 0, 20, 5, 1, 0) == 0
    assert p.command(209, 1, 0, math.nan, 5, 1, 0) != 0
    assert p.command(209, 1, 0, 10, math.nan, 1, 0) != 0
    assert p.command(209, 1, 1, math.nan, 5, 1, 0) != 0
    assert p.command(209, 1, 0, 10, 5, -1, 0) != 0
    assert p.command(209, 6, 0, 10, 5, 1, 0) != 0
    p.assert_named('DP_MODE', 3)
    p.param('FRAME_TYPE', 901)
    assert p.command(209, 5, 0, 10, 5, 1, 0) == 0  # Motor5 remains frame902.
    assert p.command(176, 1, 0) == 0
    p.param('FRAME_TYPE', 902)
    print('PASS: motor-test replacement, NaN rejection, frame held until reboot', flush=True)

    p.param('OUT_DARM_ACT', 1)
    assert p.command(400, 0) == 0
    p.param('SERVO1_TRIM', 1400)
    p.param('SERVO2_TRIM', 1600)
    p.param('SERVO1_FUNCTION', 34)
    p.param('SERVO2_FUNCTION', 33)
    p.pump(0.5)
    assert p.servo(1) == 1400, p.servo(1)
    assert p.servo(2) == 1600, p.servo(2)
    p.param('OUT_DARM_ACT', 0)
    print('PASS: swapped physical motor outputs retain their own trims', flush=True)

    p.param('IN_RC_KILL', 5)
    assert p.command(400, 1, 21196) == 0
    p.overrides[4] = 1900
    p.assert_named('DP_MODE', 4)
    p.overrides[4] = 1500
    p.pump(0.5)
    assert p.command(176, 1, 0) != 0  # recovery must first disarm
    assert p.command(400, 0) == 0
    assert p.command(176, 1, 0) == 0
    p.assert_named('DP_MODE', 0)
    p.assert_named('DP_OUT', 0)
    print('PASS: failsafe disarm/MANUAL recovery stays disarmed', flush=True)

    # Reboot used to dereference the absent AP_Vehicle singleton. Confirm
    # the new image loads saved parameters and applies a pending frame change.
    p.param('IN_RC_KILL', 0)
    p.param('FRAME_TYPE', 901)
    assert p.command(246, 1) == 0
    p.connection.close()
    time.sleep(0.5)
    p.check_alive()
    p.connection = mavutil.mavlink_connection(
        p.endpoint, source_system=255)
    p.pump(1)
    assert p.param('FRAME_TYPE') == 901
    assert p.param('SERVO1_FUNCTION') == 34
    assert p.param('SERVO2_TRIM') == 1600
    assert p.command(400, 1, 21196) == 0
    assert p.command(209, 5, 0, 10, 1, 1, 0) != 0
    assert p.command(400, 0) == 0
    print('PASS: reboot reloads saved calibration and applies new frame', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path('build/sitl/bin/MiniDP'))
    parser.add_argument('--port', type=int, default=6170)
    args = parser.parse_args()
    directory = tempfile.mkdtemp(prefix='minidp-regression-')
    print(f'SITL artifacts: {directory}', flush=True)
    p = None
    try:
        p = Probe(args.binary.resolve(), directory, args.port)
        run(p)
    finally:
        if p:
            p.close()


if __name__ == '__main__':
    main()
