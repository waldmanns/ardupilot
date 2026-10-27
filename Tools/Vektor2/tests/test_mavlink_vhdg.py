#!/usr/bin/env python3
"""Heading hold integration against real EKF/compass/routing in isolated SITL.

PWM9..14 observe VSP outputs. RC7/8 separately drive the rover simulator to
disturb measured yaw; this checks feedback wiring, not VSP hydrodynamics.
"""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil


NAMES = {'HDG_NOW', 'HDG_TGT', 'HDG_ERR', 'HDG_RATE', 'HDG_REQ', 'HDG_I',
         'HDG_STATE', 'HDG_INHIB', 'HDG_WAIT', 'HDG_SCALE', 'V1_HDG_US', 'V2_HDG_US'}
STATUS_NAMES = {'HDG_STATE', 'HDG_INHIB', 'HDG_REASON'}


def run(binary):
    with tempfile.TemporaryDirectory(prefix='vektor2-vhdg-sitl-') as directory:
        # Older installed pymavlink dialects predate NAMED_VALUE_STRING. Use
        # this checkout's actual wire definitions without modifying the install.
        if not hasattr(mavutil.mavlink, 'MAVLINK_MSG_ID_NAMED_VALUE_STRING'):
            repository = Path(__file__).resolve().parents[3]
            dialect = Path(directory) / 'vektor2_dialect.py'
            generation = subprocess.run([
                sys.executable, str(repository / 'modules/mavlink/pymavlink/tools/mavgen.py'),
                '--lang', 'Python3', '--wire-protocol', '2.0', '--no-validate',
                '--output', str(dialect),
                str(repository / 'modules/mavlink/message_definitions/v1.0/ardupilotmega.xml')],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            assert generation.returncode == 0, generation.stdout[-6000:]
            spec = importlib.util.spec_from_file_location('vektor2_dialect', dialect)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            mavutil.mavlink = module
        defaults = Path(directory) / 'defaults.parm'
        defaults.write_text('SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\n'
                            'RC_OPTIONS 1\nRC_OVERRIDE_TIME -1\nGPS1_TYPE 0\n'
                            'VSP_TEL_HZ 10\n')
        with open(Path(directory) / 'sitl.log', 'w+') as log:
            process = subprocess.Popen(
                [str(binary), '--model', 'rover', '--speedup', '1',
                 '--instance', '25', '--defaults', str(defaults),
                 '--serial0', 'tcp:6002', '--serial1', 'tcp:6003'],
                cwd=directory, stdout=log, stderr=subprocess.STDOUT)
            link = None
            seen = set()
            telemetry = {}
            try:
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    try:
                        link = mavutil.mavlink_connection('tcp:127.0.0.1:6003', source_system=255)
                        break
                    except OSError:
                        if process.poll() is not None:
                            raise RuntimeError('SITL exited during startup')
                        time.sleep(0.1)
                assert link is not None and link.wait_heartbeat(timeout=30), 'No heartbeat'
                system, component = link.target_system, link.target_component

                def wait_for(kind, predicate, timeout=8):
                    deadline = time.monotonic() + timeout
                    last = None
                    while time.monotonic() < deadline:
                        if process.poll() is not None:
                            raise RuntimeError(f'SITL exited with status {process.returncode}')
                        msg = link.recv_match(blocking=True, timeout=0.1)
                        if msg is not None:
                            if msg.get_type() in ('NAMED_VALUE_FLOAT', 'NAMED_VALUE_STRING') and \
                                    msg.name in NAMES | STATUS_NAMES:
                                seen.add(msg.name)
                                telemetry[msg.name] = msg.value
                            if msg.get_type() == kind:
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
                        assert abs(msg.param_value - value) < 0.0001, (name, msg.param_value)
                    return msg.param_value

                def named(name, predicate=lambda v: True, timeout=8):
                    return wait_for('NAMED_VALUE_FLOAT',
                                    lambda m: m.name == name and predicate(m.value), timeout).value

                def state(value, timeout=8):
                    return named('HDG_STATE', lambda v: int(v) == value, timeout)

                def reason(value, timeout=8):
                    return wait_for('NAMED_VALUE_STRING',
                                    lambda m: m.name == 'HDG_REASON' and m.value == value, timeout)

                channels = [1505, 1500, 1700, 1505, 1500, 1800, 1500, 1500]

                def override():
                    link.mav.rc_channels_override_send(system, component, *channels, *([65535]*10))

                def quiet(duration):
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        msg = link.recv_match(blocking=True, timeout=0.1)
                        if msg is not None and msg.get_type() in ('NAMED_VALUE_FLOAT', 'NAMED_VALUE_STRING'):
                            assert msg.name not in NAMES | STATUS_NAMES, msg

                expected = {'VHDG_ENABLE': 0, 'VHDG_VSP_EN': 1, 'VHDG_STK_DB': 30, 'VHDG_IDLE_MS': 1500,
                            'VHDG_P': 0, 'VHDG_I': 0, 'VHDG_D': 0, 'VHDG_IMAX': 0.1,
                            'VHDG_HDG_DB': 1, 'VHDG_MAX': 0.2, 'VHDG_SLEW': 0.5,
                            'VHDG_FILT_HZ': 2, 'VHDG_Y1_INV': 0, 'VHDG_Y2_INV': 0,
                            'VHDG_TEL_HZ': 5}
                for name, value in expected.items():
                    assert len(name) <= 16 and abs(parameter(name) - value) < 0.0001, name
                assert all(len(name) <= 10 for name in NAMES | STATUS_NAMES)
                routes = [(1, 101), (2, 102), (3, 103), (4, 111), (5, 112), (6, 113),
                          (101, 9), (102, 10), (103, 11), (111, 12), (112, 13), (113, 14),
                          (7, 1), (8, 3)]
                for n, (source, destination) in enumerate(routes, 1):
                    parameter(f'RT{n}_SRC', source)
                    parameter(f'RT{n}_DST', destination)
                for n in (1, 2):
                    parameter(f'VSP{n}_LIM', 100)
                override()
                link.mav.command_long_send(system, component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    36, 50000, 0, 0, 0, 0, 0)
                wait_for('SERVO_OUTPUT_RAW', lambda m:
                         (m.servo9_raw, m.servo10_raw, m.servo11_raw,
                          m.servo12_raw, m.servo13_raw, m.servo14_raw) ==
                         (1501, 1500, 1700, 1501, 1500, 1800))
                quiet(0.4)
                print('PASS: all 15 parameters/defaults, disabled baseline and telemetry silence', flush=True)

                for name, value in {'VHDG_IDLE_MS': 1200, 'VHDG_P': 0.05,
                                    'VHDG_HDG_DB': 0, 'VHDG_SLEW': 10,
                                    'VHDG_TEL_HZ': 20}.items():
                    parameter(name, value)
                parameter('VHDG_ENABLE', 1)
                state(4, timeout=45)  # real EKF compass-yaw alignment, no GPS
                assert math.isfinite(named('HDG_TGT'))
                channels[6:8] = [1750, 1600]  # external yaw disturbance
                override()
                named('HDG_ERR', lambda v: abs(v) > 5)
                wait_for('SERVO_OUTPUT_RAW', lambda m:
                         m.servo10_raw < 1490 and m.servo13_raw < 1490 and
                         m.servo9_raw == 1501 and m.servo12_raw == 1501 and
                         m.servo11_raw == 1700 and m.servo14_raw == 1800)
                channels[6:8] = [1500, 1500]
                override()
                assert NAMES | STATUS_NAMES <= seen, seen
                print('PASS: compass/EKF-gated feedback, all telemetry keys, paired Y, unchanged X/RPM', flush=True)

                parameter('VHDG_Y2_INV', 1)
                state(4)
                channels[6:8] = [1750, 1600]
                override()
                wait_for('SERVO_OUTPUT_RAW', lambda m:
                         m.servo10_raw < 1490 and m.servo13_raw > 1510)
                channels[6:8] = [1500, 1500]
                override()
                named('V1_HDG_US', lambda v: v <= -19)
                # Exercise the actual paired Logic/VRS composition, not just
                # each feature independently. Positive roll demand first asks
                # for an opposing +Y ray, then for an aiding -Y ray.
                for name, value in {'VSP1_RS_ANG': 90, 'VSP1_RS_SIGN': 1,
                                    'VSP2_RS_ANG': 270, 'VSP2_RS_SIGN': -1,
                                    'VRS_P': 0.1, 'VRS_TRIM': 5, 'VRS_SLEW': 10,
                                    'VRS_ENABLE': 1}.items():
                    parameter(name, value)
                wait_for('NAMED_VALUE_FLOAT', lambda m:
                         m.name == 'RS_INHIB' and int(m.value) & 1024)
                parameter('VSP1_RS_ANG', 270)
                wait_for('SERVO_OUTPUT_RAW', lambda m:
                         m.servo9_raw == 1501 and m.servo10_raw < 1480 and
                         m.servo11_raw == 1700 and m.servo13_raw > 1500 and
                         m.servo14_raw == 1800)
                parameter('VRS_ENABLE', 0)
                print('PASS: simultaneous VHDG/VRS, opposing-ray priority and aiding roll bursts', flush=True)
                for axis in (0, 1, 3, 4):
                    previous = channels[axis]
                    channels[axis] = 1600
                    override()
                    state(2)
                    reason('Manual override')
                    assert named('V1_HDG_US') == 0 and named('V2_HDG_US') == 0
                    assert math.isnan(named('HDG_TGT'))
                    channels[axis] = previous
                    override()
                    state(3)
                    named('HDG_WAIT', lambda v: 0 < v <= 1.2)
                    state(4)
                    assert abs(named('HDG_ERR')) < 1
                print('PASS: independent inversion, either rod overrides both axes, neutral wait and recapture', flush=True)

                for n in (1, 2, 3):
                    parameter(f'SIM_MAG{n}_FAIL', 1)
                named('HDG_INHIB', lambda v: int(v) & 1)
                reason('Compass invalid')
                assert named('V1_HDG_US') == 0 and named('V2_HDG_US') == 0
                for n in (1, 2, 3):
                    parameter(f'SIM_MAG{n}_FAIL', 0)
                state(4, timeout=20)
                saved = channels[5]
                channels[5] = 0  # release only VSP2 RPM: other controls remain valid
                override()
                named('HDG_INHIB', lambda v: int(v) & 4)
                reason('Control input lost')
                channels[5] = saved
                override()
                state(4)
                print('PASS: real compass failure/recovery and individual missing-input inhibition', flush=True)

                parameter('RT4_SRC', 101)  # direct VSP1 -> VSP2
                named('HDG_INHIB', lambda v: int(v) & 32)
                reason('VSP cross-feed')
                parameter('RT4_SRC', 4)
                state(4)
                parameter('RT15_SRC', 101)
                parameter('RT15_DST', 121)
                parameter('RT4_SRC', 121)  # indirect VSP1 -> bow -> VSP2
                named('HDG_INHIB', lambda v: int(v) & 32)
                parameter('RT4_SRC', 4)
                parameter('RT15_SRC', 1)
                parameter('RT1_SRC', 121)  # thruster component feedback inhibits heading
                named('HDG_INHIB', lambda v: int(v) & 32)
                parameter('RT1_SRC', 1)
                parameter('RT15_DST', 0)
                state(4)
                parameter('VHDG_Y1_INV', 2)
                named('HDG_INHIB', lambda v: int(v) & 8)
                reason('Config/ring invalid')
                parameter('VHDG_Y1_INV', 0)
                state(4)
                print('PASS: direct/indirect cross-feed inhibition, upstream dependency order, invalid settings', flush=True)

                parameter('VSP_TEL_HZ', 0)
                state(4)
                named('HDG_NOW')  # VSP telemetry must not gate heading telemetry

                def check_rates(rate, status_period_ms, data_period_ms, duration=3.2):
                    target = named('HDG_TGT')
                    parameter('VHDG_TEL_HZ', rate)
                    while link.recv_match(blocking=False) is not None:
                        pass
                    timestamps = {name: [] for name in ('HDG_NOW', *sorted(STATUS_NAMES))}
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        msg = link.recv_match(blocking=True, timeout=0.1)
                        if msg is None or msg.get_type() not in ('NAMED_VALUE_FLOAT', 'NAMED_VALUE_STRING'):
                            continue
                        if msg.name in timestamps:
                            timestamps[msg.name].append(msg.time_boot_ms)
                        if msg.name == 'HDG_TGT':
                            assert msg.value == target, 'Telemetry rate change recaptured target'
                        if msg.name == 'HDG_REASON':
                            assert 0 < len(msg.value) <= 64
                        if rate == 0 and msg.name in NAMES - STATUS_NAMES:
                            raise AssertionError(f'Measurement emitted at zero rate: {msg}')
                    for name in STATUS_NAMES:
                        stamps = timestamps[name]
                        assert len(stamps) >= (4 if status_period_ms == 500 else 2), (rate, name, stamps)
                        assert all(status_period_ms - 20 <= b - a <= status_period_ms + 100
                                   for a, b in zip(stamps, stamps[1:])), (rate, name, stamps)
                    # Queued fields share a sample timestamp, even if a short
                    # transmit-buffer retry straddles a measurement-window edge.
                    common = set(timestamps['HDG_STATE']) & set(timestamps['HDG_INHIB']) & \
                        set(timestamps['HDG_REASON'])
                    assert len(common) >= 2, timestamps
                    stamps = timestamps['HDG_NOW']
                    if data_period_ms is None:
                        assert not stamps, stamps
                    else:
                        assert len(stamps) >= 4, (rate, stamps)
                        assert all(data_period_ms - 20 <= b - a <= data_period_ms + 100
                                   for a, b in zip(stamps, stamps[1:])), (rate, stamps)

                check_rates(5, 1000, 200)
                check_rates(9, 1000, 111)
                check_rates(10, 500, 100)
                check_rates(20, 500, 50)
                check_rates(0, 1000, None)
                print('PASS: independent VHDG rate, 1/2 Hz status/string threshold, zero-rate status, target retention', flush=True)
                parameter('VHDG_ENABLE', 0)
                while link.recv_match(blocking=False) is not None:
                    pass
                quiet(0.5)
                print('PASS: live-disable float/string silence', flush=True)
            except Exception:
                print('Last heading telemetry:', telemetry, flush=True)
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
