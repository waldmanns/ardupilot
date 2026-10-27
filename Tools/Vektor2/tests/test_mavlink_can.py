#!/usr/bin/env python3
"""Check routed PWM on SITL's standard multicast CAN transport.

Requires the same pymavlink installation as the other MAVLink tests. CAN packet
decoding uses only Python's standard library; no DroneCAN package or vcan setup.
Run alone (the multicast bus is shared with other SITL instances).
"""
import argparse
import binascii
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil


def run(binary):
    with tempfile.TemporaryDirectory(prefix='vektor2-can-test-') as directory:
        defaults = Path(directory) / 'defaults.parm'
        defaults.write_text(
            'SERIAL0_PROTOCOL 2\nRC_OPTIONS 1\nGPS1_TYPE 0\n'
            'CAN_P1_DRIVER 1\nCAN_D1_PROTOCOL 1\nSIM_CAN_TYPE1 1\n'
            'CAN_D1_UC_NODE 117\nCAN_D1_UC_SRV_BM 1\n'
            'CAN_D1_UC_OPTION 16\nCAN_D1_UC_SRV_RT 50\n'
            'RT1_SRC 1\nRT1_DST 1\nRC_OVERRIDE_TIME 10\n')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bus, \
                open(Path(directory) / 'sitl.log', 'w+') as log:
            bus.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            bus.bind(('', 57732))
            bus.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                           socket.inet_aton('239.65.82.0') + socket.inet_aton('0.0.0.0'))
            bus.settimeout(0.1)
            process = subprocess.Popen(
                [str(binary), '--model', 'rover', '--speedup', '1',
                 '--instance', '24', '--defaults', str(defaults),
                 '--serial0', 'tcp:5992'], cwd=directory,
                stdout=log, stderr=subprocess.STDOUT)
            link = None
            try:
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    try:
                        link = mavutil.mavlink_connection('tcp:127.0.0.1:5992',
                                                         source_system=255)
                        break
                    except OSError:
                        assert process.poll() is None, 'SITL exited'
                        time.sleep(0.1)
                assert link is not None and link.wait_heartbeat(timeout=30)
                system, component = link.target_system, link.target_component
                link.mav.command_long_send(
                    system, component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, 36, 50000, 0, 0, 0, 0, 0)

                def parameter(name, value):
                    link.mav.param_set_send(system, component, name.encode(), value,
                                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        msg = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                        if msg is not None and msg.param_id == name:
                            assert msg.param_value == value, (name, msg.param_value)
                            return
                    raise AssertionError('Parameter timeout: ' + name)

                def override(value):
                    link.mav.rc_channels_override_send(
                        system, component, value, *([65535] * 17))

                def command():
                    try:
                        packet = bus.recv(74)
                    except socket.timeout:
                        return None
                    if len(packet) < 11:
                        return None
                    magic, crc, flags, can_id = struct.unpack_from('<HHHI', packet)
                    assert magic == 0x2934
                    assert crc == binascii.crc_hqx(packet[4:], 0xffff)
                    # DroneCAN ArrayCommand type 1010 from our node, message frame.
                    if can_id & 0xff != 117 or (can_id >> 8) & 0xffff != 1010:
                        return None
                    assert flags == 0, 'Expected classic CAN'
                    data = packet[10:]
                    assert len(data) == 5 and data[-1] & 0xc0 == 0xc0
                    return struct.unpack('<BBe', data[:-1])

                def expect(actuator, pwm):
                    deadline = time.monotonic() + 5
                    can_seen = False
                    pwm_seen = actuator > 16
                    while time.monotonic() < deadline:
                        if command() == (actuator, 4, float(pwm)):
                            can_seen = True
                        msg = link.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)
                        if msg is not None and actuator <= 16:
                            pwm_seen |= getattr(msg, f'servo{actuator}_raw') == pwm
                        if can_seen and pwm_seen:
                            return
                    raise AssertionError(f'Output {actuator}: expected {pwm}, '
                                         f'CAN seen={can_seen}, local PWM seen={pwm_seen}')

                def expect_stopped():
                    # Allow route synchronization and queued frames to drain.
                    deadline = time.monotonic() + 1
                    while time.monotonic() < deadline:
                        command()
                    deadline = time.monotonic() + 0.5
                    while time.monotonic() < deadline:
                        assert command() is None, 'Disabled output still transmitted'

                for value in (1000, 1500, 2000):
                    override(value)
                    expect(1, value)
                parameter('PWM1_MIN', 1100)
                parameter('PWM1_MAX', 1900)
                parameter('PWM1_INV', 1)
                override(1250)
                expect(1, 1700)
                parameter('RT1_SRC', 0)
                expect_stopped()
                # Channels above 16 default to GPIO in the standard registry;
                # Vektor's bridge must make them available to routed CAN output.
                parameter('CAN_D1_UC_SRV_BM', 65536)
                parameter('RT1_DST', 17)
                parameter('RT1_SRC', 1)
                override(1600)
                expect(17, 1600)
                parameter('PWM17_MIN', 2200)  # invalid calibration disables output
                expect_stopped()
                print('PASS: CAN PWM endpoints, scaling, inversion, route removal, '
                      'channel 17, invalid calibration')
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
