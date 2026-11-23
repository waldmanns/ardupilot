#!/usr/bin/env python3
"""Sequencer parameter, command, router, telemetry and storage checks in SITL."""
import argparse
from pathlib import Path
import struct
import tempfile
import time
import zlib

from test_mavlink_functions import Sitl, mavutil

MAV = mavutil.mavlink
USER = MAV.MAV_CMD_USER_1
APPLY = MAV.MAV_CMD_USER_2


def file_fixture(revision, rate=5):
    payload = struct.pack('<BBHH', rate, 0, 1500, 2000)
    payload += struct.pack('<BBBBHBHH12s', 1, 1, 0, 0, 1350, 0, 1500, 2000, b'FTP ramp')
    payload += struct.pack('<HBBBHHH', 1, 1, 1, 1, 1200, 1250, 1750)
    data = bytearray(struct.pack('<4sBBHIII', b'VSEQ', 1, 1, 1, 20 + len(payload), revision, 0) + payload)
    crc = zlib.crc32(data)
    struct.pack_into('<I', data, 16, crc)
    return data, crc


class SequencerSitl(Sitl):
    def start(self):
        super().start()
        self.link.target_component = self.link.messages["HEARTBEAT"].get_srcComponent()
        self.link.mav.srcComponent = 190
        self.token = 0
        self.sequence = 0

    def write_rejected(self, name, value, expected):
        self.link.mav.param_set_send(self.link.target_system, self.link.target_component,
                                     name.encode(), value, MAV.MAV_PARAM_TYPE_REAL32)
        msg = self.wait('PARAM_VALUE', lambda m: m.param_id == name)
        assert msg.param_value == expected, msg

    def command(self, command, op, lane=0, revision=0, crc=0, expected=0,
                reason=0, integer=False, token=None):
        if token is None:
            self.token += 1
            token = self.token
        args = (op, lane, revision & 65535, revision >> 16, token, crc & 65535, crc >> 16)
        target = (self.link.target_system, self.link.target_component)
        if integer:
            self.link.mav.command_int_send(*target, MAV.MAV_FRAME_MISSION, command, 0, 0, *args)
        else:
            self.link.mav.command_long_send(*target, command, 0, *args)
        ack = self.wait('COMMAND_ACK', lambda m: m.command == command and m.result != MAV.MAV_RESULT_IN_PROGRESS)
        assert (ack.result, ack.result_param2) == (expected, reason), ack
        return token

    def ftp(self, opcode, data=b'', session=0, offset=0, error=None):
        self.sequence += 2
        payload = struct.pack('<HBBBBBBI', self.sequence, session, opcode,
                              len(data), 0, 0, 0, offset) + data
        self.link.mav.file_transfer_protocol_send(
            0, self.link.target_system, self.link.target_component, payload.ljust(251, b'\0'))
        response = self.wait('FILE_TRANSFER_PROTOCOL', lambda m:
                             struct.unpack_from('<H', bytes(m.payload))[0] == self.sequence + 1)
        raw = bytes(response.payload)
        _, sid, op, _, req, _, _, _ = struct.unpack_from('<HBBBBBBI', raw)
        assert req == opcode and op == (129 if error is not None else 128), (opcode, raw)
        if error is not None:
            assert raw[12] == error, raw
        return sid

    def frame(self, predicate=lambda f: True, request=True, timeout=8):
        if request:
            self.command(USER, 8)
        deadline = time.monotonic() + timeout
        current = {}
        stamp = None
        while time.monotonic() < deadline:
            msg = self.receive()
            if msg is None or msg.get_type() != 'NAMED_VALUE_INT':
                continue
            if msg.name == 'VS_BEGIN':
                stamp = msg.time_boot_ms
                current = {'VS_BEGIN': msg.value, 'timestamp': stamp}
            if stamp != msg.time_boot_ms:
                continue
            if msg.name == 'VS_END':
                assert msg.value == current['VS_BEGIN']
                assert len(current) == 5 + 5 * self.lanes, current
                if predicate(current):
                    return current
                current = {}
                stamp = None
            else:
                current[msg.name] = msg.value & 0xffffffff
        raise AssertionError('No matching coherent sequencer frame')

    def parameters(self):
        self.link.mav.param_request_list_send(self.link.target_system, self.link.target_component)
        deadline = time.monotonic() + 30
        params = {}
        total = None
        while time.monotonic() < deadline:
            msg = self.receive()
            if msg is None or msg.get_type() != 'PARAM_VALUE' or msg.param_index == 65535:
                continue
            total = msg.param_count
            assert len(msg.param_id) <= 16
            params[msg.param_index] = msg.param_id
            if len(params) == total:
                assert len(set(params.values())) == total
                return set(params.values())
        raise AssertionError(f'Incomplete PARAM_REQUEST_LIST {len(params)}/{total}')


def lane(frame, number):
    prefix = f'VSL{number:02d}_'
    block = frame[prefix + 'BLK']
    output = frame[prefix + 'OUT']
    times = frame[prefix + 'TIME']
    return dict(block=block & 511, position=block >> 9 & 63, held=block >> 15 & 1,
                state=block >> 16 & 7, valid=block >> 19 & 1, trigger=block >> 20 & 1,
                reason=block >> 21 & 15, logical=output & 65535, pwm=output >> 16,
                elapsed=frame[prefix + 'EL'], block_elapsed=times & 65535, duration=times >> 16,
                run=frame[prefix + 'RUN'])


def playback(sitl, directory):
    sitl.lanes = int(sitl.parameter('SEQ_LANES'))
    max_blocks = int(sitl.parameter('SEQ_BLOCKS'))
    assert (sitl.lanes, max_blocks) in {(4, 48), (16, 256)}
    assert sitl.parameter('SEQ_VER') == 2
    assert sitl.parameter('SEQ_SCHEMA') == 3
    assert sitl.parameter('SEQ_ENABLE') == 0
    sitl.command(USER, 8.5, expected=MAV.MAV_RESULT_DENIED, reason=4)
    sitl.command(USER, float('nan'), expected=MAV.MAV_RESULT_DENIED, reason=4)
    sysid, compid = sitl.link.target_system, sitl.link.target_component
    sitl.link.mav.command_long_send(sysid, compid + 1, USER, 0, 8, 0, 0, 0, 1, 0, 0)
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        msg = sitl.receive()
        assert msg is None or msg.get_type() != 'COMMAND_ACK' or msg.command != USER, msg
    hidden = sitl.parameters()
    assert 'SEQ_ENABLE' in hidden and 'SEQ_STORE' in hidden
    assert 'SEQ_HZ_TEL' not in hidden and 'SEQB_1_DUR' not in hidden and 'SEQ1_AUTOSTART' not in hidden
    assert sitl.parameter('SEQ1_IDLE', 1400) == 1400
    assert sitl.parameter('SEQ1_IDLE', 1500) == 1500
    sitl.command(APPLY, 1, expected=MAV.MAV_RESULT_DENIED, reason=1)
    assert sitl.parameter('SEQ_ENABLE', 1) == 1
    active = sitl.parameters()
    assert f'SEQB_{max_blocks}_DUR' in active and f'SEQ{sitl.lanes}_NAME4' in active
    assert f'SEQ{sitl.lanes}_AUTOSTART' in active
    assert f'SEQ{sitl.lanes}_REPEAT' in active
    assert len(active - hidden) == 4 + 13 * sitl.lanes + 6 * max_blocks
    sitl.write_rejected('SEQ_HZ_TEL', 11, 5)
    sitl.write_rejected('SEQ_HZ_TEL', 4.5, 5)
    sitl.write_rejected('SEQ_BLOCKS', 5, max_blocks)

    settings = {
        'SEQ_HZ_TEL': 10,
        'SEQ1_ENABLE': 1, 'SEQ1_IDLE': 1200,
        'SEQ2_ENABLE': 1, 'SEQ2_TYPE': 1, 'SEQ2_IDLE': 0,
        'SEQ3_ENABLE': 1, 'SEQ3_CLOCK': 1, 'SEQ3_IDLE': 1600,
        'SEQB_1_LANE': 1, 'SEQB_1_POS': 1, 'SEQB_1_TYPE': 1, 'SEQB_1_DUR': 4800,
        'SEQB_1_FROM': 1100, 'SEQB_1_TO': 1900,
        'SEQB_2_LANE': 3, 'SEQB_2_POS': 1, 'SEQB_2_DUR': 8000, 'SEQB_2_TO': 1800,
        f'SEQB_{max_blocks}_LANE': 2, f'SEQB_{max_blocks}_POS': 1,
        f'SEQB_{max_blocks}_DUR': 300, f'SEQB_{max_blocks}_TO': 1,
    }
    for name, value in settings.items():
        assert sitl.parameter(name, value) == value
    sitl.link.mav.srcComponent = 191
    assert sitl.parameter('SEQ1_IDLE', 1300) == 1300
    sitl.link.mav.srcComponent = 190
    sitl.route(1, 301, 1)
    sitl.route(2, 302, 2)
    sitl.route(3, 303, 3)
    assert sitl.parameter('SEQ1_IDLE', 1200) == 1200
    frame = sitl.frame(lambda f: lane(f, 1)['pwm'] == 1200 and lane(f, 2)['pwm'] == 1000)
    revision = frame['VS_REV']
    assert revision and not list(directory.glob('VSEQ*.BIN'))
    sitl.command(APPLY, 1, expected=MAV.MAV_RESULT_DENIED, reason=3)
    sitl.command(APPLY, 1, revision=revision, integer=True)
    sitl.link.mav.command_long_send(sitl.link.target_system, sitl.link.target_component,
                                   MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 50000, 0, 0, 0, 0, 0)
    sitl.wait('SERVO_OUTPUT_RAW', lambda m: m.servo1_raw == 1200 and m.servo2_raw == 1000)
    sitl.command(USER, 1, revision=revision, integer=True)
    sitl.command(USER, 5, lane=1, revision=revision, expected=MAV.MAV_RESULT_DENIED, reason=8)
    start = sitl.command(USER, 3, revision=revision)
    frame = sitl.frame(lambda f: lane(f, 1)['elapsed'] >= 300 and lane(f, 1)['state'] == 3)
    ramp, lamp = lane(frame, 1), lane(frame, 2)
    assert ramp['block'] == 1 and ramp['block_elapsed'] == ramp['elapsed'], ramp
    assert abs(ramp['logical'] - (1100 + 800 * ramp['elapsed'] / 4800)) <= 0.5, ramp
    assert lamp['held'] and lamp['block'] == max_blocks and lamp['logical'] == 1 and lamp['pwm'] == 2000
    assert lamp['run'] == ramp['run'] and lamp['elapsed'] == lamp['duration'] == 300
    sitl.command(USER, 5, lane=3, revision=revision)
    frame = sitl.frame(lambda f: lane(f, 3)['state'] == 3)
    assert lane(frame, 3)['run'] != ramp['run']
    sitl.write_rejected('SEQ1_IDLE', 777, 1200)
    sitl.command(USER, 4)
    frame = sitl.frame(lambda f: lane(f, 1)['state'] != 3)
    assert lane(frame, 1)['state'] == 1 and lane(frame, 3)['state'] == 3
    assert lane(frame, 1)['pwm'] == 1200 and lane(frame, 2)['pwm'] == 1000
    sitl.command(USER, 3, revision=revision, token=start, expected=MAV.MAV_RESULT_DENIED, reason=3)
    sitl.route(3, 303, 4)
    frame = sitl.frame(lambda f: lane(f, 3)['reason'] == 4)
    assert lane(frame, 3)['pwm'] == 1600 and lane(frame, 3)['block'] == 0
    sitl.command(USER, 2)
    assert sitl.parameter('SEQ1_IDLE', 1300) == 1300
    assert sitl.parameter('SEQ_ENABLE', 0) == 0
    frame = sitl.frame(lambda f: not f['VS_STATE'] & 1)
    assert not lane(frame, 1)['valid']
    sitl.wait('SERVO_OUTPUT_RAW', lambda m: m.servo1_raw == m.servo2_raw == m.servo4_raw == 0)
    assert sitl.parameter('SEQ1_IDLE') == 1300
    assert sitl.parameter('SEQ_ENABLE', 1) == 1
    sitl.stop()
    sitl.start()
    frame = sitl.frame(lambda f: lane(f, 1)['pwm'] == 1300)
    assert lane(frame, 2)['pwm'] == 1000 and not frame['VS_STATE'] & 2
    assert sitl.parameter(f'SEQB_{max_blocks}_LANE') == 2
    assert sitl.parameter(f'SEQB_{max_blocks}_TO') == 1
    assert sitl.parameter('SEQ1_IDLE', 1200) == 1200
    print('PASS: ordinary parameter writes and reboot persistence, playback markers and route changes', flush=True)


def storage(sitl, directory):
    sitl.command(USER, 2)
    revision = sitl.frame()['VS_REV']
    data, crc = file_fixture(1)
    sid = sitl.ftp(6, b'VSEQNEW.BIN\0')
    sitl.ftp(7, data, session=sid)
    sitl.link.mav.srcComponent = 191
    sitl.ftp(6, b'VSEQNEW.BIN\0', error=9)
    sitl.link.mav.srcComponent = 190
    sitl.command(APPLY, 2, revision=revision, crc=crc, expected=MAV.MAV_RESULT_TEMPORARILY_REJECTED, reason=2)
    sitl.ftp(2)  # Abandon a fully written staging file without a successful close.
    sitl.command(APPLY, 2, revision=revision, crc=crc, expected=MAV.MAV_RESULT_DENIED, reason=4)
    sitl.ftp(6, b'./vseq0.bin\0', error=9)
    sid = sitl.ftp(6, b'VSEQNEW.BIN\0')
    sitl.ftp(7, data, session=sid)
    sitl.ftp(1, session=sid)
    sitl.command(APPLY, 2, revision=revision, crc=crc ^ 1, expected=MAV.MAV_RESULT_DENIED, reason=4)
    sitl.command(APPLY, 2, revision=revision, crc=crc, integer=True)
    frame = sitl.frame(lambda f: lane(f, 1)['pwm'] == 1350)
    imported_revision = frame['VS_REV']
    assert imported_revision != revision and not frame['VS_STATE'] & 2
    assert sitl.parameter('SEQ1_IDLE') == 1350
    assert sitl.parameter('SEQ1_AUTOSTART') == 0  # Uploaded schema-1 fixture.
    assert sitl.parameter('SEQ2_REPEAT') == 0
    sitl.command(USER, 1, revision=imported_revision)
    start = sitl.command(USER, 3, revision=imported_revision)
    complete = sitl.frame(lambda f: lane(f, 1)['state'] == 4, request=False)
    sitl.command(USER, 3, revision=imported_revision, token=start)
    assert lane(sitl.frame(), 1)['run'] == lane(complete, 1)['run']
    sitl.stop()
    (directory / 'VSEQNEW.BIN').unlink()
    sitl.start()
    frame = sitl.frame(lambda f: f['VS_REV'] == imported_revision)
    assert frame['VS_STATE'] & 1 and not frame['VS_STATE'] & 2
    assert lane(frame, 1)['pwm'] == 1350
    print('PASS: optional FTP import validates files and persists as ordinary parameters', flush=True)


def triggers_and_zero_rate(sitl):
    assert sitl.parameter('SEQ_HZ_TEL', 0) == 0
    assert sitl.parameter('SEQ_EDGE', 1) == 1
    sitl.route(4, 1, 417)
    revision = sitl.frame(lambda f: f['VS_STATE'] >> 16 == 2)['VS_REV']
    sitl.command(USER, 1, revision=revision)
    sitl.override([1700] + [65535] * 7 + [0] * 10)
    frame = sitl.frame(lambda f: f['VS_STATE'] >> 8 & 7 == 2)
    assert lane(frame, 1)['trigger'] and frame['VS_SHARED'] == 0
    sitl.quiet(0.3)
    sitl.override([1200] + [65535] * 7 + [0] * 10)
    sitl.quiet(0.1)
    sitl.override([1700] + [65535] * 7 + [0] * 10)
    frame = sitl.frame(lambda f: lane(f, 1)['state'] == 3)
    assert lane(frame, 1)['run'] != 0
    sitl.route(5, 2, 417)  # A duplicate inhibits the whole binding.
    frame = sitl.frame(lambda f: lane(f, 1)['reason'] == 4)
    assert not lane(frame, 1)['trigger'] and lane(frame, 1)['pwm'] == 1200
    sitl.quiet(0.3)
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        msg = sitl.receive()
        assert msg is None or msg.get_type() != 'NAMED_VALUE_INT' or msg.name != 'VS_BEGIN', msg
    assert sitl.frame()['VS_REV'] == revision
    print('PASS: initial trigger baseline, rising edge, duplicate route and zero-rate status', flush=True)


def slow_link(sitl):
    # Leave a low-bandwidth link to drain while the primary link serves RC,
    # parameters and its own requested playback snapshot.
    assert sitl.parameter('SIM_BAUDLIMIT_EN', 1) == 1
    auxiliary = mavutil.mavlink_connection('tcp:127.0.0.1:6004', source_system=254,
                                           source_component=191)
    primary = sitl.link
    try:
        heartbeat = auxiliary.wait_heartbeat(timeout=8)
        assert heartbeat is not None
        auxiliary.mav.command_long_send(heartbeat.get_srcSystem(), heartbeat.get_srcComponent(),
                                        USER, 0, 8, 0, 0, 0, 1, 0, 0)
        sitl.override([1700] + [65535] * 7 + [0] * 10)
        assert sitl.parameter('SEQ_HZ_TEL') == 0
        fast_frame = sitl.frame()
        sitl.link = auxiliary
        slow_frame = sitl.frame(lambda f: f['VS_REV'] == fast_frame['VS_REV'], request=False)
        assert lane(fast_frame, 1)['pwm'] == lane(slow_frame, 1)['pwm'] == 1200
    finally:
        sitl.link = primary
        auxiliary.close()
    print('PASS: coherent per-link snapshots alongside RC and parameters', flush=True)


def boot_autostart(sitl):
    sitl.command(USER, 2)
    sitl.write_rejected('SEQ3_AUTOSTART', 2, 0)
    for name, value in {
        'SEQ3_AUTOSTART': 1, 'SEQ4_ENABLE': 1, 'SEQ4_CLOCK': 1, 'SEQ4_AUTOSTART': 1,
        'SEQB_3_LANE': 4, 'SEQB_3_POS': 1, 'SEQB_3_DUR': 8000, 'SEQB_3_TO': 1800,
    }.items():
        sitl.parameter(name, value)
    frame = sitl.frame(lambda f: lane(f, 4)['valid'])
    revision = frame['VS_REV']
    assert not frame['VS_STATE'] & 2 and lane(frame, 3)['run'] == 0

    # An incomplete sequence is saved and visible after reboot but inhibits output.
    assert sitl.parameter('SEQB_3_POS', 0) == 0
    sitl.stop()
    sitl.start()
    frame = sitl.frame(lambda f: f['VS_STATE'] >> 16 == 3)
    assert not frame['VS_STATE'] & 2 and not lane(frame, 3)['valid']
    assert sitl.parameter('SEQB_3_POS') == 0
    assert sitl.parameter('SEQB_3_POS', 1) == 1
    frame = sitl.frame(lambda f: f['VS_REV'] == revision)
    assert not frame['VS_STATE'] & 2 and lane(frame, 3)['run'] == 0
    sitl.stop()
    sitl.start()
    frame = sitl.frame(lambda f: lane(f, 3)['state'] == 3)
    assert frame['VS_REV'] == revision and frame['VS_STATE'] & 2 and frame['VS_SHARED'] == 0
    assert lane(frame, 3)['block'] == 2 and lane(frame, 1)['run'] == 0
    assert lane(frame, 4)['block'] == 3 and lane(frame, 4)['elapsed'] == lane(frame, 3)['elapsed']
    assert sitl.parameter('SEQ1_AUTOSTART') == 0 and sitl.parameter('SEQ3_AUTOSTART') == 1
    sitl.command(USER, 6, lane=3)
    stopped = sitl.frame(lambda f: lane(f, 3)['reason'] == 1)
    assert lane(stopped, 3)['state'] == 1 and lane(stopped, 3)['run'] == lane(frame, 3)['run']
    assert lane(stopped, 4)['state'] == 3

    sitl.parameter('SEQ_ENABLE', 0)
    sitl.frame(lambda f: not f['VS_STATE'] & 1)
    sitl.stop()
    sitl.start()
    frame = sitl.frame(lambda f: f['VS_REV'] == revision)
    assert not frame['VS_STATE'] & 3 and lane(frame, 3)['run'] == 0
    sitl.parameter('SEQ_ENABLE', 1)
    frame = sitl.frame(lambda f: f['VS_STATE'] & 1)
    assert not frame['VS_STATE'] & 2 and lane(frame, 3)['run'] == 0

    sitl.parameter('SEQ2_AUTOSTART', 1)
    sitl.parameter('SEQ4_AUTOSTART', 0)
    sitl.parameter(f"SEQB_{int(sitl.parameter('SEQ_BLOCKS'))}_DUR", 1500)
    sitl.route(4, 1, 0)
    sitl.route(5, 2, 0)
    revision = sitl.frame(lambda f: f['VS_REV'] != revision)['VS_REV']
    sitl.stop()
    sitl.start()
    frame = sitl.frame(lambda f: f['VS_STATE'] & (1 << 6))
    assert frame['VS_REV'] == revision and frame['VS_SHARED'] == 0
    assert lane(frame, 3)['state'] == 3 and lane(frame, 4)['run'] == 0
    sitl.override([1700] + [65535] * 7 + [0] * 10)
    sitl.route(4, 1, 417)
    frame = sitl.frame(lambda f: lane(f, 2)['state'] == 3)
    assert frame['VS_STATE'] & 2 and not frame['VS_STATE'] & (1 << 6)
    assert frame['VS_SHARED'] == lane(frame, 2)['run'] and lane(frame, 2)['trigger']
    assert lane(frame, 1)['run'] == 0 and lane(frame, 4)['run'] == 0
    sitl.command(USER, 4)
    sitl.command(USER, 3, revision=revision)
    manual = sitl.frame(lambda f: lane(f, 1)['state'] == 3)
    assert lane(manual, 1)['run'] == lane(manual, 2)['run'] != frame['VS_SHARED']
    print('PASS: per-lane autostart, shared subset, activation and trigger readiness', flush=True)


def repeat_playback(sitl):
    for name, value in {
        'SEQ_EDGE': 0, 'SEQ_HZ_TEL': 10, 'SEQ1_AUTOSTART': 1, 'SEQB_1_DUR': 600,
        'SEQ2_AUTOSTART': 1, 'SEQ2_REPEAT': 1,
        'SEQ3_AUTOSTART': 0, 'SEQ3_REPEAT': 1, 'SEQB_2_DUR': 800,
        'SEQ4_ENABLE': 0, 'SEQ4_AUTOSTART': 1, 'SEQ4_REPEAT': 1,
    }.items():
        assert sitl.parameter(name, value) == value
    sitl.write_rejected('SEQ4_REPEAT', 2, 1)
    sitl.stop()
    sitl.start()
    first = sitl.frame(lambda f: lane(f, 1)['state'] == 4 and lane(f, 2)['state'] == 3)
    later = sitl.frame(lambda f: f['timestamp'] - first['timestamp'] >= 900, request=False)
    assert lane(later, 1)['state'] == 4 and lane(later, 2)['state'] == 3
    assert lane(later, 2)['run'] == first['VS_SHARED'] == later['VS_SHARED']
    assert lane(later, 3)['run'] == lane(later, 4)['run'] == 0
    assert not lane(later, 4)['valid'] and sitl.parameter('SEQ4_REPEAT') == 1

    sitl.command(USER, 5, lane=3, revision=first['VS_REV'])
    first_manual = sitl.frame(lambda f: lane(f, 3)['state'] == 3)
    repeat = sitl.frame(lambda f: f['timestamp'] - first_manual['timestamp'] >= 800, request=False)
    assert lane(repeat, 3)['state'] == 3
    assert lane(repeat, 3)['run'] == lane(first_manual, 3)['run']
    sitl.command(USER, 7)
    stopped = sitl.frame(lambda f: lane(f, 2)['reason'] == lane(f, 3)['reason'] == 1)
    assert lane(stopped, 2)['state'] == lane(stopped, 3)['state'] == 1
    print('PASS: shared autostart and manual repeat, disabled lane persistence and stop', flush=True)


def run(binary):
    with tempfile.TemporaryDirectory(prefix='vektor2-sequencer-') as tmp:
        directory = Path(tmp)
        (directory / 'defaults.parm').write_text(
            'SERIAL0_BAUD 57\nSERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\nRC_OPTIONS 1\nLOG_BACKEND_TYPE 0\n')
        sitl = SequencerSitl(binary, directory)
        try:
            sitl.start()
            playback(sitl, directory)
            triggers_and_zero_rate(sitl)
            slow_link(sitl)
            boot_autostart(sitl)
            repeat_playback(sitl)
            assert not list(directory.glob('VSEQ*.BIN'))
            storage(sitl, directory)
        except BaseException:
            sitl.log.flush()
            print((directory / 'sitl.log').read_text()[-6000:])
            raise
        finally:
            sitl.stop()
            sitl.log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path,
                        default=Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2')
    run(parser.parse_args().binary.resolve())
