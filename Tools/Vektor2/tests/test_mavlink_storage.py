#!/usr/bin/env python3
"""Exercise SD/file/logger transports in an isolated SITL filesystem.

No physical card is touched. POSIX SITL deliberately cannot format its host
filesystem; the format test verifies rejection/failure without deleting files.
"""
import argparse
from pathlib import Path
import os
import struct
import subprocess
import tempfile
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil, DFReader


def run(binary, backend=3):
    with tempfile.TemporaryDirectory(prefix='vektor2-storage-sitl-') as directory:
        root = Path(directory)
        defaults = root / 'defaults.parm'
        defaults.write_text('SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\n'
                            'RC_OPTIONS 1\nRC_OVERRIDE_TIME -1\nGPS1_TYPE 0\n'
                            f'LOG_BACKEND_TYPE {backend}\nVSP_TEL_HZ 0\n')
        with open(root / 'sitl.log', 'w+') as log:
            process = subprocess.Popen(
                [str(binary), '--model', 'rover', '--speedup', '1', '--instance', '26',
                 '--defaults', str(defaults), '--serial0', 'tcp:6012', '--serial1', 'tcp:6013'],
                cwd=directory, stdout=log, stderr=subprocess.STDOUT)
            link = None
            remote_blocks = {}
            latest = {}
            try:
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    try:
                        link = mavutil.mavlink_connection('tcp:127.0.0.1:6013', source_system=255)
                        break
                    except OSError:
                        if process.poll() is not None:
                            raise RuntimeError('SITL exited during startup')
                        time.sleep(0.1)
                assert link is not None and link.wait_heartbeat(timeout=30), 'No heartbeat'
                system, component = link.target_system, link.target_component

                def receive(timeout=0.1):
                    if process.poll() is not None:
                        raise RuntimeError(f'SITL exited: {process.returncode}')
                    msg = link.recv_match(blocking=True, timeout=timeout)
                    if msg is not None:
                        latest[msg.get_type()] = msg
                    if msg is not None and msg.get_type() == 'REMOTE_LOG_DATA_BLOCK':
                        remote_blocks[msg.seqno] = bytes(msg.data)
                        link.mav.remote_log_block_status_send(system, component, msg.seqno,
                                                             mavutil.mavlink.MAV_REMOTE_LOG_DATA_BLOCK_ACK)
                    return msg

                def wait_for(kind, predicate=lambda m: True, timeout=10):
                    deadline = time.monotonic() + timeout
                    last = None
                    while time.monotonic() < deadline:
                        msg = receive()
                        if msg is not None and msg.get_type() == kind:
                            last = msg
                            if predicate(msg):
                                return msg
                    raise AssertionError(f'Timeout for {kind}; last={last}')

                def pump(duration):
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        receive()

                def parameter(name, value=None):
                    if value is None:
                        link.mav.param_request_read_send(system, component, name.encode(), -1)
                    else:
                        link.mav.param_set_send(system, component, name.encode(), value,
                                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    reply = wait_for('PARAM_VALUE', lambda m: m.param_id == name)
                    if value is not None:
                        assert abs(reply.param_value - value) < 0.0001, reply
                    return reply.param_value

                def command(command_id, *parameters, expected=0):
                    parameters = list(parameters) + [0] * (7 - len(parameters))
                    link.mav.command_long_send(system, component, command_id, 0, *parameters)
                    ack = wait_for('COMMAND_ACK', lambda m: m.command == command_id)
                    assert ack.result == expected, ack
                    return ack

                for name, value in {'LOG_BACKEND_TYPE': backend, 'LOG_DISARMED': 1,
                                    'LOG_BITMASK': 63, 'VLOG_HZ': 10, 'LOG_FILE_BUFSIZE': 8}.items():
                    assert abs(parameter(name) - value) < 0.0001, name
                command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
                capabilities = wait_for('AUTOPILOT_VERSION').capabilities
                assert capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FTP
                command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                        mavutil.mavlink.MAVLINK_MSG_ID_STORAGE_INFORMATION, 0)
                card = wait_for('STORAGE_INFORMATION', lambda m: m.status == 2)
                assert card.storage_id == 1 and card.storage_count == 1
                assert card.total_capacity > 0 and 0 <= card.available_capacity <= card.total_capacity
                stats = os.statvfs(directory)
                capacity_mib = stats.f_blocks * stats.f_frsize / 1048576
                assert abs(card.total_capacity - capacity_mib) < max(2, capacity_mib * 1e-6)

                if backend == 0:
                    link.mav.log_request_end_send(system, component)
                    link.mav.log_request_data_send(system, component, 1, 0, 100)
                    link.mav.log_request_list_send(system, component, 0, 65535)
                    wait_for('LOG_ENTRY', lambda m: m.num_logs == 0)
                    wait_for('HEARTBEAT')
                    print('PASS: no-backend log end/data/list requests remain safe; storage/FTP still available', flush=True)
                    return

                command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                        mavutil.mavlink.MAVLINK_MSG_ID_STORAGE_INFORMATION, 2,
                        expected=mavutil.mavlink.MAV_RESULT_UNSUPPORTED)
                command(mavutil.mavlink.MAV_CMD_REQUEST_STORAGE_INFORMATION, 1, 1)
                wait_for('STORAGE_INFORMATION', lambda m: m.status == 2)
                command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 261, 1000000)
                stamps = [wait_for('STORAGE_INFORMATION').time_boot_ms for _ in range(3)]
                assert all(950 <= b - a <= 1100 for a, b in zip(stamps, stamps[1:])), stamps
                command(mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL, 261)
                # MESSAGE_INTERVAL precedes COMMAND_ACK; retain replies seen
                # while waiting for the command acknowledgement.
                assert latest['MESSAGE_INTERVAL'].message_id == 261
                assert latest['MESSAGE_INTERVAL'].interval_us == 1000000
                command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                        mavutil.mavlink.MAVLINK_MSG_ID_MESSAGE_INTERVAL, 261)
                assert latest['MESSAGE_INTERVAL'].interval_us == 1000000
                command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 261, -0.5,
                        expected=mavutil.mavlink.MAV_RESULT_DENIED)
                command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 261, -1)
                command(mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL, 261)
                assert latest['MESSAGE_INTERVAL'].interval_us == -1
                print('PASS: logger parameters, FTP capability, storage capacity/request/interval commands', flush=True)

                routes = [(1, 101), (2, 102), (3, 103), (4, 111), (5, 112), (6, 113),
                          (101, 9), (102, 10), (103, 11), (111, 12), (112, 13), (113, 14)]
                for n, (source, destination) in enumerate(routes, 1):
                    parameter(f'RT{n}_SRC', source)
                    parameter(f'RT{n}_DST', destination)
                for name, value in {'VSP1_LIM': 100, 'VSP2_LIM': 100, 'VHDG_ENABLE': 1,
                                    'VHDG_IDLE_MS': 1000, 'VRS_ENABLE': 1,
                                    'VSP1_RS_ANG': 90, 'VSP1_RS_SIGN': 1,
                                    'VSP2_RS_ANG': 270, 'VSP2_RS_SIGN': -1,
                                    'VRS_P': 0.1, 'VRS_TRIM': 5}.items():
                    parameter(name, value)
                link.mav.rc_channels_override_send(system, component,
                    1505, 1500, 1700, 1505, 1500, 1800, *([65535]*12))
                link.mav.remote_log_block_status_send(system, component,
                    mavutil.mavlink.MAV_REMOTE_LOG_DATA_BLOCK_START,
                    mavutil.mavlink.MAV_REMOTE_LOG_DATA_BLOCK_ACK)
                wait_for('NAMED_VALUE_FLOAT', lambda m: m.name == 'HDG_STATE' and m.value == 4, timeout=45)
                wait_for('REMOTE_LOG_DATA_BLOCK', timeout=15)
                command(mavutil.mavlink.MAV_CMD_STORAGE_FORMAT, 1, 1,
                        expected=mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED)

                sequence = 0

                def ftp(opcode, data=b'', session=0, offset=0, size=None, error=None):
                    nonlocal sequence
                    sequence = (sequence + 2) & 65535
                    payload = struct.pack('<HBBBBBBI', sequence, session, opcode,
                                          len(data) if size is None else size, 0, 0, 0, offset) + data
                    payload += bytes(251 - len(payload))
                    link.mav.file_transfer_protocol_send(0, system, component, payload)
                    response = wait_for('FILE_TRANSFER_PROTOCOL', lambda m:
                        struct.unpack_from('<H', bytes(m.payload))[0] == (sequence + 1) & 65535)
                    raw = bytes(response.payload)
                    seq, sid, op, count, request, complete, _, ofs = struct.unpack_from('<HBBBBBBI', raw)
                    body = raw[12:12+count]
                    assert request == opcode and op == (129 if error is not None else 128), (opcode, raw)
                    if error is not None:
                        assert body[0] == error, body
                    return sid, body, ofs

                ftp(2)  # reset sessions
                ftp(9, b'storage-test\0')
                fixture = bytes(range(256)) * 4
                session, _, _ = ftp(6, b'storage-test/data.bin\0')
                for offset in range(0, len(fixture), 200):
                    ftp(7, fixture[offset:offset+200], session=session, offset=offset)
                ftp(1, session=session)
                _, listing, _ = ftp(3, b'storage-test\0')
                assert b'data.bin' in listing, listing
                _, checksum, _ = ftp(14, b'storage-test/data.bin\0')
                expected_crc = 0
                for byte in fixture:
                    expected_crc ^= byte
                    for _ in range(8):
                        expected_crc = (expected_crc >> 1) ^ (0xEDB88320 if expected_crc & 1 else 0)
                assert struct.unpack('<I', checksum)[0] == expected_crc
                session, size, _ = ftp(4, b'storage-test/data.bin\0')
                assert struct.unpack('<I', size)[0] == len(fixture)
                downloaded = bytearray()
                while len(downloaded) < len(fixture):
                    _, data, _ = ftp(5, session=session, offset=len(downloaded), size=239)
                    downloaded.extend(data)
                assert downloaded == fixture
                ftp(5, session=session, offset=len(fixture), size=239, error=6)  # EOF
                ftp(1, session=session)
                ftp(13, b'storage-test/data.bin\0storage-test/renamed.bin\0')
                ftp(8, b'storage-test/renamed.bin\0')
                ftp(10, b'storage-test\0')
                print('PASS: MAVFTP mkdir/list/upload/read/CRC/EOF/rename/delete/rmdir', flush=True)

                parameter('VLOG_HZ', 2)
                pump(3)
                # Append a small final marker just after a card write and stop
                # before the logger's normal 2 s flush timeout. Background close
                # must preserve the marker still in the RAM buffer.
                files = sorted((root / 'logs').glob('*.BIN'))
                assert files
                initial_size = files[-1].stat().st_size
                deadline = time.monotonic() + 6
                while files[-1].stat().st_size == initial_size and time.monotonic() < deadline:
                    receive()
                assert files[-1].stat().st_size != initial_size
                parameter('VLOG_HZ', 0)
                parameter('LOG_DISARMED', 0)
                pump(2)
                logfiles = sorted((root / 'logs').glob('*.BIN'))
                assert logfiles, 'No binary log created'
                filename = logfiles[-1]
                contents = filename.read_bytes()
                assert len(contents) > 4096 and contents[:2] == b'\xa3\x95'
                parser = DFReader.DFReader_binary(str(filename))
                records = {}
                while True:
                    message = parser.recv_msg()
                    if message is None:
                        break
                    records.setdefault(message.get_type(), []).append(message)
                required = {'FMT', 'PARM', 'VATT', 'VIMU', 'VGPS', 'VPOS', 'VVSP',
                            'VRC', 'VPWM', 'VHDG', 'VHST', 'VROL', 'VRSD', 'VSTA'}
                assert required <= records.keys(), required - records.keys()
                assert any(m.Name == 'VLOG_HZ' and m.Value == 0 for m in records['PARM']), \
                    'Final parameter marker was discarded instead of flushed on stop'
                assert any(m.State == 4 for m in records['VHDG']), 'No logged heading hold state'
                assert any(m.OutRPM == 1700 for m in records['VVSP'] if m.Drive == 1)
                assert any(m.OutRPM == 1800 for m in records['VVSP'] if m.Drive == 2)
                assert any(m.Chan == 9 and m.PWM == 1501 for m in records['VPWM'])
                intervals = [b.TimeUS - a.TimeUS for a, b in zip(records['VATT'], records['VATT'][1:])]
                assert any(95000 <= dt <= 120000 for dt in intervals), intervals
                assert any(490000 <= dt <= 520000 for dt in intervals), intervals
                assert 0 in remote_blocks and len(remote_blocks) > 10
                assert remote_blocks[0][:2] == b'\xa3\x95'
                link.mav.remote_log_block_status_send(system, component,
                    mavutil.mavlink.MAV_REMOTE_LOG_DATA_BLOCK_STOP,
                    mavutil.mavlink.MAV_REMOTE_LOG_DATA_BLOCK_ACK)
                print('PASS: decodable binary sensor/RC/VSP/heading/roll/storage logs, 10/2 Hz samples, remote block streaming', flush=True)

                link.mav.log_request_list_send(system, component, 0, 65535)
                entry = wait_for('LOG_ENTRY', lambda m: m.num_logs > 0)
                assert entry.size == len(contents), (entry.size, len(contents))
                link.mav.log_request_data_send(system, component, entry.id, 0, entry.size)
                blocks = {}
                deadline = time.monotonic() + 30
                while sum(len(block) for block in blocks.values()) < entry.size and time.monotonic() < deadline:
                    message = receive()
                    if message is not None and message.get_type() == 'LOG_DATA' and message.id == entry.id:
                        blocks[message.ofs] = bytes(message.data[:message.count])
                native_log = b''.join(blocks[key] for key in sorted(blocks))
                assert native_log == contents, (len(native_log), len(contents))
                link.mav.log_request_end_send(system, component)
                session, size, _ = ftp(4, f'logs/{filename.name}\0'.encode())
                assert struct.unpack('<I', size)[0] == len(contents)
                _, prefix, _ = ftp(5, session=session, size=239)
                assert prefix == contents[:239]
                ftp(1, session=session)
                print('PASS: LOG_ENTRY/LOG_DATA full download and MAVFTP access match the recorded binary exactly', flush=True)

                command(mavutil.mavlink.MAV_CMD_STORAGE_FORMAT, 2, 1,
                        expected=mavutil.mavlink.MAV_RESULT_UNSUPPORTED)
                command(mavutil.mavlink.MAV_CMD_STORAGE_FORMAT, 1, 0,
                        expected=mavutil.mavlink.MAV_RESULT_UNSUPPORTED)
                command(mavutil.mavlink.MAV_CMD_STORAGE_FORMAT, 1, 1,
                        expected=mavutil.mavlink.MAV_RESULT_FAILED)  # POSIX has no formatter
                assert filename.read_bytes() == contents, 'SITL format touched existing data'
                link.mav.log_erase_send(system, component)
                deadline = time.monotonic() + 15
                # Erase is asynchronous and visits every possible log number.
                # LASTLOG.TXT is removed only when that pass has completed.
                while (any((root / 'logs').glob('*.BIN')) or
                       (root / 'logs' / 'LASTLOG.TXT').exists()) and time.monotonic() < deadline:
                    receive()
                assert not list((root / 'logs').glob('*.BIN'))
                assert not (root / 'logs' / 'LASTLOG.TXT').exists()
                link.mav.log_request_list_send(system, component, 0, 65535)
                wait_for('LOG_ENTRY', lambda m: m.num_logs == 0)
                print('PASS: explicit format parameters/recording guard, nondestructive SITL refusal and LOG_ERASE', flush=True)
                parameter('VLOG_HZ', 5)
                parameter('LOG_DISARMED', 1)
                deadline = time.monotonic() + 12
                restarted = None
                while time.monotonic() < deadline:
                    candidates = sorted((root / 'logs').glob('*.BIN'))
                    if candidates and candidates[-1].stat().st_size > 4096:
                        restarted = candidates[-1]
                        break
                    receive()
                assert restarted is not None, 'Recording did not restart after stop/erase'
                pump(1.2)
                parameter('LOG_DISARMED', 0)
                pump(2)
                parser = DFReader.DFReader_binary(str(restarted))
                assert parser.recv_match(type='VHDG') is not None, 'Restarted log is not decodable'
                print('PASS: clean recording stop preserves buffered data, and recording restarts after erase', flush=True)
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read()[-7000:])
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
    binary = parser.parse_args().binary.resolve()
    run(binary)
    run(binary, backend=0)
