#!/usr/bin/env python3
"""VFN parameters, every router source, controller gates and persistence in SITL.

Uses an isolated parameter store and local TCP ports 6004/6005. The simulator
does not model VSP hydrodynamics; existing VRS/VHDG suites check their feedback.
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


class Sitl:
    def __init__(self, binary, directory):
        self.binary = binary
        self.directory = directory
        self.process = None
        self.link = None
        self.log = open(directory / 'sitl.log', 'a+')
        self.last = {}

    def start(self):
        self.process = subprocess.Popen([
            str(self.binary), '--model', 'rover', '--speedup', '1', '--instance', '26',
            '--defaults', str(self.directory / 'defaults.parm'),
            '--serial0', 'tcp:6004', '--serial1', 'tcp:6005'],
            cwd=self.directory, stdout=self.log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            assert self.process.poll() is None, 'SITL exited during startup'
            try:
                self.link = mavutil.mavlink_connection('tcp:127.0.0.1:6005', source_system=255)
                break
            except OSError:
                time.sleep(0.1)
        assert self.link is not None and self.link.wait_heartbeat(timeout=30), 'No heartbeat'

    def stop(self):
        if self.link is not None:
            self.link.close()
            self.link = None
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            self.process = None

    def receive(self):
        assert self.process.poll() is None, 'SITL exited'
        msg = self.link.recv_match(blocking=True, timeout=0.1)
        if msg is not None and msg.get_type() == 'NAMED_VALUE_FLOAT':
            self.last[msg.name] = msg.value
        return msg

    def wait(self, kind, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.receive()
            if msg is not None and msg.get_type() == kind and predicate(msg):
                return msg
        raise AssertionError(f'Timed out waiting for {kind}; VFN={self.last}')

    def parameter(self, name, value=None):
        if value is None:
            self.link.mav.param_request_read_send(
                self.link.target_system, self.link.target_component, name.encode(), -1)
        else:
            self.link.mav.param_set_send(
                self.link.target_system, self.link.target_component, name.encode(), value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        msg = self.wait('PARAM_VALUE', lambda m: m.param_id == name)
        if value is not None:
            assert abs(msg.param_value - value) < 0.0001, (name, value, msg.param_value)
        return msg.param_value

    def snapshot(self, keys, predicate, timeout=8):
        samples = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.receive()
            if msg is None or msg.get_type() != 'NAMED_VALUE_FLOAT' or msg.name not in keys:
                continue
            row = samples.setdefault(msg.time_boot_ms, {})
            row[msg.name] = msg.value
            if set(keys) <= row.keys() and predicate(row):
                return row
        raise AssertionError(f'No matching coherent VFN snapshot; last samples: {list(samples.items())[-2:]}')

    def override(self, channels):
        self.link.mav.rc_channels_override_send(
            self.link.target_system, self.link.target_component, *channels)

    def route(self, slot, source, destination):
        self.parameter(f'RT{slot}_SRC', source)
        self.parameter(f'RT{slot}_DST', destination)

    def slot(self, number, **settings):
        for suffix, value in settings.items():
            self.parameter(f'VFN{number}_{suffix.upper()}', value)

    def named(self, name, predicate=lambda v: True, timeout=8):
        return self.wait('NAMED_VALUE_FLOAT', lambda m: m.name == name and predicate(m.value), timeout).value

    def quiet(self, duration=0.5):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            msg = self.receive()
            assert msg is None or msg.get_type() != 'NAMED_VALUE_FLOAT' or not msg.name.startswith('VFN'), msg


def run(binary):
    with tempfile.TemporaryDirectory(prefix='vektor2-vfn-sitl-') as directory:
        directory = Path(directory)
        (directory / 'defaults.parm').write_text(
            'SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\nRC_OPTIONS 1\n'
            'RC_OVERRIDE_TIME -1\nGPS1_TYPE 0\nVSP_TEL_HZ 0\nVHDG_TEL_HZ 0\nLOG_DISARMED 0\n')
        sitl = Sitl(binary, directory)
        try:
            sitl.start()
            for n in range(1, 11):
                for suffix, value in {'ENABLE': 0, 'TYPE': 0, 'MODE': 0, 'LOW': 1700, 'HIGH': 2000}.items():
                    name = f'VFN{n}_{suffix}'
                    assert len(name) <= 16 and sitl.parameter(name) == value, name
            assert sitl.parameter('VFN_TEL_HZ') == 1
            sitl.quiet()
            print('PASS: all 51 VFN parameters/defaults and no unconfigured telemetry', flush=True)

            # No physical propulsion channels are driven. Expose VSP outputs
            # on PWM9..14, plus two independent upstream component sources.
            routes = [(1, 101), (2, 102), (3, 103), (4, 111), (5, 112), (6, 113),
                      (101, 9), (102, 10), (103, 11), (111, 12), (112, 13), (113, 14),
                      (7, 121), (8, 131)]
            for slot, (source, destination) in enumerate(routes, 1):
                sitl.route(slot, source, destination)
            for n in (1, 2):
                sitl.parameter(f'VSP{n}_LIM', 100)
            # Standard RC_Channels owns 16 MAVLink overrides; physical RC can
            # supply all 18 router channels. Exercise the last override here.
            channels = [1505, 1500, 1700, 1505, 1500, 1800, 1550, 1650] + [1500] * 7 + [2100]
            channels += [65535, 65535]
            sitl.override(channels)
            sitl.link.mav.command_long_send(sitl.link.target_system, sitl.link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 50000, 0, 0, 0, 0, 0)
            sitl.wait('SERVO_OUTPUT_RAW', lambda m: m.servo9_raw == 1501 and m.servo11_raw == 1700)
            sources = [1, 101, 102, 103, 111, 112, 113, 121, 131, 16]
            expected = [1505, 1501, 1500, 1700, 1501, 1500, 1800, 1500, 1500, 2100]
            for n, source in enumerate(sources, 1):
                sitl.route(14 + n, source, 200 + n)
                sitl.slot(n, low=800, high=2200, type=1, enable=1)
            sitl.parameter('VFN_TEL_HZ', 5)
            keys = [f'VFN{n}_{suffix}' for n in range(1, 11) for suffix in ('IN', 'ST', 'ERR')]
            keys += ['VFN_VRS', 'VFN_HDG']
            assert all(len(name) <= 10 for name in keys)
            sitl.snapshot(keys, lambda row: all(
                row[f'VFN{n}_IN'] == expected[n-1] and row[f'VFN{n}_ST'] == 31 and row[f'VFN{n}_ERR'] == 0
                for n in range(1, 11)) and row['VFN_VRS'] == 0 and row['VFN_HDG'] == 0)
            print('PASS: all ten slots, every router source, fan-out, full-range RC and coherent telemetry', flush=True)

            # Return to one action per feature. All other slots remain routed
            # but unused, so their live inputs cannot accidentally request it.
            for n in range(2, 11):
                sitl.slot(n, type=0, enable=0)
            sitl.route(15, 16, 201)
            sitl.slot(1, low=2050, high=2150)
            sitl.parameter('VRS_ENABLE', 1)
            sitl.named('VFN_VRS', lambda v: v == 1)
            for pulse, active in ((2050, 1), (2150, 1), (2049, 0), (2151, 0), (800, 0), (2200, 0)):
                channels[15] = pulse
                sitl.override(channels)
                sitl.snapshot(['VFN1_IN', 'VFN1_ST', 'VFN_VRS'], lambda row:
                    row['VFN1_IN'] == pulse and bool(int(row['VFN1_ST']) & 16) == bool(active)
                    and row['VFN_VRS'] == active)
            sitl.slot(1, low=800, high=800)
            channels[15] = 800
            sitl.override(channels)
            sitl.named('VFN_VRS', lambda v: v == 1)
            sitl.slot(1, low=2200, high=2200)
            channels[15] = 2200
            sitl.override(channels)
            sitl.named('VFN_VRS', lambda v: v == 1)
            print('PASS: inclusive adjustable bounds including 800/2200, master-gated VRS activation', flush=True)

            for name, value in {'VSP1_RS_ANG': 90, 'VSP1_RS_SIGN': 1,
                                'VSP2_RS_ANG': 270, 'VSP2_RS_SIGN': -1,
                                'VRS_P': 0.1, 'VRS_TRIM': 5, 'VRS_SLEW': 10,
                                'VRS_ON_MS': 200, 'VRS_OFF_MS': 50}.items():
                sitl.parameter(name, value)
            sitl.wait('SERVO_OUTPUT_RAW', lambda m: m.servo10_raw > 1500, timeout=45)
            channels[15] = 2199
            sitl.override(channels)
            sitl.named('VFN_VRS', lambda v: v == 0)
            baseline = (1501, 1500, 1700, 1501, 1500, 1800)
            deadline = time.monotonic() + 0.8
            observed = 0
            while time.monotonic() < deadline:
                msg = sitl.receive()
                if msg is not None and msg.get_type() == 'SERVO_OUTPUT_RAW':
                    assert tuple(getattr(msg, f'servo{n}_raw') for n in range(9, 15)) == baseline, msg
                    observed += 1
            assert observed >= 5
            sitl.parameter('VRS_P', 0)
            print('PASS: VFN gate drives real VRS corrections and removes them from physical PWM', flush=True)

            # Configure while disabled so the old 2200-us input cannot arm
            # the toggle before the intended initial in-range sample arrives.
            sitl.slot(1, enable=0)
            channels[15] = 1800
            sitl.override(channels)
            sitl.slot(1, low=1700, high=2000, mode=1, enable=1)
            sitl.named('VFN_VRS', lambda v: v == 0)
            channels[15] = 1500
            sitl.override(channels)
            sitl.named('VFN1_ST', lambda v: int(v) & 32)
            channels[15] = 1800
            sitl.override(channels)
            sitl.named('VFN_VRS', lambda v: v == 1)
            sitl.slot(2, type=1, enable=1, low=1700, high=2000)
            sitl.named('VFN_VRS', lambda v: v == 1)  # unrelated slot edit retains F1 latch
            channels[15] = 65534  # release only extension channel 16
            sitl.override(channels)
            sitl.snapshot(['VFN1_ERR', 'VFN_VRS'], lambda row: row['VFN1_ERR'] == 8 and row['VFN_VRS'] == 0)
            channels[15] = 1800
            sitl.override(channels)
            sitl.snapshot(['VFN1_IN', 'VFN1_ST', 'VFN_VRS'], lambda row:
                row['VFN1_IN'] == 1800 and row['VFN1_ST'] == 15 and row['VFN_VRS'] == 0)
            print('PASS: toggle arming/latching, individual source loss and recovery without a false entry', flush=True)

            sitl.slot(1, mode=0)
            sitl.named('VFN_VRS', lambda v: v == 1)
            sitl.route(25, 7, 201)  # duplicate producer must inhibit F1, not retain the first route
            sitl.snapshot(['VFN1_ERR', 'VFN_VRS'], lambda row: row['VFN1_ERR'] == 4 and row['VFN_VRS'] == 0)
            sitl.route(25, 0, 0)
            sitl.named('VFN_VRS', lambda v: v == 1)
            sitl.route(15, 2661, 201)  # must not wrap a large component code back to VSP1
            sitl.snapshot(['VFN1_ERR', 'VFN_VRS'], lambda row: int(row['VFN1_ERR']) & 4 and row['VFN_VRS'] == 0)
            sitl.route(15, 16, 201)
            sitl.slot(1, low=2100, high=2000)
            sitl.snapshot(['VFN1_ERR', 'VFN_VRS'], lambda row: row['VFN1_ERR'] == 1 and row['VFN_VRS'] == 0)
            sitl.slot(1, low=1700)
            sitl.named('VFN_VRS', lambda v: v == 1)
            sitl.slot(1, enable=0)
            sitl.named('VFN_VRS', lambda v: v == 0)
            sitl.slot(2, low=1400, high=1600)  # VSP1 X output requests the same feature
            sitl.named('VFN_VRS', lambda v: v == 1)
            channels[2] = 0  # upstream loss leaves nonzero geometry but invalid producer
            sitl.override(channels)
            sitl.snapshot(['VFN2_ERR', 'VFN2_IN', 'VFN_VRS'], lambda row:
                row['VFN2_ERR'] == 8 and math.isnan(row['VFN2_IN']) and row['VFN_VRS'] == 0)
            channels[2] = 1700
            sitl.override(channels)
            sitl.named('VFN_VRS', lambda v: v == 1)
            print('PASS: OR arbitration, duplicate/invalid routes, invalid ranges and propagated component validity', flush=True)

            # VSP1's output controls Auto heading on the same VSP pair. This is
            # supported by next-cycle application, not rejected as a graph cycle.
            sitl.slot(2, type=2)
            sitl.parameter('VHDG_IDLE_MS', 300)
            sitl.parameter('VHDG_ENABLE', 1)
            sitl.named('VFN_HDG', lambda v: v == 1)
            sitl.named('HDG_STATE', lambda v: v == 4, timeout=45)
            sitl.parameter('VHDG_ENABLE', 0)
            sitl.named('VFN_HDG', lambda v: v == 0)
            sitl.parameter('VHDG_ENABLE', 1)
            sitl.named('HDG_STATE', lambda v: v == 4)
            sitl.slot(2, low=1700, high=2000)
            sitl.named('VFN_HDG', lambda v: v == 0)
            sitl.slot(2, low=1400, high=1600)
            sitl.named('HDG_STATE', lambda v: v == 4)
            print('PASS: component-to-heading feedback, master enable and target reacquisition', flush=True)

            # Save a latched-on toggle, then restart using the same EEPROM. The
            # configuration must persist while the runtime latch starts off.
            sitl.slot(2, type=0, enable=0)
            sitl.slot(1, enable=1, mode=1)
            channels[15] = 1500
            sitl.override(channels)
            sitl.named('VFN1_ST', lambda v: int(v) & 32)
            channels[15] = 1800
            sitl.override(channels)
            sitl.named('VFN_VRS', lambda v: v == 1)
            persisted = {name: sitl.parameter(name) for name in
                         ('VFN1_ENABLE', 'VFN1_TYPE', 'VFN1_MODE', 'VFN1_LOW', 'VFN1_HIGH',
                          'VFN10_LOW', 'VFN_TEL_HZ', 'RT15_SRC', 'RT15_DST')}
            sitl.stop()
            sitl.start()
            for name, expected_value in persisted.items():
                assert sitl.parameter(name) == expected_value, name
            sitl.override(channels)
            sitl.snapshot(['VFN1_ST', 'VFN_VRS'], lambda row: row['VFN1_ST'] == 15 and row['VFN_VRS'] == 0)
            sitl.parameter('VFN_TEL_HZ', 0)
            # Flush already queued samples before checking that no new ones arrive.
            deadline = time.monotonic() + 0.4
            while time.monotonic() < deadline:
                sitl.receive()
            sitl.quiet()
            sitl.slot(1, type=0, enable=0)
            sitl.parameter('VFN_TEL_HZ', 5)
            sitl.named('VFN_VRS', lambda v: v == 1)  # no assignments: legacy master behavior
            print('PASS: persistent configuration, volatile toggle latch, zero telemetry rate and legacy fallback', flush=True)
        except Exception:
            sitl.log.flush()
            sitl.log.seek(0)
            print(sitl.log.read()[-6000:], flush=True)
            raise
        finally:
            sitl.stop()
            sitl.log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path,
                        default=Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2')
    run(parser.parse_args().binary.resolve())
