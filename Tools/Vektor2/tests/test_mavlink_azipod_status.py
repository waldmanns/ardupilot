#!/usr/bin/env python3
"""AZ2.4 target/SITL acceptance; requires built Vektor2 and pymavlink."""
import argparse
import math
from pathlib import Path
import tempfile
from test_mavlink_functions import Sitl, mavutil


def run(binary):
    with tempfile.TemporaryDirectory(prefix='vektor2-azstatus-') as directory:
        directory = Path(directory)
        (directory / 'defaults.parm').write_text(
            'SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\nGPS1_TYPE 0\nLOG_DISARMED 0\n'
            'MAIN_PROP_TYPE 1\nAZI_CONF 1\nAZI2_REV 1\nAZI_TEL_HZ 5\n')
        s = Sitl(binary, directory)
        try:
            s.start()
            s.link.target_component = s.link.messages['HEARTBEAT'].get_srcComponent()
            s.link.mav.srcComponent = 190
            assert s.parameter('AZI_TEL_HZ') == 5
            for n in (1, 2):
                s.named(f'AZI{n}_STATE', lambda v: v == 0)
                s.named(f'AZI{n}_RB', lambda v: int(v) & 2)
                for suffix in ('AZPWM', 'MTPWM', 'ACMD', 'ATGT', 'MCMD'):
                    s.named(f'AZI{n}_{suffix}', math.isnan)
            for slot, (src, dst) in enumerate(((141, 1), (142, 2), (151, 3), (152, 4)), 1):
                s.route(slot, src, dst)
            # Parameter acknowledgments can arrive before the runtime routes are applied.
            s.named('AZI2_RB', lambda v: not (int(v) & 2))
            target = (s.link.target_system, s.link.target_component)
            cmd = mavutil.mavlink.MAV_CMD_USER_3

            def send(op, steering, motor):
                s.link.mav.command_long_send(*target, cmd, 0, op, 2, steering, motor, 42, 0, 0)
                ack = s.wait('COMMAND_ACK', lambda m: m.command == cmd)
                assert ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED, ack

            send(1, 1500, 0)
            for name, predicate in (
                ('AZI2_STATE', lambda v: v == 4),
                ('AZI2_RB', lambda v: v == 0),
                ('AZI1_MCMD', lambda v: v == 0),
                ('AZI2_AZPWM', lambda v: v == 1200),
                ('AZI2_MTPWM', lambda v: v == 1400),
                ('AZI2_ACMD', lambda v: abs(v + 54) < .01),
                ('AZI2_ATGT', lambda v: abs(v + 54) < .01),
                ('AZI2_MCMD', lambda v: abs(v + .2) < .001),
            ):
                send(2, 1200, -20)  # refresh lease before each observation
                s.named(name, predicate, timeout=.7)
            send(3, 1500, 0)
            s.named('AZI2_STATE', lambda v: v != 4)
            s.named('AZI2_RB', lambda v: int(v) & 32)
            print('PASS: AZ2.4 named aliases, NaN validity, reasons and calibration')
        finally:
            s.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2')
    run(parser.parse_args().binary.resolve())
