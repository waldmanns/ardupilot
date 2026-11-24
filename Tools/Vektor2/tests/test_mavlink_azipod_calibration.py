#!/usr/bin/env python3
"""Target/SITL acceptance for AZ2.3 USER_3 (requires built Vektor2 + pymavlink)."""
import argparse
from pathlib import Path
import tempfile
from test_mavlink_functions import Sitl, mavutil

MAV = mavutil.mavlink


def run(binary):
    with tempfile.TemporaryDirectory(prefix='vektor2-azcal-sitl-') as directory:
        directory = Path(directory)
        (directory / 'defaults.parm').write_text(
            'SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\nGPS1_TYPE 0\nLOG_DISARMED 0\n'
            'MAIN_PROP_TYPE 1\nAZI_CONF 1\nAZI2_REV 1\n')
        s = Sitl(binary, directory)
        try:
            s.start()
            s.link.target_component = s.link.messages['HEARTBEAT'].get_srcComponent()
            s.link.mav.srcComponent = 190
            for slot, (src, dst) in enumerate(((141, 1), (142, 2), (151, 3), (152, 4)), 1):
                s.route(slot, src, dst)
            target = (s.link.target_system, s.link.target_component)
            s.link.mav.command_long_send(*target, MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 50000, 0, 0, 0, 0, 0)

            def pulse(expected):
                s.wait('SERVO_OUTPUT_RAW', lambda m: tuple(getattr(m, f'servo{i}_raw') for i in range(1, 5)) == expected)

            def send(op, pod, steering, motor, token, detail=0, integer=False):
                if integer:
                    s.link.mav.command_int_send(*target, MAV.MAV_FRAME_MISSION, MAV.MAV_CMD_USER_3,
                        0, 0, op, pod, steering, motor, token, 0, 0)
                else:
                    s.link.mav.command_long_send(*target, MAV.MAV_CMD_USER_3, 0,
                        op, pod, steering, motor, token, 0, 0)
                ack = s.wait('COMMAND_ACK', lambda m: m.command == MAV.MAV_CMD_USER_3)
                assert ack.result_param2 == detail, ack
                assert ack.result == (MAV.MAV_RESULT_ACCEPTED if detail == 0 else
                    MAV.MAV_RESULT_TEMPORARILY_REJECTED if detail == 2 else
                    MAV.MAV_RESULT_FAILED if detail == 3 else MAV.MAV_RESULT_DENIED), ack

            pulse((1500, 1000, 1500, 1500))
            send(2, 1, 1500, 10, 17, detail=4)
            send(1, 1, 1500, 0, 17)
            send(2, 1, 1700, 10, 17)
            pulse((1700, 1100, 1500, 1500))
            send(2, 1, 1700, -10, 17, detail=1)
            send(3, 1, 1700, 0, 17)
            pulse((1500, 1000, 1500, 1500))
            send(1, 2, 1500, 0, 18, integer=True)
            send(2, 2, 1200, -20, 18, integer=True)
            pulse((1500, 1000, 1200, 1400))
            # No refresh: the session expires and both drives stop/rehome.
            pulse((1500, 1000, 1500, 1500))
            send(2, 2, 1200, -20, 18, detail=4, integer=True)
            print('PASS: AZ2.3 calibration LONG/INT, direction, isolated output and timeout')
        finally:
            s.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2')
    run(parser.parse_args().binary.resolve())
