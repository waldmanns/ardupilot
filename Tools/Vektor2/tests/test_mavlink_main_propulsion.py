#!/usr/bin/env python3
"""AZ2 boot-selection acceptance test for a real Vektor2 SITL build (not host mocks)."""
import argparse
from pathlib import Path
import tempfile
import time
from test_mavlink_functions import Sitl, mavutil


def run(binary):
    with tempfile.TemporaryDirectory(prefix='vektor2-az2-sitl-') as directory:
        directory = Path(directory)
        (directory / 'defaults.parm').write_text(
            'SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\nRC_OPTIONS 1\nRC_OVERRIDE_TIME -1\n'
            'GPS1_TYPE 0\nLOG_DISARMED 0\nVSP_TEL_HZ 10\nTHR_TEL_HZ 10\n')
        s = Sitl(binary, directory)
        channels = [2000, 1500, 1700, 1500] + [1500] * 12 + [65535] * 2

        def read_pwm(vsp, bow=1500):
            s.link.mav.command_long_send(s.link.target_system, s.link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 50000, 0, 0, 0, 0, 0)
            s.override(channels)
            s.wait('SERVO_OUTPUT_RAW', lambda m:
                   (m.servo9_raw, m.servo10_raw, m.servo11_raw, m.servo12_raw,
                    m.servo13_raw, m.servo14_raw, m.servo15_raw, m.servo16_raw) ==
                   ((1535, 1500, 1700, bow, 0, 0, 0, 0) if vsp else (0, 0, 0, bow, 1500, 1000, 1500, 1000)))

        def restart():
            # Allow the SITL parameter storage thread to commit changes before
            # restarting in the same isolated storage directory.
            time.sleep(1)
            s.stop()
            s.last.clear()
            s.start()

        try:
            s.start()
            assert s.parameter('MAIN_PROP_TYPE') == 0
            assert s.parameter('AZI_CONF') == 0
            s.parameter('VSP1_LIM', 35)
            s.parameter('AZI2_RATE', 120)
            s.parameter('THR_BOW_ENABLE', 1)
            for slot, (src, dst) in enumerate(((1,101),(2,102),(3,103),(101,9),(102,10),(103,11),
                                              (4,121),(121,12),(1,141),(2,142),(141,13),(142,14),
                                              (1,151),(2,152),(151,15),(152,16)), 1):
                s.route(slot, src, dst)
            read_pwm(True)
            s.parameter('AZI_CONF', 1)
            s.parameter('MAIN_PROP_TYPE', 1)
            read_pwm(True)  # reboot-required selector cannot switch live
            restart()
            assert s.parameter('MAIN_PROP_TYPE') == 1
            assert s.parameter('VSP1_LIM') == 35
            assert s.parameter('AZI_CONF') == 1
            assert s.parameter('AZI2_RATE') == 120
            channels[0] = 1500
            read_pwm(False)
            channels[3] = 1800
            s.override(channels)
            s.wait('SERVO_OUTPUT_RAW', lambda m: m.servo12_raw > 1500 and m.servo9_raw == 0 and m.servo13_raw == 1500 and m.servo14_raw == 1000)
            channels[3] = 1500
            read_pwm(False)
            s.parameter('AZI_CONF', 0)
            read_pwm(False)  # layout is latched too
            restart()
            s.link.mav.command_long_send(s.link.target_system, s.link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 50000, 0, 0, 0, 0, 0)
            s.override(channels)
            s.wait('SERVO_OUTPUT_RAW', lambda m: m.servo13_raw == m.servo14_raw == m.servo15_raw == m.servo16_raw == 0)
            s.parameter('MAIN_PROP_TYPE', 0)
            s.parameter('AZI_CONF', 1)
            restart()
            channels[0] = 2000
            read_pwm(True)
            assert s.parameter('VSP1_LIM') == 35 and s.parameter('AZI2_RATE') == 120
            print('PASS: exclusive boot selection, layout-only activation, retained calibration, neutral startup, independent bow thruster')
        finally:
            s.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2')
    run(parser.parse_args().binary.resolve())
