#!/usr/bin/env python3
"""Independent bow/stern control and heading without VSP routes, in isolated SITL."""
from pathlib import Path
import tempfile
from test_mavlink_functions import Sitl, mavutil


def run():
    binary = Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2'
    with tempfile.TemporaryDirectory(prefix='vektor2-thrusters-') as directory:
        directory = Path(directory)
        (directory / 'defaults.parm').write_text(
            'SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\nRC_OPTIONS 1\nRC_OVERRIDE_TIME -1\n'
            'GPS1_TYPE 0\nLOG_DISARMED 0\nTHR_TEL_HZ 10\nVHDG_TEL_HZ 10\nVSP_TEL_HZ 0\n')
        s = Sitl(binary, directory)
        try:
            s.start()
            for prefix in ('THR_BOW', 'THR_STN'):
                assert s.parameter(prefix + '_ENABLE') == 0
                assert s.parameter(prefix + '_HDG_EN') == 0
                s.parameter(prefix + '_SLEW', 10)
            assert s.parameter('VHDG_VSP_EN') == 1
            for n, (src, dst) in enumerate(((1, 121), (2, 131), (121, 9), (131, 10), (3, 201), (4, 202)), 1):
                s.route(n, src, dst)
            channels = [1800, 1700, 1800, 1800] + [1500] * 12 + [65535] * 2
            s.override(channels)
            s.link.mav.command_long_send(s.link.target_system, s.link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 50000, 0, 0, 0, 0, 0)
            def pwm(bow, stern):
                return s.wait('SERVO_OUTPUT_RAW', lambda m: m.servo9_raw == bow and m.servo10_raw == stern)
            pwm(1500, 1500)
            s.slot(1, type=3, enable=1)
            s.slot(2, type=4, enable=1)
            s.parameter('THR_BOW_ENABLE', 1)
            s.parameter('THR_STN_ENABLE', 1)
            s.named('THRBOW_ST', lambda v: v == 1)
            s.named('THRSTN_ST', lambda v: v == 1)
            pwm(1500, 1500)
            channels[0] = channels[1] = 1500
            s.override(channels)
            s.named('THRBOW_ST', lambda v: v == 2)
            s.named('THRSTN_ST', lambda v: v == 2)
            channels[0], channels[1] = 1800, 1700
            s.override(channels)
            pwm(1800, 1700)
            channels[2] = 1500
            s.override(channels)
            pwm(1500, 1700)
            channels[2] = 1800
            s.override(channels)
            s.named('THRBOW_ST', lambda v: v == 1)
            pwm(1500, 1700)
            print('PASS: independent VFN gates, stop outputs and neutral-before-re-enable', flush=True)
            channels[0] = channels[1] = 1500
            s.override(channels)
            for name, value in {'VHDG_VSP_EN': 0, 'VHDG_IDLE_MS': 100, 'VHDG_P': 0.1,
                                'VHDG_ENABLE': 1, 'THR_BOW_HDG_EN': 1, 'THR_STN_HDG_EN': 1,
                                'THR_STN_HDG_SIGN': -1}.items():
                s.parameter(name, value)
            s.named('HDG_STATE', lambda v: v == 4, timeout=60)
            s.named('THRBOW_ST', lambda v: v == 3)
            s.named('THRSTN_ST', lambda v: v == 3)
            # Displace simulator yaw using independent manual propulsion channels.
            s.route(7, 7, 1)
            s.route(8, 8, 3)
            channels[6] = 1500
            channels[7] = 1500
            s.override(channels)
            s.named('HDG_STATE', lambda v: v == 4)
            channels[6], channels[7] = 1700, 1700
            s.override(channels)
            s.wait('SERVO_OUTPUT_RAW', lambda m: (m.servo9_raw - 1500) * (m.servo10_raw - 1500) < 0, timeout=30)
            channels[6] = channels[7] = 1500
            channels[0] = 1800
            s.override(channels)
            s.named('HDG_STATE', lambda v: v == 2)
            pwm(1800, 1500)
            channels[0] = 1500
            s.override(channels)
            s.named('HDG_STATE', lambda v: v == 4)
            # Unrelated VSP cross-feed must not inhibit thruster-only heading.
            s.route(9, 101, 111)
            s.named('HDG_STATE', lambda v: v == 4)
            # Remove only bow permission; stern remains a usable heading actuator.
            channels[2] = 1500
            s.override(channels)
            s.named('THRBOW_ST', lambda v: v == 0)
            s.named('HDG_STATE', lambda v: v == 4)
            s.named('THRSTN_ST', lambda v: v == 3)
            channels[1] = 0
            s.override(channels)
            s.named('THRSTN_ST', lambda v: v == 4)
            pwm(1500, 1500)
            s.named('HDG_STATE', lambda v: v == 1)
            print('PASS: independent heading, opposite signs, manual release, remaining actuator and input loss', flush=True)
        except Exception:
            s.log.flush()
            s.log.seek(0)
            print(s.log.read()[-6000:])
            raise
        finally:
            s.stop()
            s.log.close()


if __name__ == '__main__':
    run()
