#!/usr/bin/env python3
"""Exercise VMODE ownership and real routed PWM outputs in isolated SITL."""
from pathlib import Path
import tempfile
from test_mavlink_functions import Sitl, mavutil


def run():
    binary = Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2'
    with tempfile.TemporaryDirectory(prefix='vektor2-vector-') as directory:
        directory = Path(directory)
        (directory / 'defaults.parm').write_text(
            'SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\nRC_OPTIONS 1\nRC_OVERRIDE_TIME -1\n'
            'GPS1_TYPE 0\nLOG_DISARMED 0\nVSP_TEL_HZ 10\n')
        s = Sitl(binary, directory)
        try:
            s.start()
            assert s.parameter('VSP1_VMODE') == s.parameter('VSP2_VMODE') == 0
            s.named('VSP1_AUTH', lambda v: v == 0)
            s.named('VSP2_AUTH', lambda v: v == 0)
            for slot, (src, dst) in enumerate(((1,101),(2,102),(101,9),(102,10),
                                              (1,111),(2,112),(111,11),(112,12),
                                              (3,113),(113,13)), 1):
                s.route(slot, src, dst)
            s.link.mav.command_long_send(s.link.target_system, s.link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 50000, 0,0,0,0,0)
            channels = [2000,1500,1700] + [1500]*13 + [65535]*2
            s.override(channels)
            def pwm(x1,y1,x2,y2):
                s.wait('SERVO_OUTPUT_RAW', lambda m:
                       (m.servo9_raw,m.servo10_raw,m.servo11_raw,m.servo12_raw) == (x1,y1,x2,y2))
            pwm(1525,1500,1525,1500)  # default legacy
            s.named('VSP1_AUTH', lambda v: v == 1)
            s.named('VSP2_AUTH', lambda v: v == 1)
            s.parameter('VSP1_DIR', 1)  # CCW
            s.parameter('VSP1_VMODE', 1)
            s.named('VSP1_VST', lambda v: v == 1)
            s.named('VSP1_AUTH', lambda v: v == 8)
            pwm(1500,1500,1525,1500)  # independent legacy second drive
            channels[0] = 1500
            s.override(channels)
            s.named('VSP1_VST', lambda v: v == 2)
            channels[0] = 2000
            s.override(channels)
            pwm(1475,1500,1525,1500)  # same input, CCW vector reverses X
            s.named('VSP1_TDIR', lambda v: abs(v) < 0.1)
            # THR_ANG must not be applied a second time in vector mode.
            s.parameter('VSP1_THR_ANG', 90)
            pwm(1475,1500,1525,1500)
            s.parameter('VSP2_VMODE', 1)
            pwm(1475,1500,1500,1500)
            s.parameter('VRS_ENABLE', 1)
            s.parameter('VHDG_ENABLE', 1)
            channels[0] = 1500
            s.override(channels)
            s.named('VSP2_VST', lambda v: v == 2)
            channels[0] = 2000
            s.override(channels)
            pwm(1475,1500,1525,1500)
            s.named('VSP1_AUTH', lambda v: v == 1)
            s.named('VSP2_AUTH', lambda v: v == 1)
            s.wait('SERVO_OUTPUT_RAW', lambda m: m.servo13_raw == 1700)  # RPM passthrough
            s.parameter('VSP1_MAP_ANG', 90)
            pwm(1500,1500,1525,1500)
            channels[0] = 1500
            s.override(channels)
            s.named('VSP1_VST', lambda v: v == 2)
            channels[0] = 2000
            s.override(channels)
            pwm(1500,1525,1525,1500)
            # Leaving vector mode also requires neutral; legacy rotation returns.
            s.parameter('VSP1_THR_ANG', 0)
            s.parameter('VSP1_VMODE', 0)
            pwm(1500,1500,1525,1500)
            channels[0] = 1500
            s.override(channels)
            s.named('VSP1_VST', lambda v: v == 0)
            channels[0] = 2000
            s.override(channels)
            pwm(1525,1500,1525,1500)
            # Return both to vector control, then expire RC without removing routes.
            s.parameter('VSP1_VMODE', 1)
            channels[0] = 1500
            s.override(channels)
            s.named('VSP1_VST', lambda v: v == 2)
            channels[0] = 2000
            s.override(channels)
            pwm(1500,1525,1525,1500)
            s.parameter('RC_OVERRIDE_TIME', 0.3)
            s.named('VSP1_VST', lambda v: v == 3)
            s.named('VSP2_VST', lambda v: v == 3)
            s.named('VSP1_AUTH', lambda v: v == 8)
            s.named('VSP2_AUTH', lambda v: v == 8)
            pwm(1500,1500,1500,1500)
            s.parameter('RC_OVERRIDE_TIME', -1)
            s.override(channels)
            s.named('VSP1_VST', lambda v: v == 1)
            s.named('VSP1_AUTH', lambda v: v == 8)
            pwm(1500,1500,1500,1500)
            print('PASS: VMODE defaults, independent drives, transitions, rotation, angle, RPM and RC-loss neutral PWM')
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
