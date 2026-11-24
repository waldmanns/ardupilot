#!/usr/bin/env python3
"""Exercise coordinated VSP mode handovers and fault rearm in SITL."""
import argparse
from math import isfinite
from pathlib import Path
from tempfile import TemporaryDirectory

from test_mavlink_functions import Sitl, mavutil


def run(binary):
    params = {
        'SERIAL1_PROTOCOL': 2, 'SERIAL1_BAUD': 460, 'RC_OPTIONS': 1,
        'RC_OVERRIDE_TIME': -1, 'GPS1_TYPE': 2, 'LOG_DISARMED': 0,
        'VSP_TEL_HZ': 10, 'DP_TEL_HZ': 5, 'VSP_CONF': 2,
        'VSP1_VMODE': 1, 'VSP2_VMODE': 1, 'VSP1_LIM': 60, 'VSP2_LIM': 60,
        'VNAV_ENABLE': 1, 'VNAV_HDG_ENABLE': 1, 'VNAV_HDG_KP': 0.02,
        'VNAV_DP_ENABLE': 1, 'VNAV_DP_POS_KP': 0.5, 'VNAV_DP_VEL_KP': 1,
        'VNAV_DP_MAX_SPD': 1, 'VNAV_DP_ACC_LIM': 1,
        'VNAV_VSP_EN': 1, 'VNAV_VSP1_X': 1, 'VNAV_VSP2_X': 1,
        'VNAV_VSP1_Y': -1, 'VNAV_VSP2_Y': 1,
        'VNAV_VSP1_F': 1, 'VNAV_VSP2_F': 1,
        'VNAV_AL_MAX_SRG': 1, 'VNAV_AL_MAX_SWAY': 1, 'VNAV_AL_MAX_YAW': 1,
        'VNAV_VSP_RPM': 900,  # Older configuration, with no measured shaft RPM.
    }
    routes = ((1, 101), (2, 102), (3, 111), (4, 112), (101, 9), (102, 10), (111, 11), (112, 12))
    for slot, (source, target) in enumerate(routes, 1):
        params[f'RT{slot}_SRC'] = source
        params[f'RT{slot}_DST'] = target

    with TemporaryDirectory(prefix='vektor2-vsp-nav-sitl-') as folder:
        directory = Path(folder)
        (directory / 'defaults.parm').write_text(''.join(f'{key} {value}\n' for key, value in params.items()))
        sitl = Sitl(binary, directory)
        try:
            sitl.start()
            channels = [1500] * 16 + [65535] * 2
            sitl.override(channels)
            sitl.link.mav.command_long_send(sitl.link.target_system, sitl.link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 50000, 0, 0, 0, 0, 0)

            def state(value, timeout=10):
                sitl.named('NAV_STATE', lambda state: state == value, timeout=timeout)

            def pilot(forward):
                channels[0] = channels[2] = forward
                sitl.override(channels)

            sitl.named('VSP1_VST', lambda value: value == 2)
            sitl.named('VSP2_VST', lambda value: value == 2)
            assert sitl.parameter('VNAV_VSP_RPM') == 900
            sitl.parameter('VNAV_MODE', 2)
            state(4, timeout=60)  # Free directly to DP, once the EKF is ready.
            sitl.named('VSP1_AUTH', lambda value: value == 18)
            sitl.parameter('VNAV_MODE', 0)
            state(0)

            pilot(1900)
            sitl.wait('SERVO_OUTPUT_RAW', lambda m: m.servo9_raw > 1510 and m.servo11_raw > 1510)
            sitl.parameter('VNAV_MODE', 1)
            state(2)
            heading = sitl.named('HDG_TGT', isfinite)
            sitl.parameter('VNAV_MODE', 2)
            state(3)  # Heading stays in control while the sticks are off center.
            sitl.named('VSP1_AUTH', lambda value: value == 3)
            sitl.named('NAV_RBITS', lambda value: int(value) & (1 << 6))
            assert int(sitl.last['NAV_RBITS']) & ((1 << 14) | (1 << 15)) == 0
            pilot(1500)
            state(4)
            sitl.named('VSP1_AUTH', lambda value: value == 18)
            sitl.named('VSP_AL_ST', lambda value: int(value) & 0x80)
            assert int(sitl.last['VSP_AL_ST']) & ((1 << 8) | (1 << 10) | (1 << 13)) == 0

            sitl.parameter('VNAV_MODE', 1)
            state(2)
            sitl.named('VSP1_AUTH', lambda value: value == 3)
            assert abs(sitl.named('HDG_TGT', isfinite) - heading) < 0.01
            sitl.parameter('VNAV_MODE', 0)
            state(0)

            sitl.parameter('VNAV_MODE', 2)
            state(4)
            pilot(1900)
            sitl.named('NAV_RBITS', lambda value: int(value) & (1 << 14))
            sitl.parameter('VNAV_MODE', 1)
            state(1)
            sitl.named('NAV_RBITS', lambda value: int(value) & (1 << 14))
            sitl.parameter('VNAV_MODE', 0)
            state(0)
            print('PASS: all mode handovers, DP wait, retained heading, ignored RPM threshold, and pilot-override rearm')
        finally:
            sitl.stop()
            sitl.log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2')
    run(parser.parse_args().binary.resolve())
