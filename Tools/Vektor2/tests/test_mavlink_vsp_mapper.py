#!/usr/bin/env python3
"""Check mapper estimates against final rod outputs over real MAVLink in SITL."""
from pathlib import Path
import math
import tempfile
from test_mavlink_functions import Sitl


def run():
    binary = Path(__file__).resolve().parents[3] / 'build/sitl/bin/Vektor2'
    with tempfile.TemporaryDirectory(prefix='vektor2-mapper-sitl-') as directory:
        directory = Path(directory)
        (directory / 'defaults.parm').write_text(
            'SERIAL1_PROTOCOL 2\nSERIAL1_BAUD 460\nRC_OPTIONS 1\nRC_OVERRIDE_TIME -1\n'
            'GPS1_TYPE 0\nLOG_DISARMED 0\nVSP_TEL_HZ 10\n')
        s = Sitl(binary, directory)
        try:
            s.start()
            for n in (1, 2):
                assert s.parameter(f'VSP{n}_DIR') == 0
                s.named(f'VSP{n}_TMAG', math.isnan)
            for slot, (src, dst) in enumerate(((1,101), (2,102), (3,103),
                                              (1,111), (2,112), (3,113)), 1):
                s.route(slot, src, dst)
            channels = [1000,1500,1500] + [1500]*13 + [65535]*2
            s.override(channels)
            s.parameter('VSP1_DIR', 1)  # CCW
            s.parameter('VSP2_DIR', 0)  # CW
            def check(direction1, direction2):
                for n, direction in ((1,direction1), (2,direction2)):
                    s.named(f'VSP{n}_TDIR', lambda v: abs(v-direction) < 0.1)
                    s.named(f'VSP{n}_TMAG', lambda v: abs(v-1) < 0.001)
                    s.named(f'VSP{n}_XA', lambda v: v == 1475)
                    s.named(f'VSP{n}_YA', lambda v: v == 1500)
            check(0, 180)
            # Separate RPM routing must not suppress rod-vector diagnostics.
            s.route(3, 0, 103)
            s.route(6, 0, 113)
            check(0, 180)
            # Nor should an assigned RPM route with an unavailable source.
            s.route(3, 18, 103)
            s.route(6, 18, 113)
            check(0, 180)
            for layout in (1, 2):
                s.parameter('VSP_CONF', layout)
                check(0, 180)
            s.parameter('VSP1_MAP_ANG', 90)
            check(90, 180)  # observation changes, rods do not
            s.parameter('VSP1_DIR', 2)  # invalid enum
            s.named('VSP1_TMAG', math.isnan)
            s.named('VSP1_TDIR', math.isnan)
            s.parameter('VSP1_DIR', 1)
            channels[0] = 1500
            s.override(channels)
            for n in (1,2):
                s.named(f'VSP{n}_TMAG', lambda v: v == 0)
                s.named(f'VSP{n}_TDIR', math.isnan)
            # Missing either rod input still invalidates that drive's estimate.
            s.route(1, 0, 101)
            s.named('VSP1_TMAG', math.isnan)
            s.named('VSP1_TDIR', math.isnan)
            s.route(1, 1, 101)
            s.named('VSP1_TMAG', lambda v: v == 0)
            s.route(2, 0, 102)
            s.named('VSP1_TMAG', math.isnan)
            s.named('VSP2_TMAG', lambda v: v == 0)
            s.route(2, 2, 102)
            s.parameter('VSP1_MAP_ANG', 0)
            channels[0] = channels[1] = 1000
            s.override(channels)
            for n, direction in ((1, 45), (2, 225)):
                s.named(f'VSP{n}_XA', lambda v: v == 1482)
                s.named(f'VSP{n}_YA', lambda v: v == 1482)
                s.named(f'VSP{n}_TDIR', lambda v: abs(v-direction) < 0.1)
                s.named(f'VSP{n}_TMAG', lambda v: v == 1)
            print('PASS: VSP direction/intensity telemetry, layouts, neutral and invalidation')
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
