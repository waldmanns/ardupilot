#!/usr/bin/env python3
"""Host-only DP + shared yaw controller unit tests under sanitizers."""
from pathlib import Path
import os
import subprocess
import tempfile


def main():
    source = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='vektor2-dp-test-') as directory:
        binary = Path(directory) / 'test_navigation_dp'
        subprocess.run([
            os.environ.get('CXX', '/usr/bin/g++'), '-std=c++11', '-O1', '-g',
            '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
            '-fno-omit-frame-pointer', '-I', str(source),
            str(source / 'tests/test_navigation_dp.cpp'),
            str(source / 'NavigationDP.cpp'), str(source / 'NavigationHeading.cpp'),
            '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
