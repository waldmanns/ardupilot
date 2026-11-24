#!/usr/bin/env python3
"""Host regression for the limited-angle azipod solver and controller."""
from pathlib import Path
import os
import subprocess
import tempfile

if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='vektor2-azi-') as d:
        binary = Path(d) / 'test_azipod'
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++11', '-O1', '-g',
                        '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                        '-fno-omit-frame-pointer', '-fno-pie', '-no-pie', '-I', str(root),
                        str(root/'tests/test_azipod.cpp'), str(root/'AzipodControl.cpp'),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
