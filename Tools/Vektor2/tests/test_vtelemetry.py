#!/usr/bin/env python3
"""Run Vektor2 VTEL mapping / i-BUS / HoTT packet tests without ArduPilot."""
from pathlib import Path
import os
import subprocess
import tempfile


def main():
    source = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='vektor2-vtel-test-') as directory:
        binary = Path(directory) / 'test_vtelemetry'
        subprocess.run([
            os.environ.get('CXX', '/usr/bin/g++'), '-std=c++11', '-O1', '-g',
            '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
            '-fno-omit-frame-pointer', '-I', str(source),
            str(source / 'tests/test_vtelemetry.cpp'),
            str(source / 'VTelemetryCore.cpp'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
