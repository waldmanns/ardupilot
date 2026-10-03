#!/usr/bin/env python3
"""Compile and run VRS controller/driver tests with address/undefined sanitizers.

Uses only the host C++ compiler, production controller/VSP sources and a tiny
telemetry transport stub. Build products are isolated in a temporary directory.
"""
from pathlib import Path
import os
import subprocess
import tempfile


def main():
    source = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='vektor2-vrs-test-') as directory:
        binary = Path(directory) / 'test_roll_stabilization'
        command = [os.environ.get('CXX', '/usr/bin/g++'), '-std=c++11', '-O1', '-g',
                   '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                   '-fno-omit-frame-pointer', '-I', str(source),
                   str(source / 'tests/test_roll_stabilization.cpp'),
                   str(source / 'RollStabilization.cpp'), str(source / 'LibVSP.c'),
                   str(source / 'VSP1.cpp'), str(source / 'VSP2.cpp'), '-o', str(binary)]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
