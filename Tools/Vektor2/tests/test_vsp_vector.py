#!/usr/bin/env python3
"""Run shared VSP mapper tests under host sanitizers."""
from pathlib import Path
import os
import subprocess
import tempfile


def main():
    source = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='vektor2-mapper-test-') as directory:
        binary = Path(directory) / 'test_vsp_vector'
        subprocess.run([
            os.environ.get('CXX', '/usr/bin/g++'), '-std=c++11', '-O1', '-g',
            '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
            '-fno-omit-frame-pointer', '-I', str(source),
            str(source / 'tests/test_vsp_vector.cpp'),
            str(source / 'VspMapper.cpp'), str(source / 'VspVectorControl.cpp'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
