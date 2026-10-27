#!/usr/bin/env python3
"""Exercise the production function evaluator under host ASan/UBSan."""
from pathlib import Path
import os
import subprocess
import tempfile


def main():
    source = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='vektor2-vfn-test-') as directory:
        binary = Path(directory) / 'test_functions'
        subprocess.run([
            os.environ.get('CXX', '/usr/bin/g++'), '-std=c++11', '-O1', '-g',
            '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
            '-fno-omit-frame-pointer', '-I', str(source),
            str(source / 'tests/test_functions.cpp'), str(source / 'Functions.cpp'),
            '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
