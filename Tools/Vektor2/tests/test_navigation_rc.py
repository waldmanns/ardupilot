#!/usr/bin/env python3
"""Exercise the production RC actions and unified supervisor on host."""
import os
from pathlib import Path
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='vektor2-nav-rc-') as directory:
        binary = Path(directory) / 'test_navigation_rc'
        subprocess.run([
            os.environ.get('CXX', 'g++'), '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
            '-I', str(root), str(root / 'tests/test_navigation_rc.cpp'),
            str(root / 'Functions.cpp'), str(root / 'NavigationSupervisor.cpp'),
            '-o', str(binary)
        ], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
