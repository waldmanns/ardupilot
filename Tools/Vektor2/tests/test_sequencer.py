#!/usr/bin/env python3
"""Exercise both sequencer profiles under host ASan and UBSan."""
from pathlib import Path
import os
import subprocess
import tempfile


def main():
    source = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='vektor2-seq-test-') as directory:
        binary = Path(directory) / 'test_sequencer'
        for large in (0, 1):
            subprocess.run([
                os.environ.get('CXX', '/usr/bin/g++'), '-std=c++11', '-O2', '-g',
                '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                '-fno-omit-frame-pointer', '-no-pie', '-I', str(source),
                f'-DVEKTOR2_SEQ_LARGE={large}',
                str(source / 'tests/test_sequencer.cpp'), str(source / 'Sequencer.cpp'),
                str(source / 'SequencerDocument.cpp'), '-o', str(binary)
            ], check=True)
            subprocess.run([str(binary)], check=True)
            print(f'PASS: sequencer {"large" if large else "compact"} profile', flush=True)


if __name__ == '__main__':
    main()
