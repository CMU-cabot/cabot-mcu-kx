"""Regression tests exercising the actual firmware block and build entry point."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BuildFailureTests(unittest.TestCase):
    def test_application_failure_stops_firmware_target(self):
        # Firmware build must fail before even starting a successful bootloader build.
        with tempfile.TemporaryDirectory() as tmp:
            test_directory = Path(tmp)
            shutil.copy(ROOT / 'build.sh', test_directory / 'build.sh')
            (test_directory / 'tools').mkdir()
            (test_directory / 'tools/generate_version.py').write_text('pass\n')
            (test_directory / 'bootloader').mkdir()
            bootloader_stub = test_directory / 'bootloader/build.sh'
            bootloader_stub.write_text('#!/bin/bash\ntouch bootloader-started\nexit 0\n')
            bootloader_stub.chmod(0o755)
            compiler = test_directory / 'arduino-cli'
            compiler.write_text('#!/bin/bash\nexit 23\n')
            compiler.chmod(0o755)
            result = subprocess.run(['bash', 'build.sh', 'firmware'], cwd=test_directory,
                                    env={**os.environ, 'PATH': tmp + ':' + os.environ['PATH']},
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((test_directory / 'bootloader-started').exists())
