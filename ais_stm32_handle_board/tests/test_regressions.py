"""Regression tests exercising the actual firmware block and build entry point."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from firmware_test_support import FIRMWARE_ROOT, read_firmware, run_firmware_test

ROOT = FIRMWARE_ROOT


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


class ServoCommandTests(unittest.TestCase):
    def test_consumes_once_and_preserves_new_command_during_io(self):
        source = read_firmware()
        task_start = source.index("void task20ms(")
        block_start = source.index("    if(servo_enable)", task_start)
        block_end = source.index("    vTaskDelayUntil", block_start)
        servo_block = source[block_start:block_end]

        # The fixture checks one-shot consumption, then OFF arriving during ON.
        run_firmware_test(self, "servo_command.cpp", servo_block)
