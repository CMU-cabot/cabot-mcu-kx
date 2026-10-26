"""Compile actual firmware fragments with readable C++ hardware substitutes."""
import os
from pathlib import Path
import subprocess
import tempfile

FIRMWARE_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = Path(__file__).resolve().parent / "fixtures"


def read_firmware():
    # Point at the old .ino to demonstrate that a regression test fails there.
    path = Path(os.environ.get(
        "HANDLE_TEST_SOURCE", FIRMWARE_ROOT / "ais_stm32_handle_board.ino"
    ))
    return path.read_text()


def run_firmware_test(test_case, fixture_name, firmware_fragment):
    template = (FIXTURES / fixture_name).read_text()
    marker = "// FIRMWARE_UNDER_TEST"
    test_case.assertEqual(template.count(marker), 1)
    source = template.replace(marker, firmware_fragment)

    with tempfile.TemporaryDirectory() as directory:
        source_path = Path(directory) / "test.cpp"
        executable = Path(directory) / "test"
        source_path.write_text(source)
        compile_result = subprocess.run(
            ["g++", "-std=c++11", str(source_path), "-o", str(executable)],
            capture_output=True, text=True,
        )
        test_case.assertEqual(compile_result.returncode, 0, compile_result.stderr)
        result = subprocess.run([str(executable)], capture_output=True, text=True)
        test_case.assertEqual(result.returncode, 0, result.stderr)
