"""Unread torque-ON must survive a target frame arriving during CAN reception."""
import unittest

from firmware_test_support import read_firmware, run_firmware_test


class ReceiveTests(unittest.TestCase):
    def test_off_on_and_target_arrival_does_not_lose_on(self):
        source = read_firmware()
        task_start = source.index("void task_read(")
        task_end = source.index("void process_message(", task_start)
        receive_task = source[task_start:task_end]

        # Fixture starts OFF, queues ON, then injects a target during the read.
        # It checks that ON is retained and the target can take effect.
        run_firmware_test(self, "can_receive.cpp", receive_task)
