"""Finite RX-buffer regression; virtual time avoids flaky wall-clock assertions."""
import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.can_bootloader_protocol import CAN_ID_DATA, OP_QUERY
from tools.can_bootloader_sim import BootloaderModel
from tools.can_update import BootloaderClient, UpdateError
from test_can_bootloader import ModelTransport, make_image


class BufferedTransport(ModelTransport):
    # Two receive buffers, a slower MCU draining one frame per 0.5 ms.
    def __init__(self):
        super().__init__(BootloaderModel())
        self.now=0.; self.queue=[]; self.dropped=0; self.data_sent=0

    def sleep(self, seconds):
        self.now+=seconds
        self.drain()

    def drain(self):
        while self.queue and self.queue[0][0] <= self.now:
            _, cid, data=self.queue.pop(0)
            super().send(cid,data)

    def send(self,cid,data):
        self.drain()
        if cid==CAN_ID_DATA:
            self.data_sent+=1
            if len(self.queue)==2:
                self.dropped+=1
                return
            due=max(self.now,self.queue[-1][0] if self.queue else self.now)+.0005
            self.queue.append((due,cid,data))
        else:
            # A query waits for buffered frames in this model. The regression
            # therefore isolates RX capacity rather than CAN arbitration.
            self.sleep(.001)
            super().send(cid,data)


class PacingTests(unittest.TestCase):
    def test_full_image_on_two_buffer_receiver_without_overrun(self):
        transport=BufferedTransport(); image=make_image(50744)
        with patch('tools.can_update.time.sleep',side_effect=transport.sleep):
            BootloaderClient(transport).update(image)
        self.assertTrue(transport.model.metadata_valid)
        self.assertEqual(transport.model.memory[:len(image)],image)
        self.assertEqual(transport.dropped,0,'unpaced bursts overrun the two RX buffers')
        self.assertLess(transport.now,15,'virtual transfer budget exceeded')

    def test_permanent_data_loss_fails_instead_of_looping_forever(self):
        class BlackholeTransport(ModelTransport):
            sent = 0
            def send(self, cid, data):
                if cid == CAN_ID_DATA:
                    self.sent += 1
                    if self.sent > 48:
                        raise AssertionError("updater did not stop after three stalled windows")
                    return
                super().send(cid, data)
        transport = BlackholeTransport(BootloaderModel())
        with patch('tools.can_update.time.sleep'), self.assertRaisesRegex(UpdateError, "stalled"):
            BootloaderClient(transport, retries=3).update(make_image(500))
        self.assertFalse(transport.model.metadata_valid)
