import os
import struct
import sys
import threading
import unittest
from collections import deque
from pathlib import Path

HANDLE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HANDLE_DIR))

from tools.can_bootloader_protocol import (  # noqa: E402
    APP_ADDRESS,
    APP_MAX_SIZE,
    CAN_ID_CONTROL,
    CAN_ID_DATA,
    CAN_ID_RESPONSE,
    OP_ABORT,
    OP_APP_VERSION,
    OP_BEGIN_CRC,
    OP_FINISH,
    OP_PAGE_BEGIN,
    OP_PAGE_COMMIT,
    OP_QUERY,
    STATE_WAIT,
    STATE_VALID_APP,
    format_app_version,
    make_response,
    pack_socketcan_frame,
    unpack_socketcan_frame,
    parse_response,
)
from tools.can_bootloader_sim import BootloaderModel, SocketCanSimulator  # noqa: E402
from tools.can_update import (  # noqa: E402
    BootloaderClient,
    SocketCanTransport,
    UpdateError,
    validate_image,
)


def make_image(size=5000):
    image = bytearray((index * 37 + 11) & 0xFF for index in range(size))
    struct.pack_into("<II", image, 0, 0x20003000, APP_ADDRESS + 0x101)
    return bytes(image)


class ModelTransport:
    def __init__(self, model):
        self.model = model
        self.responses = deque()

    def send(self, can_id, data):
        for response in self.model.process(can_id, data):
            self.responses.append(response)

    def receive(self, _timeout):
        if self.responses:
            return self.responses.popleft()
        return None


class CorruptDataTransport(ModelTransport):
    def __init__(self, model):
        super().__init__(model)
        self.corrupted = False

    def send(self, can_id, data):
        if can_id == CAN_ID_DATA and not self.corrupted:
            data = bytearray(data)
            data[2] ^= 0x80
            data = bytes(data)
            self.corrupted = True
        super().send(can_id, data)


class DropResponseTransport(ModelTransport):
    def __init__(self, model, operations):
        super().__init__(model)
        self.operations = set(operations)
        self.dropped = set()

    def send(self, can_id, data):
        responses = self.model.process(can_id, data)
        for response in responses:
            response_id, payload = response
            operation = parse_response(payload).operation
            if operation in self.operations and operation not in self.dropped:
                self.dropped.add(operation)
                continue
            self.responses.append((response_id, payload))


class ApplicationVersionTransport:
    def __init__(self, version):
        self.version = version
        self.responses = deque()

    def send(self, can_id, data):
        if can_id == CAN_ID_CONTROL and data[0] == OP_APP_VERSION:
            self.responses.append(
                (
                    CAN_ID_RESPONSE,
                    make_response(OP_APP_VERSION, 0, 0, STATE_VALID_APP, detail=self.version),
                )
            )

    def receive(self, _timeout):
        if self.responses:
            return self.responses.popleft()
        return None


class ProtocolTests(unittest.TestCase):
    def test_socketcan_frame_round_trip(self):
        packed = pack_socketcan_frame(0x7A1, b"12345678")
        self.assertEqual(len(packed), 16)
        self.assertEqual(unpack_socketcan_frame(packed), (0x7A1, b"12345678"))

    def test_image_validation(self):
        image = make_image()
        self.assertIsInstance(validate_image(image), int)

        with self.assertRaisesRegex(UpdateError, "maximum"):
            validate_image(make_image(APP_MAX_SIZE + 1))

        invalid = bytearray(image)
        struct.pack_into("<I", invalid, 0, 0x1000)
        with self.assertRaisesRegex(UpdateError, "stack"):
            validate_image(bytes(invalid))

    def test_query_application_version(self):
        version = BootloaderClient(ApplicationVersionTransport(0x0102)).query_application_version()
        self.assertEqual(version, 0x0102)
        self.assertEqual(format_app_version(version), "1.2")

    def test_successful_multi_page_update(self):
        image = make_image()
        model = BootloaderModel()
        progress = []
        client = BootloaderClient(
            ModelTransport(model), progress=lambda done, total: progress.append((done, total))
        )
        client.update(image)
        self.assertTrue(model.metadata_valid)
        self.assertEqual(model.memory[:len(image)], image)
        self.assertEqual(progress[-1], (len(image), len(image)))

    def test_missing_data_frame_resumes_from_reported_offset(self):
        image = make_image(3000)
        model = BootloaderModel(drop_data_offset_once=30)
        BootloaderClient(ModelTransport(model)).update(image)
        self.assertTrue(model.metadata_valid)
        self.assertTrue(model._dropped)

    def test_control_commands_are_idempotent_after_lost_ack(self):
        image = make_image(3000)
        model = BootloaderModel()
        transport = DropResponseTransport(
            model, {OP_BEGIN_CRC, OP_PAGE_BEGIN, OP_PAGE_COMMIT, OP_FINISH}
        )
        BootloaderClient(transport).update(image)
        self.assertTrue(model.metadata_valid)
        self.assertEqual(
            transport.dropped,
            {OP_BEGIN_CRC, OP_PAGE_BEGIN, OP_PAGE_COMMIT, OP_FINISH},
        )

    def test_page_crc_failure_does_not_commit(self):
        model = BootloaderModel()
        with self.assertRaisesRegex(UpdateError, "page CRC"):
            BootloaderClient(CorruptDataTransport(model)).update(make_image(1000))
        self.assertFalse(model.metadata_valid)

    def test_flash_failure_does_not_commit(self):
        model = BootloaderModel(flash_fail_page=0)
        with self.assertRaisesRegex(UpdateError, "flash"):
            BootloaderClient(ModelTransport(model)).update(make_image(1000))
        self.assertFalse(model.metadata_valid)

    def test_full_image_crc_failure_does_not_commit(self):
        model = BootloaderModel(corrupt_image_before_finish=True)
        with self.assertRaisesRegex(UpdateError, "image CRC"):
            BootloaderClient(ModelTransport(model)).update(make_image(1000))
        self.assertFalse(model.metadata_valid)

    def test_timeout_and_abort_leave_waiting_with_invalid_metadata(self):
        model = BootloaderModel()
        client = BootloaderClient(ModelTransport(model))
        client.enter_bootloader()
        client.command(0x02, struct.pack("<BI", 1, 1000))
        client.command(0x03, struct.pack("<I", 0))
        model.expire_update()
        self.assertEqual(model.state, STATE_WAIT)
        self.assertFalse(model.metadata_valid)
        response = client.command(OP_QUERY)
        self.assertEqual(response.state, STATE_WAIT)
        client.command(OP_ABORT)
        image = make_image(1000)
        client.update(image)
        self.assertTrue(model.metadata_valid)
        self.assertEqual(model.memory[:len(image)], image)


@unittest.skipUnless(os.path.exists("/sys/class/net/vcan0"), "vcan0 is not configured")
class VcanIntegrationTests(unittest.TestCase):
    def test_updater_against_socketcan_simulator(self):
        model = BootloaderModel()
        simulator = SocketCanSimulator("vcan0", model)
        stop = threading.Event()
        thread = threading.Thread(target=simulator.run, args=(stop,), daemon=True)
        thread.start()
        transport = SocketCanTransport("vcan0")
        try:
            BootloaderClient(transport).update(make_image(1000))
            self.assertTrue(model.metadata_valid)
        finally:
            transport.close()
            stop.set()
            thread.join(timeout=1)
            simulator.close()


if __name__ == "__main__":
    unittest.main()
