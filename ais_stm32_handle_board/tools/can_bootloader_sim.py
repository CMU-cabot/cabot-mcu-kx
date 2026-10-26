#!/usr/bin/env python3
"""SocketCAN simulator for the handle bootloader update protocol."""

import argparse
import select
import socket
import struct
import sys
import time
import zlib

try:
    from .can_bootloader_protocol import *  # noqa: F403
except ImportError:
    from can_bootloader_protocol import *  # type: ignore # noqa: F403


class BootloaderModel:
    def __init__(self, drop_data_offset_once=None, flash_fail_page=None,
                 corrupt_image_before_finish=False):
        self.drop_data_offset_once = drop_data_offset_once
        self.flash_fail_page = flash_fail_page
        self.corrupt_image_before_finish = corrupt_image_before_finish
        self._dropped = False
        self.memory = bytearray(b"\xff" * APP_MAX_SIZE)  # noqa: F405
        self.metadata_valid = False
        self.pending_size = None
        self.image_size = 0
        self.image_crc = 0
        self.state = STATE_WAIT  # noqa: F405
        self.next_page = 0
        self.page_index = 0
        self.page_length = 0
        self.page_crc = 0
        self.expected_offset = 0
        self.page_buffer = bytearray()
        self.last_activity = time.monotonic()

    def response(self, operation, status=STATUS_OK, detail=0):  # noqa: F405
        page = self.page_index if self.state == STATE_PAGE else self.next_page  # noqa: F405
        return CAN_ID_RESPONSE, make_response(  # noqa: F405
            operation, status, page, self.state, self.expected_offset, detail
        )

    def expire_update(self):
        if self.state in (STATE_UPDATE, STATE_PAGE, STATE_READY):  # noqa: F405
            self.state = STATE_WAIT  # noqa: F405
            self.next_page = 0
            self.expected_offset = 0

    def process(self, can_id, data):
        self.last_activity = time.monotonic()
        if can_id == CAN_ID_DATA:  # noqa: F405
            return self._data(data)
        if can_id != CAN_ID_CONTROL or len(data) != 8:  # noqa: F405
            return []
        op = data[0]
        if op == OP_ENTER:  # noqa: F405
            if data[1:5] != b"CBL1":
                return [self.response(op, STATUS_BAD_ARGUMENT)]  # noqa: F405
            return [self.response(op, detail=1)]
        if op == OP_QUERY:  # noqa: F405
            return [self.response(op, detail=1)]
        if op == OP_BEGIN_INFO:  # noqa: F405
            version = data[1]
            size = struct.unpack_from("<I", data, 2)[0]
            if version != PROTOCOL_VERSION or not 8 <= size <= APP_MAX_SIZE:  # noqa: F405
                return [self.response(op, STATUS_BAD_ARGUMENT)]  # noqa: F405
            self.pending_size = size
            return [self.response(op, detail=PROTOCOL_VERSION)]  # noqa: F405
        if op == OP_BEGIN_CRC:  # noqa: F405
            if self.pending_size is None:
                requested_crc = struct.unpack_from("<I", data, 1)[0]
                if (self.state in (STATE_UPDATE, STATE_PAGE, STATE_READY) and  # noqa: F405
                        requested_crc == self.image_crc):
                    return [self.response(op)]
                return [self.response(op, STATUS_BAD_STATE)]  # noqa: F405
            self.metadata_valid = False
            self.image_size = self.pending_size
            self.image_crc = struct.unpack_from("<I", data, 1)[0]
            self.pending_size = None
            self.next_page = 0
            self.expected_offset = 0
            self.state = STATE_UPDATE  # noqa: F405
            return [self.response(op)]
        if op == OP_PAGE_BEGIN:  # noqa: F405
            index, length, page_crc = struct.unpack_from("<BHI", data, 1)
            expected_length = min(
                FLASH_PAGE_SIZE, self.image_size - index * FLASH_PAGE_SIZE  # noqa: F405
            )
            new_page = self.state == STATE_UPDATE and index == self.next_page  # noqa: F405
            repeated_page = self.state == STATE_PAGE and index == self.page_index  # noqa: F405
            if (not (new_page or repeated_page) or length != expected_length):
                return [self.response(op, STATUS_BAD_STATE)]  # noqa: F405
            self.page_index = index
            self.page_length = length
            self.page_crc = page_crc
            self.page_buffer = bytearray(b"\xff" * FLASH_PAGE_SIZE)  # noqa: F405
            self.expected_offset = 0
            self.state = STATE_PAGE  # noqa: F405
            return [self.response(op)]
        if op == OP_PAGE_COMMIT:  # noqa: F405
            if (self.state in (STATE_UPDATE, STATE_READY) and self.next_page > 0 and  # noqa: F405
                    data[1] == self.next_page - 1):
                return [self.response(op)]
            if (self.state != STATE_PAGE or data[1] != self.page_index or  # noqa: F405
                    self.expected_offset != self.page_length):
                return [self.response(op, STATUS_BAD_STATE)]  # noqa: F405
            if zlib.crc32(self.page_buffer[:self.page_length]) & 0xFFFFFFFF != self.page_crc:
                return [self.response(op, STATUS_PAGE_CRC)]  # noqa: F405
            if self.flash_fail_page == self.page_index:
                return [self.response(op, STATUS_FLASH)]  # noqa: F405
            start = self.page_index * FLASH_PAGE_SIZE  # noqa: F405
            self.memory[start:start + FLASH_PAGE_SIZE] = self.page_buffer
            self.next_page += 1
            self.expected_offset = 0
            self.state = (STATE_READY if self.next_page * FLASH_PAGE_SIZE >= self.image_size  # noqa: F405
                          else STATE_UPDATE)  # noqa: F405
            return [self.response(op)]
        if op == OP_FINISH:  # noqa: F405
            if self.state == STATE_VALID_APP and self.metadata_valid:  # noqa: F405
                return [self.response(op)]
            if self.state != STATE_READY:  # noqa: F405
                return [self.response(op, STATUS_BAD_STATE)]  # noqa: F405
            if self.corrupt_image_before_finish:
                self.memory[8] ^= 1
            actual = zlib.crc32(self.memory[:self.image_size]) & 0xFFFFFFFF
            if actual != self.image_crc:
                return [self.response(op, STATUS_IMAGE_CRC)]  # noqa: F405
            stack, reset = struct.unpack_from("<II", self.memory)
            if not (SRAM_START <= stack <= SRAM_END and reset & 1 and  # noqa: F405
                    APP_ADDRESS <= (reset & ~1) < APP_ADDRESS + self.image_size):  # noqa: F405
                return [self.response(op, STATUS_BAD_VECTOR)]  # noqa: F405
            self.metadata_valid = True
            self.state = STATE_VALID_APP  # noqa: F405
            return [self.response(op)]
        if op == OP_ABORT:  # noqa: F405
            self.expire_update()
            return [self.response(op)]
        return [self.response(op, STATUS_UNSUPPORTED)]  # noqa: F405

    def _data(self, data):
        if self.state != STATE_PAGE or len(data) != 8:  # noqa: F405
            return [self.response(OP_DATA, STATUS_BAD_STATE)]  # noqa: F405
        offset = struct.unpack_from("<H", data)[0]
        if (self.drop_data_offset_once == offset and not self._dropped):
            self._dropped = True
            return []
        if offset != self.expected_offset or offset >= self.page_length:
            return [self.response(OP_DATA, STATUS_OFFSET)]  # noqa: F405
        count = min(DATA_BYTES_PER_FRAME, self.page_length - offset)  # noqa: F405
        self.page_buffer[offset:offset + count] = data[2:2 + count]
        self.expected_offset += count
        return []


class SocketCanSimulator:
    def __init__(self, interface, model):
        self.model = model
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        filters = b"".join(
            struct.pack("=II", can_id, 0x7FF)
            for can_id in (CAN_ID_CONTROL, CAN_ID_DATA)  # noqa: F405
        )
        self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        self.socket.bind((interface,))

    def run(self, stop_event=None):
        while stop_event is None or not stop_event.is_set():
            readable, _, _ = select.select([self.socket], [], [], 0.1)
            if not readable:
                if time.monotonic() - self.model.last_activity >= 5:
                    self.model.expire_update()
                continue
            can_id, data = unpack_socketcan_frame(self.socket.recv(16))  # noqa: F405
            for response_id, response in self.model.process(can_id, data):
                self.socket.send(pack_socketcan_frame(response_id, response))  # noqa: F405

    def close(self):
        self.socket.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-i", "--interface", default="vcan0")
    args = parser.parse_args(argv)
    try:
        SocketCanSimulator(args.interface, BootloaderModel()).run()
    except OSError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
