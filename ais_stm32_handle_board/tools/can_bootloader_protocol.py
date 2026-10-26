"""Wire constants and helpers shared by the updater and simulator."""

import struct
from dataclasses import dataclass

CAN_ID_CONTROL = 0x7A0
CAN_ID_RESPONSE = 0x7A1
CAN_ID_DATA = 0x7A2

OP_ENTER = 0x01
OP_BEGIN_INFO = 0x02
OP_BEGIN_CRC = 0x03
OP_QUERY = 0x04
OP_PAGE_BEGIN = 0x10
OP_PAGE_COMMIT = 0x11
OP_DATA = 0x12
OP_FINISH = 0x20
OP_ABORT = 0x21

STATUS_OK = 0
STATUS_BAD_STATE = 1
STATUS_BAD_ARGUMENT = 2
STATUS_OFFSET = 3
STATUS_PAGE_CRC = 4
STATUS_FLASH = 5
STATUS_IMAGE_CRC = 6
STATUS_BAD_VECTOR = 7
STATUS_TIMEOUT = 8
STATUS_UNSUPPORTED = 9

STATE_WAIT = 0
STATE_UPDATE = 1
STATE_PAGE = 2
STATE_READY = 3
STATE_VALID_APP = 4

STATUS_NAMES = {
    STATUS_OK: "ok",
    STATUS_BAD_STATE: "bad state",
    STATUS_BAD_ARGUMENT: "bad argument",
    STATUS_OFFSET: "unexpected data offset",
    STATUS_PAGE_CRC: "page CRC mismatch",
    STATUS_FLASH: "flash operation failed",
    STATUS_IMAGE_CRC: "image CRC mismatch",
    STATUS_BAD_VECTOR: "invalid vector table",
    STATUS_TIMEOUT: "update timeout",
    STATUS_UNSUPPORTED: "unsupported command",
}

PROTOCOL_VERSION = 1
APP_ADDRESS = 0x08002000
APP_MAX_SIZE = 0xD800
FLASH_PAGE_SIZE = 2048
DATA_BYTES_PER_FRAME = 6
DATA_WINDOW_FRAMES = 16
SRAM_START = 0x20000000
SRAM_END = 0x20003000

CAN_FRAME_STRUCT = struct.Struct("=IB3x8s")


@dataclass(frozen=True)
class Response:
    operation: int
    status: int
    page: int
    state: int
    expected_offset: int
    detail: int


def pad_frame(data: bytes) -> bytes:
    if len(data) > 8:
        raise ValueError("classic CAN payload exceeds 8 bytes")
    return data.ljust(8, b"\0")


def pack_socketcan_frame(can_id: int, data: bytes) -> bytes:
    data = pad_frame(data)
    return CAN_FRAME_STRUCT.pack(can_id, 8, data)


def unpack_socketcan_frame(frame: bytes):
    can_id, dlc, data = CAN_FRAME_STRUCT.unpack(frame)
    return can_id & 0x7FF, data[:dlc]


def parse_response(data: bytes) -> Response:
    if len(data) != 8:
        raise ValueError("bootloader response must have DLC 8")
    operation, status, page, state, expected, detail = struct.unpack("<BBBBHH", data)
    return Response(operation & 0x7F, status, page, state, expected, detail)


def make_response(operation: int, status: int, page: int, state: int,
                  expected_offset: int = 0, detail: int = 0) -> bytes:
    return struct.pack(
        "<BBBBHH",
        operation | 0x80,
        status,
        page,
        state,
        expected_offset,
        detail,
    )
