#!/usr/bin/env python3
"""Update the handle firmware through its classic-CAN bootloader."""

import argparse
import socket
import struct
import sys
import time
import zlib
from pathlib import Path

try:
    from .can_bootloader_protocol import (
        APP_ADDRESS,
        APP_MAX_SIZE,
        CAN_ID_CONTROL,
        CAN_ID_DATA,
        CAN_ID_RESPONSE,
        DATA_BYTES_PER_FRAME,
        DATA_WINDOW_FRAMES,
        FLASH_PAGE_SIZE,
        OP_ABORT,
        OP_APP_VERSION,
        OP_BEGIN_CRC,
        OP_BEGIN_INFO,
        OP_DATA,
        OP_ENTER,
        OP_FINISH,
        format_app_version,
        OP_PAGE_BEGIN,
        OP_PAGE_COMMIT,
        OP_QUERY,
        PROTOCOL_VERSION,
        SRAM_END,
        SRAM_START,
        STATUS_NAMES,
        STATUS_OK,
        pack_socketcan_frame,
        pad_frame,
        parse_response,
        unpack_socketcan_frame,
    )
except ImportError:
    from can_bootloader_protocol import (  # type: ignore
        APP_ADDRESS,
        APP_MAX_SIZE,
        CAN_ID_CONTROL,
        CAN_ID_DATA,
        CAN_ID_RESPONSE,
        DATA_BYTES_PER_FRAME,
        DATA_WINDOW_FRAMES,
        FLASH_PAGE_SIZE,
        OP_ABORT,
        OP_APP_VERSION,
        OP_BEGIN_CRC,
        OP_BEGIN_INFO,
        OP_DATA,
        OP_ENTER,
        OP_FINISH,
        format_app_version,
        OP_PAGE_BEGIN,
        OP_PAGE_COMMIT,
        OP_QUERY,
        PROTOCOL_VERSION,
        SRAM_END,
        SRAM_START,
        STATUS_NAMES,
        STATUS_OK,
        pack_socketcan_frame,
        pad_frame,
        parse_response,
        unpack_socketcan_frame,
    )


class UpdateError(RuntimeError):
    pass


class SocketCanTransport:
    def __init__(self, interface: str):
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_filter = struct.pack("=II", CAN_ID_RESPONSE, 0x7FF)
        self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, can_filter)
        self.socket.bind((interface,))

    def close(self):
        self.socket.close()

    def send(self, can_id: int, data: bytes):
        self.socket.send(pack_socketcan_frame(can_id, data))

    def receive(self, timeout: float):
        self.socket.settimeout(timeout)
        try:
            return unpack_socketcan_frame(self.socket.recv(16))
        except socket.timeout:
            return None


def validate_image(image: bytes):
    if len(image) < 8:
        raise UpdateError("image is too short to contain a vector table")
    if len(image) > APP_MAX_SIZE:
        raise UpdateError(
            f"image is {len(image)} bytes; maximum is {APP_MAX_SIZE} bytes"
        )
    stack, reset = struct.unpack_from("<II", image)
    reset_address = reset & ~1
    if not SRAM_START <= stack <= SRAM_END:
        raise UpdateError(f"invalid initial stack pointer: 0x{stack:08x}")
    if not (reset & 1) or not APP_ADDRESS <= reset_address < APP_ADDRESS + len(image):
        raise UpdateError(f"invalid reset handler: 0x{reset:08x}")
    return zlib.crc32(image) & 0xFFFFFFFF


class BootloaderClient:
    def __init__(self, transport, response_timeout=0.5, retries=3, progress=None):
        self.transport = transport
        self.response_timeout = response_timeout
        self.retries = retries
        self.progress = progress or (lambda _done, _total: None)

    def _wait_response(self, operation: int, timeout=None):
        deadline = time.monotonic() + (self.response_timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            received = self.transport.receive(max(0.0, deadline - time.monotonic()))
            if received is None:
                break
            can_id, data = received
            if can_id != CAN_ID_RESPONSE:
                continue
            response = parse_response(data)
            if response.operation == operation:
                return response
        return None

    def command(self, operation: int, payload: bytes = b"", check=True):
        frame = pad_frame(bytes([operation]) + payload)
        for _attempt in range(self.retries):
            self.transport.send(CAN_ID_CONTROL, frame)
            response = self._wait_response(operation)
            if response is None:
                continue
            if check and response.status != STATUS_OK:
                name = STATUS_NAMES.get(response.status, f"status {response.status}")
                raise UpdateError(
                    f"command 0x{operation:02x} failed: {name} "
                    f"(page={response.page}, offset={response.expected_offset}, "
                    f"detail={response.detail})"
                )
            return response
        raise UpdateError(f"no response to command 0x{operation:02x}")

    def enter_bootloader(self):
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.transport.send(CAN_ID_CONTROL, b"\x01CBL1\0\0\0")
            if self._wait_response(OP_ENTER, 0.1) is not None:
                break
        else:
            raise UpdateError("no response to bootloader entry request")

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                return self.command(OP_QUERY)
            except UpdateError:
                time.sleep(0.05)
        raise UpdateError("application reset, but bootloader did not answer QUERY")

    def query_application_version(self):
        return self.command(OP_APP_VERSION).detail

    def _send_page(self, page_index: int, page: bytes):
        page_crc = zlib.crc32(page) & 0xFFFFFFFF
        payload = struct.pack("<BHI", page_index, len(page), page_crc)
        self.command(OP_PAGE_BEGIN, payload)

        offset = 0
        while offset < len(page):
            frames = 0
            while offset < len(page) and frames < DATA_WINDOW_FRAMES:
                chunk = page[offset : offset + DATA_BYTES_PER_FRAME]
                self.transport.send(
                    CAN_ID_DATA,
                    struct.pack("<H", offset) + chunk.ljust(DATA_BYTES_PER_FRAME, b"\0"),
                )
                offset += len(chunk)
                frames += 1
            response = self.command(OP_QUERY)
            if response.status != STATUS_OK:
                raise UpdateError("QUERY failed while sending page")
            if response.expected_offset > len(page):
                raise UpdateError("bootloader reported an invalid page offset")
            offset = response.expected_offset

        self.command(OP_PAGE_COMMIT, bytes([page_index]))

    def update(self, image: bytes):
        image_crc = validate_image(image)
        self.enter_bootloader()
        self.command(OP_BEGIN_INFO, struct.pack("<BI", PROTOCOL_VERSION, len(image)))
        self.command(OP_BEGIN_CRC, struct.pack("<I", image_crc))

        total_pages = (len(image) + FLASH_PAGE_SIZE - 1) // FLASH_PAGE_SIZE
        for page_index in range(total_pages):
            start = page_index * FLASH_PAGE_SIZE
            page = image[start : start + FLASH_PAGE_SIZE]
            self._send_page(page_index, page)
            self.progress(min(start + len(page), len(image)), len(image))
        self.command(OP_FINISH)
        return image_crc

    def abort(self):
        return self.command(OP_ABORT)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, nargs="?", help="relocated raw application .bin")
    parser.add_argument("-i", "--interface", default="can0", help="SocketCAN interface")
    parser.add_argument("--dry-run", action="store_true", help="validate without opening CAN")
    parser.add_argument(
        "--query-app-version",
        action="store_true",
        help="print the running application version and exit",
    )
    parser.add_argument("--quiet", action="store_true", help="suppress page progress")
    args = parser.parse_args(argv)

    try:
        if args.query_app_version:
            if args.image is not None:
                parser.error("image cannot be used with --query-app-version")
            if args.dry_run:
                parser.error("--dry-run cannot be used with --query-app-version")
            transport = SocketCanTransport(args.interface)
            try:
                version = BootloaderClient(transport).query_application_version()
            finally:
                transport.close()
            print(f"application version: {format_app_version(version)} (0x{version:04x})")
            return 0

        if args.image is None:
            parser.error("image is required unless --query-app-version is used")

        image = args.image.read_bytes()
        image_crc = validate_image(image)
        pages = (len(image) + FLASH_PAGE_SIZE - 1) // FLASH_PAGE_SIZE
        print(
            f"image: {args.image}\nsize: {len(image)} / {APP_MAX_SIZE} bytes\n"
            f"pages: {pages}\ncrc32: 0x{image_crc:08x}"
        )
        if args.dry_run:
            return 0

        transport = SocketCanTransport(args.interface)
        try:
            def progress(done, total):
                if not args.quiet:
                    print(f"updated {done}/{total} bytes", flush=True)

            BootloaderClient(transport, progress=progress).update(image)
        finally:
            transport.close()
        print("update committed; handle is restarting")
        return 0
    except (OSError, UpdateError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
