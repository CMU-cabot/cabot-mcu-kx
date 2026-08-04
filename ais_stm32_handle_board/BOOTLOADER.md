# Handle board CAN bootloader

The STM32F303K8 handle board uses an external MCP2515 to receive application updates
over 1 Mbps classic CAN. The bootloader is a CMSIS-only bare-metal image; it does not
link Arduino, STM32 HAL, or FreeRTOS.

## Flash layout

| Region | Address range | Pages | Size |
| --- | --- | ---: | ---: |
| CAN bootloader | `0x08000000` - `0x08001FFF` | 0-3 | 8 KiB |
| Handle application | `0x08002000` - `0x0800F7FF` | 4-30 | 54 KiB |
| Image metadata | `0x0800F800` - `0x0800FFFF` | 31 | 2 KiB |

STM32F303K8 flash erase pages are 2 KiB. A bootloader must never erase pages 0-3
while updating the application.

The application binary is raw data whose first byte is programmed at
`0x08002000`. Its maximum length is 55,296 bytes. Do not prepend a header: the first
two words are the application initial stack pointer and Thumb reset handler.

Page 31 stores a format version, image length, image CRC32, and metadata CRC32. It is
erased before an update and its `CBL1` magic is programmed last. Therefore a reset or
power loss cannot mark a partial application as bootable. CRC values use the IEEE
CRC32 representation returned by Python `zlib.crc32`.

## Build

From the STM32 handle container:

```sh
./build.sh firmware
```

The relevant outputs are:

```text
build/bootloader/handle_bootloader.bin
build/can-application/ais_stm32_handle_board.ino.bin
```

`./build.sh` builds only the relocated application and `./build.sh bootloader` builds
only the bootloader. Every build checks its flash boundary and vector location. The
application build uses `build.flash_offset=0x2000`, `VECT_TAB_OFFSET=0x2000`, `-Os`,
and LTO. `./build.sh -s` remains available for the legacy standalone application and
is not a CAN update payload.

Validate, query the running application version, or install an application with the
standard-library-only SocketCAN tool:

```sh
python3 tools/can_update.py --dry-run \
  build/can-application/ais_stm32_handle_board.ino.bin
python3 tools/can_update.py --interface can0 --query-app-version
python3 tools/can_update.py --interface can0 \
  build/can-application/ais_stm32_handle_board.ino.bin
```

## Boot and handoff

At reset, the bootloader validates metadata, both vector words, and the complete image
CRC. An invalid image remains in update mode. A valid image listens for update entry
for 500 ms and then starts the application.

The running application also accepts the guarded `ENTER` command. It writes `CBL1` to
RTC backup register 0 and performs a system reset. The bootloader consumes and clears
that value, allowing it to wait indefinitely without relying only on the 500 ms
window.

Before handoff, the bootloader resets the MCP2515, disables SPI and interrupts, clears
NVIC pending state, sets `SCB->VTOR` and MSP from the application vector, and branches
to its reset handler.

## CAN protocol

All frames are standard 11-bit, DLC 8 frames at 1 Mbps:

| CAN ID | Direction | Purpose |
| --- | --- | --- |
| `0x7A0` | host to handle | control commands |
| `0x7A1` | handle to host | ACK, NACK, state, and expected offset |
| `0x7A2` | host to handle | page data: 16-bit offset plus 6 bytes |

Control opcodes are `ENTER`, `QUERY`, `APP_VERSION`, `BEGIN_INFO`, `BEGIN_CRC`,
`PAGE_BEGIN`, `PAGE_COMMIT`, `FINISH`, and `ABORT`. The host sends data in windows of
16 frames and uses `QUERY` to obtain the next expected offset. Lost or out-of-order
frames resume at that offset. Control commands that can lose their ACK are idempotent.

Multi-byte integers are little-endian. The eight control bytes are:

| Opcode | Bytes 1-7 |
| --- | --- |
| `0x01 ENTER` | ASCII `CBL1`, then three zero bytes |
| `0x05 APP_VERSION` | seven zero bytes |
| `0x02 BEGIN_INFO` | protocol version, 32-bit image length, two zero bytes |
| `0x03 BEGIN_CRC` | 32-bit image CRC, then three zero bytes |
| `0x04 QUERY` | seven zero bytes |
| `0x10 PAGE_BEGIN` | page index, 16-bit length, 32-bit page CRC |
| `0x11 PAGE_COMMIT` | page index, then six zero bytes |
| `0x20 FINISH` / `0x21 ABORT` | seven zero bytes |

A response contains opcode ORed with `0x80`, status, page, state, 16-bit expected
offset, and 16-bit detail. Status zero is success; the remaining values are defined in
`bootloader/protocol.h`. A data frame contains a 16-bit page offset followed by six
bytes, with unused bytes in the final frame set to zero.

The running application answers `APP_VERSION` on `0x7A1` with status `OK`, state
`VALID_APP`, and a packed `major.minor` version in the 16-bit detail field
(`detail = major << 8 | minor`). The bootloader itself does not implement this opcode.

One 2 KiB page is buffered in SRAM. The bootloader validates its CRC before erasing
and programming the corresponding application page, then verifies flash by reading it
back. `FINISH` recomputes the whole-image CRC and validates the vector before committing
metadata. An active transfer times out after five seconds without traffic and remains
in the bootloader with invalid metadata.

This is a single-slot, CRC-only design. It does not provide image signatures,
encryption, anti-rollback, a second image, or automatic rollback.

## Tests

Run host tests with:

```sh
python3 -m unittest discover -s tests -v
```

The suite covers image checks, frame packing, multi-page updates, missing data,
lost ACKs, page and image CRC failures, flash failures, timeout, and abort. If `vcan0`
already exists, it also runs the updater against the SocketCAN simulator. The
simulator can be started manually with:

```sh
python3 tools/can_bootloader_sim.py --interface vcan0
```

No physical board is available for this work. MCP2515 electrical timing, STM32 flash
programming, reset persistence, and application handoff still require later on-device
verification. Initial device programming is intentionally outside this implementation.
