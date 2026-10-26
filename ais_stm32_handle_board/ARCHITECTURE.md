# Handle firmware modules

This branch is based on 6fc8c7c0807f plus the two minimal fixes (CAN receive flags
and one-shot torque requests), not the diagnostic/recovery firmware from PR17.

| Module | Responsibility |
|---|---|
| `.ino` | Board wiring and setup, RTOS tasks and timing, sensor acquisition, routing to services |
| `can_transport` | Serialized MCP2515 frame I/O and RX flag handling |
| `handle_protocol` | Handle command IDs, command dispatch, telemetry encoding |
| `handle_servo` | Latest torque/position request, Dynamixel startup and periodic UART operations |
| `../libraries/FirmwareService` | Reusable application version and boot-entry protocol |
| `bootloader/` | Independent flash update program, validation and application handoff |

The CAN task posts requests to HandleServo; the 20 ms task consumes torque once
before UART I/O. A later request remains pending. The existing 500 us UART gap,
sensor/CAN gaps, task priorities and all four task stack allocations are retained.
No CRC read rejection, recovery retries, automatic reset, or diagnostic stream is
added. Command payload validation for ordinary handle commands is unchanged.

Common firmware service acceptance/send/entry callbacks are supplied by the
board. New behavior at that boundary: a board can reject boot entry, and entry
will not occur when the ACK could not be queued. Version validation is also
bounded for callers other than the handle. Valid handle wire responses are unchanged.

## Size validation

Use the same toolchain, flags and explicit version text to compare before/after.
`build.sh` checks the actual binary against the 55,296-byte app partition;
`bootloader/build.sh` checks its separate 8,192-byte partition. The linker checks
static RAM placement plus its heap/stack reservations. Arduino's reported free
RAM is shared by task stacks, heap allocations and runtime stack, not spare RAM
available to spend independently. Runtime headroom still needs device measurement.

No new task, dynamic allocation, or task-stack increase is introduced here.

Measured with STM32 Core 2.7.1, `-Os -flto`, and identical version text
`size-baseline` on both builds:

| Resource | Before (eeacefd) | After | Limit |
|---|---:|---:|---:|
| Application binary | 50,752 B | 51,236 B | 55,296 B |
| Static RAM (Arduino size report) | 4,168 B | 4,224 B | 12,288 B total SRAM |
| Bootloader binary | 3,276 B | 3,276 B | 8,192 B |

Application Flash headroom is 4,060 B. Static RAM grows by 56 B. The remaining
8,064 B of SRAM is shared runtime space, not measured free heap. The linker still
reserves 1,024 B for the main stack and 512 B minimum heap; all four task stack
requests remain 128 words (512 B) each. These build checks do not establish
runtime heap/stack headroom. The firmware at `8e07d60464f8` was subsequently
flashed to AIS-K25-14: three CAN updates and 34 OFF/ON position trials passed.
See [hardware validation](tests/hardware/README.md). Runtime memory headroom
and the historical UART CRC cause remain unverified.

Validation: three native C++ executables passed; Python suite 15 passed and one
vcan integration skipped; combined STM32 application/bootloader build passed.
