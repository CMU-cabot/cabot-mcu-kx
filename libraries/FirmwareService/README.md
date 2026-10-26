# FirmwareService

Application-side firmware management shared by board sketches. It implements
version query and bootloader entry messages, not flash writes. There is no
Arduino, RTOS, MCP2515, heap allocation, or board register dependency.

A board supplies `FirmwareService::Config`:

- Request/response CAN IDs. Allocate nonconflicting IDs before using multiple
  update-capable boards on one CAN bus; the handle retains 0x7A0/0x7A1.
- A static 1–64-byte printable version string.
- Context pointer and three callbacks: send, permission to enter update mode,
  and the board-specific transition to its bootloader.

Route incoming frames to `handle(id, length, data)` before ordinary application
commands. It returns false for other IDs. Sending an entry ACK must succeed
before the transition callback runs. The board callback is responsible for the
ACK drain interval and any platform-specific reset/backup marker handling.

The handle preserves its 2 ms ACK interval and STM32 BKP0 marker. Power and sensor
boards are not yet migrated: each needs an appropriate update acceptance policy,
power/output handling, bootloader, and flash layout. A power board must not simply
copy the handle's unconditional permission callback.

Arduino sketches include `<FirmwareService.h>` and build with
`--libraries ../libraries`. `FirmwareProtocol.h` contains the wire definitions
also used by the existing handle bootloader. Its currently defined default IDs
are handle allocations, not automatic allocations for other boards.
