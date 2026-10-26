# Handle servo diagnostics and recovery

Individual errors do not halt the MCU or issue an automatic torque-OFF command.
Sensor/CAN tasks continue. The latest explicitly received ON/OFF request wins.
A torque change is checked using both the UART result and TORQUE_ENABLE readback.
It retries on the next task cycles up to three initial attempts, then once per
50 task cycles (nominally one second) until applied or superseded. A failed
startup ping is also retried once per second; incoming commands remain buffered.
No new automatic re-enable is triggered solely by a periodic torque mismatch.

Failed position reads are not sent as valid position samples; later valid reads
resume normal 0x09A reporting. This firmware does not change the host controller's
existing tracking/stale-feedback safety policy, or override a servo's own hardware
shutdown. Those are separate from MCU error logging/recovery.

## Diagnostic wire format

One standard 11-bit CAN frame, ID **0x7A1**, DLC 8, discriminator **0x86**.
This reuses the existing response ID, with no additional ID allocation. Version
query responses remain 0x85 and update responses are unchanged.

| Byte | Meaning |
|---|---|
| 0 | 0x86 |
| 1 | bit 0 requested torque ON; bit 1 last actual torque ON; bit 2 torque read valid; bit 3 position read valid; bit 4 torque request pending; bits 5–7 last failed operation |
| 2 | valid torque CAN commands received, modulo 256 |
| 3 | valid target CAN commands received, modulo 256 |
| 4 | last failure's Dynamixel library error |
| 5 | last failure/alert's Dynamixel status error byte |
| 6 | Hardware Error Status, or 0xFF if unknown/read failed |
| 7 | saturating failure/alert counter (255 maximum) |

Operations: 0 none, 1 torque write/readback mismatch, 2 goal write, 3 position
read, 4 torque read, 5 hardware-status read, 6 startup/configuration/ping, 7 servo reboot.
Errors stay latched until the next error or reboot; zero error codes with a
nonzero operation/counter can mean an API failure or readback mismatch.
The hardware-alert bit (0x80) is recorded but does not invalidate an otherwise
valid read payload. Other status errors do invalidate it.

Normal traffic is **one frame per 50 task cycles** (~1 Hz). Status/error changes
can report sooner, but not more than once per five cycles (~10 Hz). Receive
counter changes alone do not trigger faster transmission. Periodic UART overhead
is two reads per second (torque and hardware status); torque writes additionally
perform readback during their bounded initial retries or 1 Hz recovery.

```sh
candump can0,7A1:7FF
```

Select DLC-8 payloads beginning with 86. CAN receipt counters distinguish a
missing command from a command received by the MCU; readback, library/status
errors and hardware status help distinguish UART/application failure. The
counters do not provide a historical trace or prove mechanical movement.

## Bounded physical-device recovery

After 250 consecutive unhealthy task cycles (~5 seconds), try one Dynamixel
protocol reboot, including when hardware-error status is nonzero or unavailable.
Keep sensor/CAN tasks alive, give the servo one second without UART access, then
reapply the latest requested torque state and target. Reboot is not power cycling.
Hardware-error status is diagnostic and does not inhibit reset escalation.

If another 250 unhealthy cycles persist, permit one MCU software reset. Save the
latest mailbox torque request and target in RTC backup register 2 first, so OFF
remains OFF and the target is not replaced with center on recovery startup.
Recovery startup skips the normal torque-OFF/mode setup sequence. Backup register
1 holds a spent reset budget: neither automatic servo nor MCU reset is repeated
while that marker remains. Ordinary software/manual resets do not clear it;
service must clear the backup domain to re-arm (retention depends on board power).
Register 0 remains reserved for the CAN bootloader. The saved request is consumed
once; later unrelated resets use the normal startup behavior.

Communication-error recovery continues at low frequency after the reset budget
is spent. The configured hardware shutdown protections are not disabled.
Automatic MCU reset briefly interrupts all handle telemetry; this path requires
physical validation before claiming reliable field recovery.
