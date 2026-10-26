# Handle regression validation

Run from the repository root (Python 3 and g++ are required):

```sh
python3 -m unittest discover -s ais_stm32_handle_board/tests -v
```

## Confirmed regressions

- `ServoCommandTests` extracts the actual servo block from the firmware, compiles
  it with a fake Dynamixel transport, and executes multiple task cycles. Before
  the fix, one torque ON command caused three writes in three cycles because
  `onoff_recived == false` did not clear the flag. After the fix it sends once.
  A new OFF command injected during the ON UART transaction is preserved and
  executed exactly once. The pending command is consumed in a short critical
  section before UART I/O; UART I/O is never inside that critical section.
- `PacingTests` models two receive buffers draining at 0.5 ms per frame. On a
  50,744-byte image the unpaced updater dropped 72,459 frames in this model.
  With 1 ms pacing it completes with no modeled drops and within a 15-second
  virtual-time budget. This model isolates RX overflow, not CAN arbitration or
  all electrical/timing behavior. Permanent data loss now terminates after the
  configured number of stalled windows instead of looping forever.
- `BuildFailureTests` invokes the real build script with a compiler that exits
  23 and a successful bootloader stub. The script at `e8b5ccb` failed the test
  because it returned zero and started the bootloader build. The early-return
  fix already merged in PR #15 passes. This test protects that existing fix.

## Validation on 2026-10-26

- 19 tests: 18 passed, one SocketCAN simulator integration test skipped because
  vcan0 was not configured.
- STM32 Core 2.7.1 build: application 50,752 / 55,296 bytes; bootloader
  3,276 / 8,192 bytes. This includes the servo fix, but was not flashed.
- AIS-K25-14: the paced updater transferred the same previously validated
  50,744-byte image (CRC32 `a652f098`) over can0 in 14.019 seconds. The earlier
  unpaced run in this session took several minutes; it was not precisely timed.
  Commit, reboot, query result `6fc8c7c0807f`, and normal handle CAN telemetry
  (`080`, `089`, `09A`, `481`) were verified. The MCU image was unchanged for
  this comparison; only the host updater changed.

## Console Issue #53 limitation

The issue concerns AIS-K25-17 on 2026-10-23: fixed reported position, tracking
retries, then a directional-indicator tracking timeout and torque disable.
That evidence does not establish whether the initial failure was mechanical,
power, UART, or position feedback. The torque replay bug above is confirmed
in firmware, but these tests do **not** reproduce or prove the entire physical
Issue #53 failure. The issue must remain open pending command/feedback and
physical indicator validation on the affected device. Do not relax the host
tracking timeout or claim the hardware fault is resolved from these tests.

## Additional PR #17 regressions

- `test_can_receive.py` compiles the actual receive task with a two-buffer model.
  ON is pending while a target frame arrives during the read. At `16191d6`,
  clearing interrupt flags first lets the target overwrite ON, and the test fails
  with `OFF->ON followed by target lost ON; no motion`. The fixed task never
  globally clears pending RX flags, checks read results and drains both buffers;
  the same test passes. The model exercises a specific race, not proof of the
  historical device's firmware/timing.
- The actual servo-task harness injects one failed ON write after OFF while
  position reads continue. At `16191d6` the test fails with `OFF->ON UART failure
  never retried; position fixed`. The new monitor verifies torque and retries,
  and the same harness passes. Repeated-command and concurrent-OFF checks remain.
- `test_servo_monitor.py` covers bounded initial retries followed by 1 Hz recovery,
  new OFF superseding an ON retry, ACK/readback mismatch, invalid position reads,
  goal-write failures, hardware alerts, error saturation and diagnostic budgets.
  In 500 cycles, changing target counts produce at most 11 diagnostic frames;
  continuously changing error state produces at most 100. Normal polling performs
  20 extra UART reads in those 500 cycles.

To reproduce pre-fix failures without changing the checkout:

```sh
git show 16191d6:ais_stm32_handle_board/ais_stm32_handle_board.ino > /tmp/handle-before-fix.ino
HANDLE_TEST_SOURCE=/tmp/handle-before-fix.ino python3 -m unittest discover -s ais_stm32_handle_board/tests -p test_can_receive.py -v
HANDLE_TEST_SOURCE=/tmp/handle-before-fix.ino python3 -m unittest discover -s ais_stm32_handle_board/tests -p test_regressions.py -v
```

The two commands above are expected to fail. Omit HANDLE_TEST_SOURCE to test the
fixed firmware. These changes have not been flashed or mechanically validated.
See [SERVO_DIAGNOSTICS.md](../SERVO_DIAGNOSTICS.md) for the low-rate wire format
and non-halting error recovery behavior.

- `test_reset_context.py` executes the actual MCU reset function against backup
  register stubs: a newly received OFF overrides an older ON, the target is
  preserved, and invalid targets do not trigger reset. Recovery state tests
  confirm one servo then one MCU reset even with hardware-error/unknown status,
  and suppression of reset loops. Physical reboot timing remains unverified.

Final extended suite: **22 tests, 21 passed, one vcan integration skip**.
STM32 build: application **52,132 / 55,296 bytes**, bootloader **3,276 / 8,192 bytes**.
No firmware containing these added servo/reset changes has been installed on a device.
