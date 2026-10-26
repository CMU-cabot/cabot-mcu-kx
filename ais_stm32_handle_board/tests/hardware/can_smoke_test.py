#!/usr/bin/env python3
"""On-device CAN test. Read-only unless --motion-cycles is explicitly nonzero."""
import argparse
import json
from pathlib import Path
import socket
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from can_update import BootloaderClient, SocketCanTransport

FRAME = struct.Struct("=IB3x8s")
TELEMETRY = {0x080: 4, 0x089: 1, 0x09A: 4, 0x481: 3}
COMMANDS = {0x098, 0x099}


class TestFailure(RuntimeError):
    pass


class CanTest:
    def __init__(self, interface, output):
        self.interface = interface
        self.started = time.monotonic()
        self.competing_sender = False
        self.raw = (output / "frames.jsonl").open("w")
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        filters = b"".join(struct.pack("=II", cid, 0xC00007FF)
                           for cid in (*TELEMETRY, *COMMANDS))
        self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        self.socket.bind((interface,))
        self.socket.settimeout(0.05)

    def log(self, direction, can_id, data):
        row = {"seconds": time.monotonic() - self.started,
               "direction": direction, "id": hex(can_id), "data": data.hex()}
        self.raw.write(json.dumps(row) + "\n")
        return row

    def send(self, can_id, data):
        frame = FRAME.pack(can_id, len(data), data.ljust(8, b"\0"))
        if self.socket.send(frame) != len(frame):
            raise TestFailure("short CAN write")
        self.log("tx", can_id, data)

    def capture(self, duration):
        rows = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            try:
                raw = self.socket.recv(FRAME.size)
            except socket.timeout:
                continue
            can_id, length, data = FRAME.unpack(raw)
            row = self.log("rx", can_id, data[:length])
            if can_id in COMMANDS:
                self.competing_sender = True
                raise TestFailure("another sender is issuing servo commands")
            if length != TELEMETRY[can_id]:
                raise TestFailure(f"unexpected DLC on {can_id:#x}: {length}")
            rows.append(row)
        self.raw.flush()
        return rows

    def version(self):
        transport = SocketCanTransport(self.interface)
        try:
            return BootloaderClient(transport).query_application_version()
        finally:
            transport.close()

    def close(self):
        self.socket.close()
        self.raw.close()


def check_telemetry(rows, duration):
    result = {}
    for can_id in TELEMETRY:
        samples = [r for r in rows if r["id"] == hex(can_id)]
        rate = len(samples) / duration
        gaps = [b["seconds"] - a["seconds"] for a, b in zip(samples, samples[1:])]
        maximum_gap = max(gaps, default=duration)
        result[hex(can_id)] = {"samples": len(samples), "hz": round(rate, 2),
                               "max_gap_seconds": round(maximum_gap, 4)}
        if not 40 <= rate <= 60 or maximum_gap > 0.2:
            raise TestFailure(f"telemetry outside 40–60 Hz / 200 ms gap: {result}")
    return result


def positions(rows):
    values = [int.from_bytes(bytes.fromhex(r["data"]), "little")
              for r in rows if r["id"] == "0x9a"]
    if any(value > 4095 for value in values):
        raise TestFailure("position outside 0..4095")
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--expected-version", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--motion-cycles", type=int, default=0,
                        help="OFF->ON trials, alternating 1900/2200; requires clear indicator")
    args = parser.parse_args()
    if not 0 <= args.motion_cycles <= 100:
        parser.error("--motion-cycles must be 0..100")
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"expected_version": args.expected_version, "interface": args.interface,
              "motion_cycles_requested": args.motion_cycles, "trials": [], "passed": False}
    test = CanTest(args.interface, args.output)
    moved = False
    try:
        report["version_before"] = test.version()
        if report["version_before"] != args.expected_version:
            raise TestFailure("unexpected firmware version")
        rows = test.capture(3)
        report["baseline"] = check_telemetry(rows, 3)
        report["initial_position"] = positions(rows)[-1]
        for trial in range(args.motion_cycles):
            goal = 1900 if trial % 2 == 0 else 2200
            moved = True
            test.send(0x098, b"\0")
            test.capture(0.15)
            test.send(0x098, b"\1")
            test.send(0x099, struct.pack("<I", goal))
            rows = test.capture(1.5)
            observed = positions(rows)
            result = {"trial": trial + 1, "target": goal,
                      "last_position": observed[-1] if observed else None,
                      "samples": len(observed),
                      "passed": len(observed) >= 5 and
                      all(abs(value - goal) <= 20 for value in observed[-5:])}
            report["trials"].append(result)
            print(json.dumps(result), flush=True)
            check_telemetry(rows, 1.5)
            if not result["passed"]:
                raise TestFailure("target not reached within 1.5 s / 20 counts")
        report["version_after"] = test.version()
        if report["version_after"] != args.expected_version:
            raise TestFailure("version changed during test")
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
    finally:
        if moved and test.competing_sender:
            report["restore_error"] = "restoration skipped: another controller is active"
            report["passed"] = False
        if moved and not test.competing_sender:
            try:
                # Restore center and ON, matching this handle's normal startup state.
                test.send(0x099, struct.pack("<I", 2047))
                test.send(0x098, b"\1")
                rows = test.capture(2)
                observed = positions(rows)
                report["restore_position"] = observed[-1] if observed else None
                if not observed or abs(observed[-1] - 2047) > 20:
                    raise TestFailure("center restoration not confirmed")
            except Exception as error:
                report["restore_error"] = str(error)
                report["passed"] = False
        test.close()
        (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
