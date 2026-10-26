#pragma once
#include <stdint.h>
#include <string.h>

// Owned by task20ms. CAN task only writes the existing atomic mailbox.
struct ServoMonitor {
  bool desired = true, actual = false, torque_valid = false, position_valid = false;
  bool pending = false, cycle_ok = true;
  uint32_t quiet_until = 0;
  uint32_t retry_at = 0;
  uint8_t remaining = 0, rx_torque = 0, rx_target = 0;
  uint8_t last_operation = 0, lib_error = 0, packet_error = 0, hardware_error = 255;
  uint8_t failures = 0;
  uint32_t ticks = 0, last_report = 0;
  uint8_t previous[8] = {};

  void command(bool on) { desired = on; remaining = 3; pending = true; }
  template<class D> bool check(D& d, bool ok, uint8_t op, bool allow_alert = false) {
    uint8_t lib = d.getLastLibErrCode(), packet = d.getLastStatusPacketError();
    bool valid = ok && lib == 0 && (packet == 0 || (allow_alert && (packet & 0x7f) == 0));
    if (valid && packet == 0) return true;
    last_operation = op; lib_error = lib; packet_error = packet;
    if (failures != 255) ++failures;
    return valid;
  }
  template<class D> bool pollTorque(D& d, uint8_t id, uint8_t item) {
    int32_t value = d.readControlTableItem(item, id);
    torque_valid = check(d, value == 0 || value == 1, 4, true);
    if (torque_valid) actual = value != 0;
    return torque_valid;
  }
  template<class D> bool step(D& d, uint8_t id, uint32_t goal,
                             uint8_t torque_item, uint8_t hardware_item, uint32_t& position) {
    ++ticks;
    if (static_cast<int32_t>(ticks - quiet_until) < 0) { position_valid = false; cycle_ok = true; return false; }
    // Failed commands remain pending: retry slowly after the initial burst.
    // No automatic torque OFF, reset, or permanent fault latch on errors.
    if (pending && remaining == 0 && static_cast<int32_t>(ticks - retry_at) >= 0) remaining = 1;
    bool attempted = remaining != 0;
    if (attempted) {
      --remaining;
      bool written = check(d, desired ? d.torqueOn(id) : d.torqueOff(id), 1);
      bool read = pollTorque(d, id, torque_item);
      if (written && read && actual == desired) { remaining = 0; pending = false; }
      else if (written && read) check(d, false, 1); // ACK but wrong readback
      if (pending && remaining == 0) retry_at = ticks + 50;
    }
    bool goal_ok = check(d, d.setGoalPosition(id, goal), 2);
    float value = d.getPresentPosition(id);
    position_valid = check(d, value >= 0 && value <= 4095, 3, true);
    if (position_valid) position = static_cast<uint32_t>(value);
    // Two extra reads per second, except bounded torque-command verification.
    if (ticks % 50 == 0) {
      if (!attempted) pollTorque(d, id, torque_item);
      int32_t hw = d.readControlTableItem(hardware_item, id);
      hardware_error = check(d, hw >= 0 && hw < 255, 5, true) ? hw : 255;
    }
    cycle_ok = goal_ok && position_valid && !pending;
    return position_valid;
  }
  bool report(uint8_t out[8]) {
    out[0] = 0x86; // diagnostic response on existing 0x7A1, distinct from version
    out[1] = (desired ? 1 : 0) | (actual ? 2 : 0) | (torque_valid ? 4 : 0) |
             (position_valid ? 8 : 0) | (pending ? 16 : 0) | (last_operation << 5);
    out[2] = rx_torque; out[3] = rx_target;
    out[4] = lib_error; out[5] = packet_error; out[6] = hardware_error; out[7] = failures;
    // Ignore receive counters for change detection: regular goals must not
    // increase diagnostic traffic. Errors are latched until the next error/reset.
    bool changed = out[1] != previous[1] || memcmp(out + 4, previous + 4, 4) != 0;
    if (ticks - last_report < 5 || (!changed && ticks - last_report < 50)) return false;
    memcpy(previous, out, 8); last_report = ticks; return true;
  }
};

struct ServoRecovery {
  uint16_t bad_cycles = 0;
  bool servo_reset_used = false, mcu_reset_used = false;
  uint8_t update(bool healthy, uint8_t /* hardware_error */) {
    // Status remains diagnostic; unavailable/error status must not block recovery.
    if (healthy) { bad_cycles = 0; return 0; }
    if (mcu_reset_used) return 0;
    if (++bad_cycles < 250) return 0;
    bad_cycles = 0;
    if (!servo_reset_used) { servo_reset_used = true; return 1; }
    mcu_reset_used = true; return 2;
  }
};
