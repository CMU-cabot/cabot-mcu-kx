#include "handle_protocol.h"
#include "test_cases.h"
#include <cassert>
#include <cstring>
#include <initializer_list>

// Link-time substitute for the touch driver; the product's header is unchanged.
namespace touch_record { int writes; uint8_t reg, value; }
cap1203::cap1203(TwoWire* wire) : port(wire) {}
void cap1203::setCalibrationStatusReg(uint8_t v) { ++touch_record::writes; touch_record::reg = 1; touch_record::value = v; }
void cap1203::setNegativeDeltaCountReg(uint8_t v) { ++touch_record::writes; touch_record::reg = 2; touch_record::value = v; }
void cap1203::setSensorInputEnableReg(uint8_t v) { ++touch_record::writes; touch_record::reg = 3; touch_record::value = v; }
void cap1203::setConfigurationReg(uint8_t v) { ++touch_record::writes; touch_record::reg = 4; touch_record::value = v; }
void cap1203::setConfiguration2Reg(uint8_t v) { ++touch_record::writes; touch_record::reg = 5; touch_record::value = v; }

struct Fixture {
    MCP2515 controller;
    volatile SemaphoreHandle_t can_mutex = 1, touch_mutex = 2;
    CanTransport can{controller, can_mutex};
    Dynamixel2Arduino device;
    HandleServo servo{device};
    cap1203 touch{nullptr};
    uint8_t vibration0 = 10, vibration1 = 20, vibration2 = 30;
    HandleProtocol protocol{can, servo, touch, touch_mutex, vibration0, vibration1, vibration2};
    Fixture() {
        touch_record::writes = 0;
        servo.initialize();
        device.on_calls = device.off_calls = 0;
        device.goal_calls = device.position_reads = 0;
    }
};

void vibration_zero_preserves_existing_count() {
    Fixture f;
    f.protocol.handle({ADDR_VIB, 3, {4, 0, 6}});
    assert(f.vibration0 == 4 && f.vibration1 == 20 && f.vibration2 == 6);
    assert(f.device.on_calls == 0 && touch_record::writes == 0);
}

void servo_commands_decode_without_immediate_uart_access() {
    Fixture f;
    f.protocol.handle({ADDR_SERVO_TGT, 4, {0x78, 0x56, 0x34, 0x12}});
    f.protocol.handle({ADDR_SERVO_EN, 1, {0}});
    assert(f.device.goal_calls == 0 && f.device.off_calls == 0);
    uint32_t position;
    assert(f.servo.update(position));
    assert(position == 0x12345678 && f.device.off_calls == 1);
    assert(touch_record::writes == 0);
}

void touch_commands_select_only_requested_register() {
    Fixture f;
    for (uint8_t index = 0; index < 5; ++index) {
        f.protocol.handle({uint32_t(ADDR_CAP_WR1 + index), 1, {uint8_t(0xa0 + index)}});
        assert(touch_record::writes == index + 1);
        assert(touch_record::reg == index + 1 && touch_record::value == 0xa0 + index);
    }
    assert(f.device.on_calls == 0 && f.device.off_calls == 0);
}

void unrelated_and_firmware_ids_do_not_operate_handle() {
    Fixture f;
    for (uint32_t id : {0x777u, 0x7a0u}) {
        f.protocol.handle({id, 8, {1, 'C', 'B', 'L', '1'}});
    }
    assert(touch_record::writes == 0 && f.controller.send_calls == 0);
    assert(f.device.on_calls == 0 && f.device.off_calls == 0 && f.device.goal_calls == 0);
    assert(f.vibration0 == 10 && f.vibration1 == 20 && f.vibration2 == 30);
}

void expect_frame(const Fixture& f, uint32_t id, uint8_t length, const uint8_t* data) {
    assert(f.controller.last_sent.can_id == id);
    assert(f.controller.last_sent.can_dlc == length);
    assert(std::memcmp(f.controller.last_sent.data, data, length) == 0);
}

void telemetry_has_expected_id_length_and_byte_order() {
    Fixture f;
    const uint8_t tof[] = {0x34, 0x12, 0xfe, 0x80};
    f.protocol.send_tof_touch(0x1234, 0xfe, 0x80);
    expect_frame(f, ADDR_TOFCAP, 4, tof);
    const uint8_t touch[] = {0x81, 0x42, 0x24};
    f.protocol.send_touch_status(touch[0], touch[1], touch[2]);
    expect_frame(f, ADDR_CAP_STAT, 3, touch);
    const uint8_t buttons[] = {0x0d};
    f.protocol.send_buttons(buttons[0]);
    expect_frame(f, ADDR_TACT, 1, buttons);
    const uint8_t position[] = {0x78, 0x56, 0x34, 0x12};
    f.protocol.send_position(0x12345678);
    expect_frame(f, ADDR_SERVO_POS, 4, position);
    assert(f.controller.send_calls == 4);
}

int main() {
    RUN_TEST(vibration_zero_preserves_existing_count);
    RUN_TEST(servo_commands_decode_without_immediate_uart_access);
    RUN_TEST(touch_commands_select_only_requested_register);
    RUN_TEST(unrelated_and_firmware_ids_do_not_operate_handle);
    RUN_TEST(telemetry_has_expected_id_length_and_byte_order);
}
