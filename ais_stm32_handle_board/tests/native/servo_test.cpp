#include "handle_servo.h"
#include "test_cases.h"
#include <cassert>

struct Fixture {
    Dynamixel2Arduino device;
    HandleServo servo{device};
    uint32_t position = 0;
    Fixture() {
        servo.initialize();
        device.on_calls = device.off_calls = 0;
        device.goal_calls = device.position_reads = 0;
    }
};

void each_torque_request_is_applied_once() {
    Fixture f;
    f.servo.request_torque(1);
    for (int i = 0; i < 3; ++i) assert(f.servo.update(f.position));
    assert(f.device.on_calls == 1 && f.device.off_calls == 0);
    f.servo.request_torque(0);
    for (int i = 0; i < 3; ++i) f.servo.update(f.position);
    assert(f.device.on_calls == 1 && f.device.off_calls == 1);
}

HandleServo* receiving_servo = nullptr;
void receive_off() { receiving_servo->request_torque(0); }
void off_arriving_during_on_survives_until_next_cycle() {
    Fixture f;
    receiving_servo = &f.servo;
    f.device.during_on = receive_off;
    f.servo.request_torque(1);
    f.servo.update(f.position);
    assert(f.device.on_calls == 1 && f.device.off_calls == 0);
    f.servo.update(f.position);
    f.servo.update(f.position);
    assert(f.device.on_calls == 1 && f.device.off_calls == 1);
    receiving_servo = nullptr;
}

void latest_request_before_cycle_wins() {
    Fixture f;
    f.servo.request_torque(1);
    f.servo.request_torque(0);
    f.servo.request_position(1900);
    f.servo.request_position(2200);
    f.servo.update(f.position);
    assert(f.device.on_calls == 0 && f.device.off_calls == 1);
    assert(f.position == 2200);
}

void unavailable_servo_does_not_issue_runtime_uart_operations() {
    Dynamixel2Arduino device;
    device.ping_ok = false;
    HandleServo servo(device);
    servo.initialize();
    servo.request_torque(1);
    servo.request_position(1900);
    uint32_t position = 123;
    assert(!servo.update(position));
    assert(position == 123);
    assert(device.on_calls == 0 && device.off_calls == 0);
    assert(device.goal_calls == 0 && device.position_reads == 0);
}

int main() {
    RUN_TEST(each_torque_request_is_applied_once);
    RUN_TEST(off_arriving_during_on_survives_until_next_cycle);
    RUN_TEST(latest_request_before_cycle_wins);
    RUN_TEST(unavailable_servo_does_not_issue_runtime_uart_operations);
}
