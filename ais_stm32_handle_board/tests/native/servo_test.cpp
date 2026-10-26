#include "handle_servo.h"
#include <cassert>

Dynamixel2Arduino device;
HandleServo servo(device);
void receive_off() { servo.request_torque(0); }

int main() {
    servo.initialize();
    device.on_calls = device.off_calls = 0;
    uint32_t position = 0;

    // One request must not repeat across three task cycles.
    servo.request_torque(1);
    for (int i = 0; i < 3; ++i) assert(servo.update(position));
    assert(device.on_calls == 1);

    // A later OFF arriving during UART ON is consumed on the next cycle.
    device.during_on = receive_off;
    servo.request_torque(1);
    servo.update(position);
    assert(device.off_calls == 0);
    servo.update(position);
    servo.update(position);
    assert(device.on_calls == 2 && device.off_calls == 1);

    servo.request_position(1900);
    servo.update(position);
    assert(position == 1900);
}
