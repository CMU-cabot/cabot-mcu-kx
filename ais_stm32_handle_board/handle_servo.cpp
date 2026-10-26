#include "handle_servo.h"
#include <STM32FreeRTOS.h>

namespace { constexpr uint8_t servo_id = 1; }

void HandleServo::initialize() {
    device_.setPortProtocolVersion(2.0);
    available_ = device_.ping(servo_id);
    if (available_) {
        device_.torqueOff(servo_id);
        device_.setOperatingMode(servo_id, OP_POSITION);
        device_.torqueOn(servo_id);
        device_.setGoalPosition(servo_id, 2047);
    }
}

void HandleServo::request_torque(uint8_t command) {
    if (!available_) return;
    taskENTER_CRITICAL();
    command_ = command;
    pending_ = true;
    taskEXIT_CRITICAL();
}

void HandleServo::request_position(uint32_t position) {
    if (available_) target_ = position;
}

bool HandleServo::update(uint32_t& position) {
    if (!available_) return false;
    taskENTER_CRITICAL();
    const bool pending = pending_;
    const uint8_t command = command_;
    pending_ = false;
    taskEXIT_CRITICAL();
    if (pending) {
        if (command == 0) device_.torqueOff(servo_id);
        if (command == 1) device_.torqueOn(servo_id);
    }
    device_.setGoalPosition(servo_id, target_);
    delayMicroseconds(500);
    // Preserve existing read/report behavior; CRC policy is a separate change.
    position = device_.getPresentPosition(servo_id);
    return true;
}
