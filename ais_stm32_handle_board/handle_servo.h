#pragma once
#include <stdint.h>
#include <Dynamixel2Arduino.h>

// CAN task posts requests; the 20 ms task alone performs runtime UART I/O.
class HandleServo {
public:
    explicit HandleServo(Dynamixel2Arduino& device) : device_(device) {}
    void initialize();
    void request_torque(uint8_t command);
    void request_position(uint32_t position);
    bool update(uint32_t& position);
private:
    Dynamixel2Arduino& device_;
    bool available_ = false;
    volatile bool pending_ = false;
    volatile uint8_t command_ = 0;
    volatile uint32_t target_ = 2047;
};
