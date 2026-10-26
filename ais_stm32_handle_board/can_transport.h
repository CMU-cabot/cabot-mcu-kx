#pragma once
#include <STM32FreeRTOS.h>
#include <mcp2515.h>

// Serializes SPI access. No application CAN IDs or command interpretation here.
class CanTransport {
public:
    CanTransport(MCP2515& controller, volatile SemaphoreHandle_t& mutex)
        : controller_(controller), mutex_(mutex) {}
    uint8_t interrupts();
    bool receive(MCP2515::RXBn buffer, can_frame& frame);
    bool send(const can_frame& frame);
private:
    MCP2515& controller_;
    volatile SemaphoreHandle_t& mutex_;
};
