#include "can_transport.h"
#include <cassert>

int main() {
    MCP2515 controller;
    volatile SemaphoreHandle_t mutex = 1;
    CanTransport can(controller, mutex);
    can_frame frame = {};

    // ON is unread in RXB0 when a target arrives during its read.
    assert(can.interrupts() == MCP2515::CANINTF_RX0IF);
    assert(can.receive(MCP2515::RXB0, frame));
    assert(frame.can_id == 0x98 && frame.data[0] == 1);
    assert(can.interrupts() == MCP2515::CANINTF_RX1IF);
    assert(can.receive(MCP2515::RXB1, frame));
    assert(frame.can_id == 0x99);

    controller.fail_read = true;
    assert(!can.receive(MCP2515::RXB0, frame));
}
