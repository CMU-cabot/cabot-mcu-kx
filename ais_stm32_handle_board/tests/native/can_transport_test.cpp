#include "can_transport.h"
#include "test_cases.h"
#include <cassert>

void unread_frame_survives_next_arrival() {
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

}

void driver_failures_are_returned_to_caller() {
    MCP2515 controller;
    volatile SemaphoreHandle_t mutex = 1;
    CanTransport can(controller, mutex);
    can_frame frame = {0x80, 2, {0x34, 0x12}};
    controller.fail_read = true;
    assert(!can.receive(MCP2515::RXB0, frame));
    assert(can.send(frame));
    assert(controller.last_sent.can_id == frame.can_id);
    assert(controller.last_sent.can_dlc == 2 && controller.last_sent.data[0] == 0x34);
    controller.fail_send = true;
    assert(!can.send(frame));
}

int main() {
    RUN_TEST(unread_frame_survives_next_arrival);
    RUN_TEST(driver_failures_are_returned_to_caller);
}
