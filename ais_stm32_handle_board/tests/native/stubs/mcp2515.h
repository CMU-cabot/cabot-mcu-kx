#pragma once
#include <stdint.h>
struct can_frame { uint32_t can_id; uint8_t can_dlc; uint8_t data[8]; };
class MCP2515 {
public:
    enum RXBn { RXB0, RXB1 };
    enum { ERROR_OK, ERROR_FAIL };
    enum { CANINTF_RX0IF = 1, CANINTF_RX1IF = 2 };
    bool occupied[2] = {true, false};
    can_frame frames[2] = {{0x98, 1, {1}}, {}};
    bool target_arrived = false, fail_read = false, fail_send = false;
    can_frame last_sent = {};
    int send_calls = 0;
    void deliver_target() {
        if (target_arrived) return;
        target_arrived = true;
        int index = occupied[0] ? 1 : 0;
        frames[index] = {0x99, 4, {0}};
        occupied[index] = true;
    }
    uint8_t getInterrupts() { return occupied[0] | (occupied[1] << 1); }
    void clearInterrupts() { occupied[0] = occupied[1] = false; deliver_target(); }
    int readMessage(RXBn index, can_frame* frame) {
        if (fail_read) return ERROR_FAIL;
        deliver_target();
        *frame = frames[index];
        occupied[index] = false;
        return ERROR_OK;
    }
    int sendMessage(const can_frame* frame) {
        ++send_calls; last_sent = *frame;
        return fail_send ? ERROR_FAIL : ERROR_OK;
    }
};
