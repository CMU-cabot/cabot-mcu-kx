#include "FirmwareService.h"
#include <cassert>
#include <cstring>

struct Board {
    bool permitted = true, send_ok = true, entered = false;
    uint32_t response_id = 0;
    unsigned sends = 0;
    uint8_t response[8] = {};
};
bool send(void* context, uint32_t id, const uint8_t* data, uint8_t length) {
    auto& board = *static_cast<Board*>(context);
    assert(length == 8);
    ++board.sends;
    board.response_id = id;
    std::memcpy(board.response, data, 8);
    return board.send_ok;
}
bool permitted(void* context) { return static_cast<Board*>(context)->permitted; }
void enter(void* context) {
    auto& board = *static_cast<Board*>(context);
    assert(board.response[0] == (OP_ENTER | 0x80) && board.response[1] == STATUS_OK);
    board.entered = true;
}

int main() {
    // Same service supports two boards with distinct IDs and versions.
    for (unsigned index = 0; index < 2; ++index) {
        Board board;
        uint32_t request_id = 0x7a0 + index * 8;
        FirmwareService service({request_id, request_id + 1, "abcdef123456", &board,
                                 send, permitted, enter});
        uint8_t request[8] = {OP_APP_VERSION};
        assert(!service.handle(0x98, 8, request));
        assert(service.handle(request_id, 1, request) && board.sends == 0);
        for (uint8_t offset = 0; offset < 12; offset += 4) {
            request[1] = offset;
            assert(service.handle(request_id, 8, request));
            assert(board.response_id == request_id + 1);
            assert(board.response[1] == STATUS_OK && board.response[3] == 12);
            assert(std::memcmp(board.response + 4, "abcdef123456" + offset, 4) == 0);
        }
        request[1] = 1;
        service.handle(request_id, 8, request);
        assert(board.response[1] == STATUS_BAD_ARGUMENT);
        request[1] = 12;
        service.handle(request_id, 8, request);
        assert(board.response[1] == STATUS_BAD_ARGUMENT);

        uint8_t boot[8] = {OP_ENTER, 'C', 'B', 'L', '1'};
        board.permitted = false;
        service.handle(request_id, 8, boot);
        assert(!board.entered && board.response[1] == STATUS_BAD_STATE);
        board.permitted = true;
        board.send_ok = false;
        service.handle(request_id, 8, boot);
        assert(!board.entered); // Never reset before successfully queuing the ACK.
        board.send_ok = true;
        service.handle(request_id, 8, boot);
        assert(board.entered);
    }
}
