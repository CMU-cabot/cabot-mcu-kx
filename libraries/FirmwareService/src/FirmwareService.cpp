#include "FirmwareService.h"

bool FirmwareService::handle(uint32_t id, uint8_t length, const uint8_t* data) const {
    if (id != config_.request_id) return false;
    if (length != 8 || !data) return true;

    uint8_t response[8] = {};
    if (data[0] == OP_ENTER && data[1] == 'C' && data[2] == 'B' &&
        data[3] == 'L' && data[4] == '1') {
        const bool permitted = config_.can_enter && config_.can_enter(config_.context);
        response[0] = OP_ENTER | 0x80;
        response[1] = permitted ? STATUS_OK : STATUS_BAD_STATE;
        response[2] = 0xff;
        response[6] = BOOT_PROTOCOL_VERSION;
        const bool sent = config_.send(config_.context, config_.response_id, response, 8);
        if (permitted && sent) config_.enter_bootloader(config_.context);
    } else if (data[0] == OP_APP_VERSION) {
        uint8_t size = 0;
        bool valid = config_.version != nullptr;
        if (valid) {
            while (size <= 64 && config_.version[size]) {
                const uint8_t c = config_.version[size];
                if (c < 33 || c > 126) valid = false;
                ++size;
            }
        }
        valid = valid && size > 0 && size <= 64;
        const uint8_t offset = data[1];
        response[0] = OP_APP_VERSION | 0x80;
        response[1] = valid && offset < size && offset % 4 == 0 ? STATUS_OK : STATUS_BAD_ARGUMENT;
        response[2] = offset;
        response[3] = valid ? size : 0;
        if (response[1] == STATUS_OK) {
            for (uint8_t i = 0; i < 4 && offset + i < size; ++i) {
                response[4 + i] = config_.version[offset + i];
            }
        }
        config_.send(config_.context, config_.response_id, response, 8);
    }
    return true;
}
