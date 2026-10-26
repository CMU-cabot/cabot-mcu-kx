#pragma once
#include <stdint.h>
#include "FirmwareProtocol.h"

// No Arduino, CAN controller, RTOS, allocation, or STM32 dependency.
// The board owns callback lifetimes and decides whether it can enter update mode.
class FirmwareService {
public:
    struct Config {
        uint32_t request_id;
        uint32_t response_id;
        const char* version;  // 1..64 printable ASCII bytes, static lifetime
        void* context;
        bool (*send)(void*, uint32_t, const uint8_t*, uint8_t);
        bool (*can_enter)(void*);
        void (*enter_bootloader)(void*); // board-specific ACK drain / reset
    };

    explicit FirmwareService(const Config& config) : config_(config) {}
    // True means this frame belongs to this service, even if malformed.
    bool handle(uint32_t id, uint8_t length, const uint8_t* data) const;

private:
    Config config_;
};
