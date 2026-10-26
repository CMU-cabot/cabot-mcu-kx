#pragma once
#include <stdint.h>
#include <cassert>
constexpr int OP_POSITION = 3;
inline void delayMicroseconds(int delay) { assert(delay == 500); }
class Dynamixel2Arduino {
public:
    int on_calls = 0, off_calls = 0;
    uint32_t position = 2047;
    bool ping_ok = true;
    int goal_calls = 0, position_reads = 0;
    void (*during_on)() = nullptr;
    void setPortProtocolVersion(float) {}
    bool ping(uint8_t) { return ping_ok; }
    bool torqueOn(uint8_t) {
        ++on_calls;
        if (during_on) { auto callback = during_on; during_on = nullptr; callback(); }
        return true;
    }
    bool torqueOff(uint8_t) { ++off_calls; return true; }
    void setOperatingMode(uint8_t, int) {}
    void setGoalPosition(uint8_t, uint32_t goal) { ++goal_calls; position = goal; }
    uint32_t getPresentPosition(uint8_t) { ++position_reads; return position; }
};
