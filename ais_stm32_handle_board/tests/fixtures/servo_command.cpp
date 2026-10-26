// Execute the real servo block with a fake UART device.
// This tests mailbox ordering, not UART timing or physical servo behavior.
#include <cstdint>
#include <cstdio>

#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
constexpr int DXL_ID = 1;
constexpr int ADDR_SERVO_POS = 0x9a;
constexpr int portMAX_DELAY = 0;
int semaphoreCanIO;
bool servo_enable = true;
volatile bool onoff_recived = false;
volatile unsigned char buff_en[1] = {0};
uint32_t servo_angle = 2047;

void receive_torque_command(bool enabled) {
    buff_en[0] = enabled ? 1 : 0;
    onoff_recived = true;
}

struct can_frame {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};

struct FakeServo {
    int on_calls = 0;
    int off_calls = 0;
    bool receive_off_during_on = false;
    uint32_t position = 2047;

    bool torqueOn(int) {
        ++on_calls;
        if (receive_off_during_on) {
            // Model a CAN request arriving while a UART transaction is running.
            receive_torque_command(false);
            receive_off_during_on = false;
        }
        return true;
    }
    bool torqueOff(int) {
        ++off_calls;
        return true;
    }
    bool setGoalPosition(int, uint32_t goal) {
        position = goal;
        return true;
    }
    uint32_t getPresentPosition(int) { return position; }
} dxl;

struct FakeCan {
    void sendMessage(can_frame*) {}
} mcp2515;
void xSemaphoreTake(int, int) {}
void xSemaphoreGive(int) {}
void delayMicroseconds(int) {}

void run_one_servo_cycle() {
    can_frame sendMsg;
    // FIRMWARE_UNDER_TEST
}

int main() {
    // Case 1: one ON request must cause exactly one write across three cycles.
    receive_torque_command(true);
    run_one_servo_cycle();
    run_one_servo_cycle();
    run_one_servo_cycle();
    if (dxl.on_calls != 1) {
        std::fprintf(stderr, "One ON request produced %d writes (expected 1)\n", dxl.on_calls);
        return 1;
    }

    // Case 2: OFF arriving during ON must survive and run on the next cycle.
    dxl.receive_off_during_on = true;
    receive_torque_command(true);
    run_one_servo_cycle();
    run_one_servo_cycle();
    run_one_servo_cycle();
    if (dxl.on_calls != 2 || dxl.off_calls != 1) {
        std::fprintf(stderr, "Concurrent OFF lost/repeated: ON=%d, OFF=%d (expected 2, 1)\n",
                     dxl.on_calls, dxl.off_calls);
        return 2;
    }
    return 0;
}
