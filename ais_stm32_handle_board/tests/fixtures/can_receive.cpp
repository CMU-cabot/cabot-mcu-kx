// Test model: an ON frame is unread when the next target frame arrives.
// The real task_read() is inserted below; SPI and RTOS are simulated here.
#include <cstdint>
#include <cstdio>

struct can_frame {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};

constexpr int semaphoreCanIO = 1;
constexpr int semaphoreCanISR = 2;
constexpr int portMAX_DELAY = 0;
int task_read_count = 0;
int completed_iterations = 0;
struct StopTask {};

void xSemaphoreTake(int semaphore, int) {
    // Allow two iterations so a frame arriving during the first read is consumed.
    if (semaphore == semaphoreCanISR && ++completed_iterations == 2) {
        throw StopTask{};
    }
}
void xSemaphoreGive(int) {}
void debug_println(const char*) {}

bool torque_enabled = false;
int position = 2048;
constexpr int target_position = 1900;

void process_message(can_frame frame) {
    if (frame.can_id == 0x98) {
        torque_enabled = frame.data[0] != 0;
    }
    if (frame.can_id == 0x99 && torque_enabled) {
        position = target_position;
    }
}

struct MCP2515 {
    enum {
        CANINTF_RX0IF = 1, CANINTF_RX1IF = 2,
        RXB0 = 0, RXB1 = 1, ERROR_OK = 0
    };

    // Initially OFF; RXB0 contains an ON request that has not been read yet.
    bool occupied[2] = {true, false};
    can_frame frames[2] = {{0x98, 1, {1}}, {}};
    bool target_arrived = false;

    void deliver_target_once() {
        if (target_arrived) return;
        target_arrived = true;
        // Clearing RX0IF early makes RXB0 available and overwrites unread ON.
        int buffer = occupied[0] ? RXB1 : RXB0;
        frames[buffer] = {0x99, 4, {0}};
        occupied[buffer] = true;
    }

    int getInterrupts() {
        return (occupied[0] ? CANINTF_RX0IF : 0) |
               (occupied[1] ? CANINTF_RX1IF : 0);
    }

    void clearInterrupts() {
        occupied[0] = occupied[1] = false;
        deliver_target_once();
    }

    int readMessage(int buffer, can_frame* frame) {
        deliver_target_once();
        *frame = frames[buffer];
        occupied[buffer] = false;  // Release only AFTER copying, like the library.
        return ERROR_OK;
    }
} mcp2515;

// FIRMWARE_UNDER_TEST

int main() {
    try {
        task_read(nullptr);
    } catch (const StopTask&) {
        // Expected: the firmware task normally runs forever.
    }

    if (!torque_enabled || position != target_position) {
        std::fprintf(stderr, "Unread ON was lost: torque=%d, position=%d (expected %d)\n",
                     torque_enabled, position, target_position);
        return 1;
    }
    return 0;
}
