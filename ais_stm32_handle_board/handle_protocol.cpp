#include "handle_protocol.h"

void HandleProtocol::handle(const can_frame& frame) {
    // Keep existing command semantics; normal-command DLC policy is unchanged.
    if (frame.can_id == ADDR_VIB) {
        if (frame.data[0]) vibration0_ = frame.data[0];
        if (frame.data[1]) vibration1_ = frame.data[1];
        if (frame.data[2]) vibration2_ = frame.data[2];
    } else if (frame.can_id == ADDR_SERVO_TGT) {
        const uint32_t target = uint32_t(frame.data[0]) |
            (uint32_t(frame.data[1]) << 8) | (uint32_t(frame.data[2]) << 16) |
            (uint32_t(frame.data[3]) << 24);
        servo_.request_position(target);
    } else if (frame.can_id == ADDR_SERVO_EN) {
        servo_.request_torque(frame.data[0]);
    } else if (frame.can_id >= ADDR_CAP_WR1 && frame.can_id <= ADDR_CAP_WR5) {
        xSemaphoreTake(touch_mutex_, portMAX_DELAY);
        switch (frame.can_id) {
        case ADDR_CAP_WR1: touch_.setCalibrationStatusReg(frame.data[0]); break;
        case ADDR_CAP_WR2: touch_.setNegativeDeltaCountReg(frame.data[0]); break;
        case ADDR_CAP_WR3: touch_.setSensorInputEnableReg(frame.data[0]); break;
        case ADDR_CAP_WR4: touch_.setConfigurationReg(frame.data[0]); break;
        case ADDR_CAP_WR5: touch_.setConfiguration2Reg(frame.data[0]); break;
        }
        xSemaphoreGive(touch_mutex_);
    }
}

void HandleProtocol::send_tof_touch(uint16_t distance, uint8_t delta, uint8_t status) {
    can_frame frame = {};
    frame.can_id = ADDR_TOFCAP;
    frame.can_dlc = 4;
    frame.data[0] = distance;
    frame.data[1] = distance >> 8;
    frame.data[2] = delta;
    frame.data[3] = status;
    can_.send(frame);
}

void HandleProtocol::send_touch_status(uint8_t general, uint8_t noise, uint8_t calibration) {
    can_frame frame = {};
    frame.can_id = ADDR_CAP_STAT;
    frame.can_dlc = 3;
    frame.data[0] = general;
    frame.data[1] = noise;
    frame.data[2] = calibration;
    can_.send(frame);
}

void HandleProtocol::send_buttons(uint8_t buttons) {
    can_frame frame = {};
    frame.can_id = ADDR_TACT;
    frame.can_dlc = 1;
    frame.data[0] = buttons;
    can_.send(frame);
}

void HandleProtocol::send_position(uint32_t position) {
    can_frame frame = {};
    frame.can_id = ADDR_SERVO_POS;
    frame.can_dlc = 4;
    for (uint8_t i = 0; i < 4; ++i) frame.data[i] = position >> (8 * i);
    can_.send(frame);
}
