#pragma once
#include "can_transport.h"
#include "handle_servo.h"
#include "cap1203.h"

//for mass production model
#define ADDR_TOFCAP     0x080       // 0b0 001 0000 000  ToF/Touch sensor data
#define ADDR_TACT       0x089       // 0b0 001 0001 001  TACT SW data
#define ADDR_VIB        0x090       // 0b0 001 0010 000  Vibrator command
#define ADDR_SERVO_EN   0x098       // 0b0 001 0011 000  Servo enable/disable
#define ADDR_SERVO_TGT  0x099       // 0b0 001 0011 001  Servor target position
#define ADDR_SERVO_POS  0x09a       // 0b0 001 0011 010  Servo present position
#define ADDR_CAP_BASE   0x480       // 0b1 001 0000 000  CAP1203 Debug base address
#define ADDR_CAP_STAT   0x481       // 0b1 001 0000 001  CAP1203 Debug sensor status
#define ADDR_CAP_WR1    0x482       // 0b1 001 0000 010  CAP1203 Debug write data 1 (setCalibrationStatusReg)
#define ADDR_CAP_WR2    0x483       // 0b1 001 0000 011  CAP1203 Debug write data 2 (setNegativeDeltaCountReg)
#define ADDR_CAP_WR3    0x484       // 0b1 001 0000 100  CAP1203 Debug write data 3 (setSensorInputEnableReg)
#define ADDR_CAP_WR4    0x485       // 0b1 001 0000 100  CAP1203 Debug write data 4 (setConfigurationReg)
#define ADDR_CAP_WR5    0x486       // 0b1 001 0000 100  CAP1203 Debug write data 5 (setConfiguration2Reg)

class HandleProtocol {
public:
    HandleProtocol(CanTransport& can, HandleServo& servo, cap1203& touch,
                   volatile SemaphoreHandle_t& touch_mutex,
                   uint8_t& vibration0, uint8_t& vibration1, uint8_t& vibration2)
        : can_(can), servo_(servo), touch_(touch), touch_mutex_(touch_mutex),
          vibration0_(vibration0), vibration1_(vibration1), vibration2_(vibration2) {}
    void handle(const can_frame& frame);
    void send_tof_touch(uint16_t distance, uint8_t delta, uint8_t status);
    void send_touch_status(uint8_t general, uint8_t noise, uint8_t calibration);
    void send_buttons(uint8_t buttons);
    void send_position(uint32_t position);
private:
    CanTransport& can_;
    HandleServo& servo_;
    cap1203& touch_;
    volatile SemaphoreHandle_t& touch_mutex_;
    uint8_t& vibration0_;
    uint8_t& vibration1_;
    uint8_t& vibration2_;
};
