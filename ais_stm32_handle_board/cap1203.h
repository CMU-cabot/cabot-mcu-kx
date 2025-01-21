#ifndef _CAP1203
#define _CAP1203

#include <Arduino.h>
#include <Wire.h>

#define BASE_DEF    0x0F
#define BASE_256X   0x08
#define BASE_128X   0x07
#define BASE_64X    0x06
#define BASE_32X    0x05
#define BASE_16X    0x04
#define BASE_8X     0x03
#define BASE_4X     0x02
#define BASE_2X     0x01
#define BASE_1X     0x00

#define DELTA_DEF   0x02
#define DELTA_128X  0x00
#define DELTA_64X   0x01
#define DELTA_32X   0x02
#define DELTA_16X   0x03
#define DELTA_8X    0x04
#define DELTA_4X    0x05
#define DELTA_2X    0x06
#define DELTA_1X    0x07

#define ADDR_MAIN_CONTROL                   0x00
#define ADDR_GENERAL_STATUS                 0x02
#define ADDR_SENSOR_INPUT_STATUS            0x03
#define ADDR_NOISE_FLAG_STATUS              0x0A
#define ADDR_SENSOR_INPUT_1_DELTA_COUNT     0x10
#define ADDR_SENSOR_INPUT_2_DELTA_COUNT     0x11
#define ADDR_SENSOR_INPUT_3_DELTA_COUNT     0x12
#define ADDR_SENSITIVITY_CONTROL            0x1F
#define ADDR_CONFIGURATION                  0x20
#define ADDR_SENSOR_INPUT_ENABLE            0x21
#define ADDR_CALIBREATION_STATUS            0x26
#define ADDR_INTERRUPT_ENABLE               0x27
#define ADDR_RECALIBRATION_CONFIGURATION    0x2F
#define ADDR_CONFIGURATION2                 0x44

#define RC_NEG_DELTA_MASK                   0x18
#define DIS_ANA_NOISE_MASK                  0x10
#define BC_OUT_RECAL_MASK                   0x40


class cap1203
{
    public:
        cap1203(TwoWire* i2c);
        cap1203(TwoWire* i2c, uint8_t addr);
        bool begin();
        
        void setMainControlReg(bool _int, bool _dsleep, bool _stby);
        void setSensitivityControlReg(uint8_t _base_shift, uint8_t _delta_sense);
        void setInterruptEnableReg(bool _cs1_int_en, bool _cs2_int_en, bool _cs3_int_en);
        void setCalibrationStatusReg(uint8_t status);
        void setNegativeDeltaCountReg(uint8_t status);
        void setSensorInputEnableReg(uint8_t status);
        void setConfigurationReg(uint8_t status);
        void setConfiguration2Reg(uint8_t status);

        uint8_t getMainControlReg();
        uint8_t getGeneralStatusReg();
        uint8_t getSensorInputStatusReg();
        int8_t  getSensorInput1DeltaCountReg();
        int8_t  getSensorInput2DeltaCountReg();
        int8_t  getSensorInput3DeltaCountReg();
        uint8_t getSensitivityControlReg();
        uint8_t getInterruptEnableReg();
        uint8_t getNoiseFlagStatsReg();
        uint8_t getCalibrationStatusReg();
        
    private:
        uint8_t cap1203_addr = 0x28;
        TwoWire* port;
        void readReg(uint8_t reg, uint8_t* data, uint8_t len);
        void writeReg(uint8_t reg, uint8_t* data, uint8_t len);
};
#endif
