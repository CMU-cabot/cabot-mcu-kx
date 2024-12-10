#include "cap1203.h"

cap1203::cap1203(TwoWire* i2c)
{
    this->port = i2c;
}

cap1203::cap1203(TwoWire* i2c, uint8_t addr)
{
    this->port = i2c;
    this->cap1203_addr = addr;
}

bool cap1203::begin()
{
    //device connection check
    uint8_t err;
    this->port->beginTransmission(cap1203_addr);
    err = this->port->endTransmission();
    if(err != 0x00){return false;}

    cap1203::setSensitivityControlReg(BASE_DEF, DELTA_DEF);
    cap1203::setInterruptEnableReg(true, true, true);//CS1 CS2 CS3
    cap1203::setMainControlReg(false, false, false);//clear interrupt
    return true;
}
        
void cap1203::setMainControlReg(bool _int, bool _dsleep, bool _stby)
{
    uint8_t data[1];
    data[0] = ((uint8_t)(_int&0x01))<<0 
            | ((uint8_t)(_dsleep&0x01))<<4 
            | ((uint8_t)(_stby&0x01))<<5;
    cap1203::writeReg(ADDR_MAIN_CONTROL, data, 1);
}

void cap1203::setSensitivityControlReg(uint8_t _base_shift, uint8_t _delta_sense)
{
    uint8_t data[1];
    data[0] = (_delta_sense&0x07)<<4 | (_base_shift&0x0f)<<0;
    cap1203::writeReg(ADDR_SENSITIVITY_CONTROL, data, 1);
}

void cap1203::setInterruptEnableReg(bool _cs1_int_en, bool _cs2_int_en, bool _cs3_int_en)
{
    uint8_t data[1];
    data[0] = ((uint8_t)(_cs1_int_en&0x01))<<0 
            | ((uint8_t)(_cs2_int_en&0x01))<<1 
            | ((uint8_t)(_cs3_int_en&0x01))<<2;
    cap1203::writeReg(ADDR_INTERRUPT_ENABLE, data, 1);
}

void cap1203::setCalibrationStatusReg(uint8_t status)
{
    uint8_t data[1];
    data[0] = status;
    cap1203::writeReg(ADDR_CALIBREATION_STATUS, data, 1);
}

void cap1203::setNegativeDeltaCountReg(uint8_t status) {
    uint8_t data[1];
    cap1203::readReg(ADDR_RECALIBRATION_CONFIGURATION, data, 1);
    data[0] = (data[0] & ~ RC_NEG_DELTA_MASK) | (status & RC_NEG_DELTA_MASK);
    cap1203::writeReg(ADDR_RECALIBRATION_CONFIGURATION, data, 1);
}

uint8_t cap1203::getMainControlReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_MAIN_CONTROL, data, 1);
    return data[0];
}

uint8_t cap1203::getGeneralStatusReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_GENERAL_STATUS, data, 1);
    return data[0];
}

uint8_t cap1203::getSensorInputStatusReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_SENSOR_INPUT_STATUS, data, 1);
    return data[0];
}

int8_t cap1203::getSensorInput1DeltaCountReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_SENSOR_INPUT_1_DELTA_COUNT, data, 1);
    return data[0];
}

int8_t cap1203::getSensorInput2DeltaCountReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_SENSOR_INPUT_2_DELTA_COUNT, data, 1);
    return data[0];
}

int8_t cap1203::getSensorInput3DeltaCountReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_SENSOR_INPUT_3_DELTA_COUNT, data, 1);
    return data[0];
}

uint8_t cap1203::getSensitivityControlReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_SENSITIVITY_CONTROL, data, 1);
    return data[0];
}

uint8_t cap1203::getInterruptEnableReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_INTERRUPT_ENABLE, data, 1);
    return data[0];
}

uint8_t cap1203::getNoiseFlagStatsReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_NOISE_FLAG_STATUS, data, 1);
    return data[0];
}

uint8_t cap1203::getCalibrationStatusReg()
{
    uint8_t data[1];
    cap1203::readReg(ADDR_CALIBREATION_STATUS, data, 1);
    return data[0];
}

void cap1203::readReg(uint8_t reg, uint8_t* data, uint8_t len)
{
    this->port->beginTransmission(cap1203_addr);
    this->port->write(reg);
    this->port->endTransmission(false);
    this->port->requestFrom(cap1203_addr, len);

    for(int i=0;i<len;i++)
    {
        if(this->port->available())
        {
            data[i] = this->port->read();
        }
    }
}

void cap1203::writeReg(uint8_t reg, uint8_t* data, uint8_t len)
{
    this->port->beginTransmission(cap1203_addr);
    this->port->write(reg);
    for(int i=0;i<len;i++)
    {
        this->port->write(data[i]);
    }
    this->port->endTransmission();
}
