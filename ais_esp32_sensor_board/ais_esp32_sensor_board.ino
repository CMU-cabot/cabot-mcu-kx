#include <CAN.h>
#include <SPI.h>
#include "WiFi.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>

#include <utility/imumaths.h>

#define DIVIDER80     80
#define GPIO_ANT1     2
#define GPIO_ANT2     25

#define I2C_SDA       21
#define I2C_SCL       22

#define BME_CS        17
#define ADDR_BNO      0x28
#define BME_RST       32
#define BNO_RST       14

#define ADDR_IMU_ACC  0x010   // 0b0 000 0010 000  IMU acceleration
#define ADDR_IMU_ANG  0x011   // 0b0 000 0010 001  IMU angular velocity 
#define ADDR_IMU_ORI  0x012   // 0b0 000 0010 010  IMU orientation
#define ADDR_BME      0x018   // 0b0 000 0011 000  BME280 temperature / humidity / pressure
#define ADDR_IMU_CAL  0x430   // 0b1 000 0110 000  IMU SensorOffsets read/write command
#define ADDR_IMU_WR0  0x431   // 0b1 000 0110 001  IMU SensorOffsets data write 0 ~ 7 Byte
#define ADDR_IMU_WR1  0x432   // 0b1 000 0110 010  IMU SensorOffsets data write8 ~ 15 Byte
#define ADDR_IMU_WR2  0x433   // 0b1 000 0110 011  IMU SensorOffsets data write16 ~ 21 Byte
#define ADDR_IMU_STAT 0x438   // 0b1 000 0111 000  IMU Calibration status system/gyro/accel/mag
#define ADDR_IMU_RE0  0x439   // 0b1 000 0111 001  IMU SensorOffsets data read 0 ~ 7 Byte
#define ADDR_IMU_RE1  0x43a   // 0b1 000 0111 010  IMU SensorOffsets data read 8 ~ 15 Byte
#define ADDR_IMU_RE2  0x43b   // 0b1 000 0111 011  IMU SensorOffsets data read 16 ~ 21 Byte
#define ADDR_WIFI0    0x428   // 0b1 000 0101 000  WIFI data ssid 0 ~ 7 Byte
#define ADDR_WIFI1    0x429   // 0b1 000 0101 001  WIFI data ssid 8 ~ 15 Byte
#define ADDR_WIFI2    0x42a   // 0b1 000 0101 010  WIFI data ssid 16 ~ 23 Byte
#define ADDR_WIFI3    0x42b   // 0b1 000 0101 011  WIFI data ssid 24 ~ 31 Byte
#define ADDR_WIFI4    0x42c   // 0b1 000 0101 100  WIFI data bssid / ch / rssi

#define CAN_FILTER    0x430  // 0b1 000 0110 000  filter for ADDR_IMU_CAL ~ ADDR_IMU_WR2 (major=0, minor=6)
#define CAN_MASK      0x7f8  // 0b1 111 1111 000  mask by priority, major, minor

#define SEALEVELPRESSURE_HPA (1013.25)

TaskHandle_t thp[3];
xSemaphoreHandle xCANSemaphore;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
Adafruit_BME280 bme(BME_CS);

volatile unsigned char buff_imu_wr0[8] = {0};
volatile unsigned char buff_imu_wr1[8] = {0};
volatile unsigned char buff_imu_wr2[8] = {0};

volatile bool flag_buff_imu_wr0 = false;
volatile bool flag_buff_imu_wr1 = false;
volatile bool flag_buff_imu_wr2 = false;

adafruit_bno055_offsets_t calib_data;

Adafruit_I2CDevice *i2c = new Adafruit_I2CDevice(0x28, &Wire);
bool readlen(Adafruit_BNO055::adafruit_bno055_reg_t reg, byte *buffer, uint8_t len)
{
  uint8_t reg_buf[1] = {(uint8_t)reg};
  return i2c->write_then_read(reg_buf, 1, buffer, len);
}

void task10ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount();
  while(true)
  {
    byte acc[6]  = {0};
    byte gyro[6] = {0};
    byte quat[8] = {0};
    
    readlen(Adafruit_BNO055::BNO055_ACCEL_DATA_X_LSB_ADDR,      acc,  6);
    readlen(Adafruit_BNO055::BNO055_GYRO_DATA_X_LSB_ADDR,       gyro, 6);
    readlen(Adafruit_BNO055::BNO055_QUATERNION_DATA_W_LSB_ADDR, quat, 8);

    xSemaphoreTake(xCANSemaphore, portMAX_DELAY);
    CAN.beginPacket(ADDR_IMU_ACC);
    CAN.write(acc[0]);
    CAN.write(acc[1]);
    CAN.write(acc[2]);
    CAN.write(acc[3]);
    CAN.write(acc[4]);
    CAN.write(acc[5]);
    CAN.endPacket();
    ets_delay_us(200);

    CAN.beginPacket(ADDR_IMU_ANG);
    CAN.write(gyro[0]);
    CAN.write(gyro[1]);
    CAN.write(gyro[2]);
    CAN.write(gyro[3]);
    CAN.write(gyro[4]);
    CAN.write(gyro[5]);
    CAN.endPacket();
    ets_delay_us(200);

    CAN.beginPacket(ADDR_IMU_ORI);
    CAN.write(quat[2]);
    CAN.write(quat[3]);
    CAN.write(quat[4]);
    CAN.write(quat[5]);
    CAN.write(quat[6]);
    CAN.write(quat[7]);
    CAN.write(quat[0]);
    CAN.write(quat[1]);
    CAN.endPacket();
    xSemaphoreGive(xCANSemaphore);
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void taskWifi(void *pvParameters)
{
  while(true)
  {
    for(int ch=1;ch<=13;ch++)
    {
      WiFi.scanNetworks(true, false, false, 100, ch);
      vTaskDelay(100);
      int n = WiFi.scanComplete();
      if(n>0)
      {
        for(int i=0;i<n;i++)
        {
          char data[33] = {0x00};
          String ssid  = WiFi.SSID(i);
          byte *bssid = WiFi.BSSID(i);
          int8_t rssi = WiFi.RSSI(i);
          int8_t ch   = WiFi.channel(i);

          ssid.toCharArray(data, ssid.length()+1);
          
          for(int f=0;f<4;f++)
          {
            xSemaphoreTake(xCANSemaphore, portMAX_DELAY);
            CAN.beginPacket(ADDR_WIFI0+f);
            for(int b=0;b<8;b++)
            {
              CAN.write(data[f*8+b]);
            }
            CAN.endPacket();
            xSemaphoreGive(xCANSemaphore);
            ets_delay_us(200);
          }  

          xSemaphoreTake(xCANSemaphore, portMAX_DELAY);
          CAN.beginPacket(ADDR_WIFI4);
          CAN.write(bssid[0]);
          CAN.write(bssid[1]);
          CAN.write(bssid[2]);
          CAN.write(bssid[3]);
          CAN.write(bssid[4]);
          CAN.write(bssid[5]);
          CAN.write(ch);
          CAN.write(rssi);
          CAN.endPacket();
          xSemaphoreGive(xCANSemaphore);
        }
      }
    }
  }
}

void task500ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 500;
  xLastWakeTime = xTaskGetTickCount();
  while(true)
  {
    int16_t temp = (int16_t)(bme.readTemperature() * 100);
    int16_t hum  = (int16_t)(bme.readHumidity()    * 100);
    int32_t pres = (uint32_t)(bme.readPressure()   * 100);

    xSemaphoreTake(xCANSemaphore, portMAX_DELAY);
    CAN.beginPacket(ADDR_BME);
    CAN.write((temp & 0x00ff) >> 0);
    CAN.write((temp & 0xff00) >> 8);
    CAN.write((hum  & 0x00ff) >> 0);
    CAN.write((hum  & 0xff00) >> 8);
    CAN.write((pres & 0x000000ff) >> 0);
    CAN.write((pres & 0x0000ff00) >> 8);
    CAN.write((pres & 0x00ff0000) >> 16);
    CAN.write((pres & 0xff000000) >> 24);    
    CAN.endPacket();
    xSemaphoreGive(xCANSemaphore);
    ets_delay_us(200);

    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    xSemaphoreTake(xCANSemaphore, portMAX_DELAY);
    CAN.beginPacket(ADDR_IMU_STAT);
    CAN.write(system);
    CAN.write(gyro);
    CAN.write(accel);
    CAN.write(mag);
    CAN.endPacket();
    xSemaphoreGive(xCANSemaphore);
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  pinMode(BME_RST, OUTPUT);
  pinMode(BNO_RST, OUTPUT);

  digitalWrite(BME_RST, HIGH);
  digitalWrite(BNO_RST, LOW);
  delay(100);
  digitalWrite(BME_RST, LOW);
  digitalWrite(BNO_RST, HIGH);
  delay(1000);
  
  calib_data = {0};
  Serial.begin(1000000);
  WiFi.mode(WIFI_STA);

  bool err = WiFi.setDualAntennaConfig(GPIO_ANT1, GPIO_ANT2, WIFI_RX_ANT_AUTO, WIFI_TX_ANT_AUTO);
  if(err == false)
  {
    Serial.println("Dual Antenna configuration failed!");
  }else
  {
    Serial.println("Dual Antenna configuration successfuly done!");
  }
  WiFi.disconnect();

  if(bme.begin())
  {
    Serial.println("BME280 ok");
  }
  else
  {
    Serial.println("BME280 Error");
  }
  
  // start the CAN bus at 1Mbps
  while(!CAN.begin(1000E3)) {
    delay(10);  //Retry interval adjust
  }

  CAN.filter(CAN_FILTER, CAN_MASK);
  
  //CAN bug fix
  volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
  *pREG_IER &= ~(uint8_t)0x10;
  
  while(!bno.begin())
  {
    delay(10);
  }
  delay(100);
  bno.setMode(OPERATION_MODE_NDOF);
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P2);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P2);

  xCANSemaphore = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(task10ms,  "task10ms",  4096, NULL, 10, &thp[0], 1);
  xTaskCreatePinnedToCore(taskWifi,  "taskWifi",  4096, NULL, 5,  &thp[1], 1); 
  xTaskCreatePinnedToCore(task500ms, "task500ms", 4096, NULL, 3,  &thp[2], 1); 
  
}

void loop() {
  unsigned char buff[8];
  int packetSize = CAN.parsePacket();
  if (packetSize)
  {
    if(!CAN.packetExtended() && !CAN.packetRtr())
    {
      for(int i=0;i<packetSize;i++)
      {
        buff[i] = CAN.read();
      }
      if(CAN.packetId() == ADDR_IMU_CAL)
      {
        if(buff[0] == 0x00)
        {
          bno.getSensorOffsets(calib_data);
          Serial.println(calib_data.accel_offset_x);
          Serial.println(calib_data.accel_offset_y);
          Serial.println(calib_data.accel_offset_z);
          Serial.println(calib_data.mag_offset_x);
          Serial.println(calib_data.mag_offset_y);
          Serial.println(calib_data.mag_offset_z);
          Serial.println(calib_data.gyro_offset_x);
          Serial.println(calib_data.gyro_offset_y);
          Serial.println(calib_data.gyro_offset_z);
          Serial.println(calib_data.accel_radius);
          Serial.println(calib_data.mag_radius);

          xSemaphoreTake(xCANSemaphore, portMAX_DELAY);
          CAN.beginPacket(ADDR_IMU_RE0);
          CAN.write(((uint16_t)calib_data.accel_offset_x & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.accel_offset_x & 0xff00)>>8);
          CAN.write(((uint16_t)calib_data.accel_offset_y & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.accel_offset_y & 0xff00)>>8);
          CAN.write(((uint16_t)calib_data.accel_offset_z & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.accel_offset_z & 0xff00)>>8);
          CAN.write(((uint16_t)calib_data.mag_offset_x   & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.mag_offset_x   & 0xff00)>>8);
          CAN.endPacket();
          xSemaphoreGive(xCANSemaphore);
          ets_delay_us(200);

          xSemaphoreTake(xCANSemaphore, portMAX_DELAY);
          CAN.beginPacket(ADDR_IMU_RE1);
          CAN.write(((uint16_t)calib_data.mag_offset_y   & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.mag_offset_y   & 0xff00)>>8);
          CAN.write(((uint16_t)calib_data.mag_offset_z   & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.mag_offset_z   & 0xff00)>>8);
          CAN.write(((uint16_t)calib_data.gyro_offset_x  & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.gyro_offset_x  & 0xff00)>>8);
          CAN.write(((uint16_t)calib_data.gyro_offset_y  & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.gyro_offset_y  & 0xff00)>>8);
          CAN.endPacket();
          xSemaphoreGive(xCANSemaphore);
          ets_delay_us(200);

          xSemaphoreTake(xCANSemaphore, portMAX_DELAY);
          CAN.beginPacket(ADDR_IMU_RE2);
          CAN.write(((uint16_t)calib_data.gyro_offset_z  & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.gyro_offset_z  & 0xff00)>>8);
          CAN.write(((uint16_t)calib_data.accel_radius   & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.accel_radius   & 0xff00)>>8);
          CAN.write(((uint16_t)calib_data.mag_radius     & 0x00ff)>>0);
          CAN.write(((uint16_t)calib_data.mag_radius     & 0xff00)>>8);
          CAN.endPacket();
          xSemaphoreGive(xCANSemaphore);
          ets_delay_us(200);
        }
      
        if(buff[0] == 0x01 && flag_buff_imu_wr0 && flag_buff_imu_wr1 && flag_buff_imu_wr2)
        {
          calib_data.accel_offset_x = (int16_t)(buff_imu_wr0[1]<<8) | (int16_t)(buff_imu_wr0[0]);
          calib_data.accel_offset_y = (int16_t)(buff_imu_wr0[3]<<8) | (int16_t)(buff_imu_wr0[2]);
          calib_data.accel_offset_z = (int16_t)(buff_imu_wr0[5]<<8) | (int16_t)(buff_imu_wr0[4]);

          calib_data.mag_offset_x   = (int16_t)(buff_imu_wr0[7]<<8) | (int16_t)(buff_imu_wr0[6]);
          calib_data.mag_offset_y   = (int16_t)(buff_imu_wr1[1]<<8) | (int16_t)(buff_imu_wr1[0]);
          calib_data.mag_offset_z   = (int16_t)(buff_imu_wr1[3]<<8) | (int16_t)(buff_imu_wr1[2]);

          calib_data.gyro_offset_x  = (int16_t)(buff_imu_wr1[5]<<8) | (int16_t)(buff_imu_wr1[4]);
          calib_data.gyro_offset_y  = (int16_t)(buff_imu_wr1[7]<<8) | (int16_t)(buff_imu_wr1[6]);
          calib_data.gyro_offset_z  = (int16_t)(buff_imu_wr2[1]<<8) | (int16_t)(buff_imu_wr2[0]);

          calib_data.accel_radius   = (int16_t)(buff_imu_wr2[3]<<8) | (int16_t)(buff_imu_wr2[2]);
          calib_data.mag_radius     = (int16_t)(buff_imu_wr2[5]<<8) | (int16_t)(buff_imu_wr2[4]);

          bno.setSensorOffsets(calib_data);
          flag_buff_imu_wr0 = false;
          flag_buff_imu_wr1 = false;
          flag_buff_imu_wr2 = false;
        }
      }
      if(CAN.packetId() == ADDR_IMU_WR0)
      {
        memcpy((unsigned char *)buff_imu_wr0, buff, 8);
        flag_buff_imu_wr0 = true;
      }
      if(CAN.packetId() == ADDR_IMU_WR1)
      {
        memcpy((unsigned char *)buff_imu_wr1, buff, 8);
        flag_buff_imu_wr1 = true;
      }
      if(CAN.packetId() == ADDR_IMU_WR2)
      {
        memcpy((unsigned char *)buff_imu_wr2, buff, 8);
        flag_buff_imu_wr2 = true;
      }
    }
  }
  delay(1);
}
