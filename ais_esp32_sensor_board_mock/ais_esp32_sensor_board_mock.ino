#include <CAN.h>
#include <SPI.h>
#include "WiFi.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>

#include <utility/imumaths.h>

#define DIVIDER80   80
#define GPIO_ANT1   2
#define GPIO_ANT2   25

#define I2C_SDA     21
#define I2C_SCL     22
#define BME_CS      17

#define SEALEVELPRESSURE_HPA (1013.25)

TaskHandle_t thp[3];

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
imu::Vector<3> acc;
imu::Vector<3> gyro;
imu::Vector<3> mag;
imu::Quaternion quat;

Adafruit_BME280 bme(BME_CS);

volatile unsigned char buff0x31[8] = {0};
volatile unsigned char buff0x32[8] = {0};
volatile unsigned char buff0x33[8] = {0};

volatile bool flag_buff0x31 = false;
volatile bool flag_buff0x32 = false;
volatile bool flag_buff0x33 = false;

adafruit_bno055_offsets_t calib_data;

void task10ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount();
  while(true)
  {
    acc  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    quat = bno.getQuat();
  
    CAN.beginPacket(0x09);
    CAN.write((((int16_t)(acc.x()*100)) & 0x00ff)>>0);
    CAN.write((((int16_t)(acc.x()*100)) & 0xff00)>>8);
    CAN.write((((int16_t)(acc.y()*100)) & 0x00ff)>>0);
    CAN.write((((int16_t)(acc.y()*100)) & 0xff00)>>8);
    CAN.write((((int16_t)(acc.z()*100)) & 0x00ff)>>0);
    CAN.write((((int16_t)(acc.z()*100)) & 0xff00)>>8);
    CAN.endPacket();
    ets_delay_us(200);
    
    CAN.beginPacket(0x0a);
    CAN.write((((int16_t)(gyro.x()*16)) & 0x00ff)>>0);
    CAN.write((((int16_t)(gyro.x()*16)) & 0xff00)>>8);
    CAN.write((((int16_t)(gyro.y()*16)) & 0x00ff)>>0);
    CAN.write((((int16_t)(gyro.y()*16)) & 0xff00)>>8);
    CAN.write((((int16_t)(gyro.z()*16)) & 0x00ff)>>0);
    CAN.write((((int16_t)(gyro.z()*16)) & 0xff00)>>8);
    CAN.endPacket();
    ets_delay_us(200);
    
    CAN.beginPacket(0x0b);
    CAN.write((((int16_t)(quat.x()*32767)) & 0x00ff)>>0);
    CAN.write((((int16_t)(quat.x()*32767)) & 0xff00)>>8);
    CAN.write((((int16_t)(quat.y()*32767)) & 0x00ff)>>0);
    CAN.write((((int16_t)(quat.y()*32767)) & 0xff00)>>8);
    CAN.write((((int16_t)(quat.z()*32767)) & 0x00ff)>>0);
    CAN.write((((int16_t)(quat.z()*32767)) & 0xff00)>>8);
    CAN.write((((int16_t)(quat.w()*32767)) & 0x00ff)>>0);
    CAN.write((((int16_t)(quat.w()*32767)) & 0xff00)>>8);
    CAN.endPacket();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void task50ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 50;
  xLastWakeTime = xTaskGetTickCount();
  while(true)
  {
    for(int i=0;i<13;i++)
    {
      WiFi.scanNetworks(true, false, false, 50, i+1);
      ets_delay_us(50000);
      int n = WiFi.scanComplete();
      
      if(n<=0){continue;}

      char data[32] = {0x1A};
      String ssid  = WiFi.SSID(0);
      byte *bssid = WiFi.BSSID(0);
      int8_t rssi = WiFi.RSSI(0);

      ssid.toCharArray(data, ssid.length()+1);
      for(int f=0;f<4;f++)
      {
        CAN.beginPacket(0x0c+f);
        for(int b=0;b<8;b++)
        {
          CAN.write(data[f*8+b]);
        }
        CAN.endPacket();
        ets_delay_us(200);
      }
      
      CAN.beginPacket(0x10);
      CAN.write(bssid[0]);
      CAN.write(bssid[1]);
      CAN.write(bssid[2]);
      CAN.write(bssid[3]);
      CAN.write(bssid[4]);
      CAN.write(bssid[5]);
      CAN.write(i);
      CAN.write(rssi);
      CAN.endPacket();
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

    CAN.beginPacket(0x11);
    CAN.write((temp & 0x00ff) >> 0);
    CAN.write((temp & 0xff00) >> 8);
    CAN.write((hum  & 0x00ff) >> 0);
    CAN.write((hum  & 0xff00) >> 8);
    CAN.write((pres & 0x000000ff) >> 0);
    CAN.write((pres & 0x0000ff00) >> 8);
    CAN.write((pres & 0x00ff0000) >> 16);
    CAN.write((pres & 0xff000000) >> 24);    
    CAN.endPacket();
    ets_delay_us(200);

    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    CAN.beginPacket(0x37);
    CAN.write(system);
    CAN.write(gyro);
    CAN.write(accel);
    CAN.write(mag);
    CAN.endPacket();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
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

  if(!bme.begin())
  {
    Serial.println("BME280 Error");
  }
  
  // start the CAN bus at 1Mbps
  if (!CAN.begin(1000E3)) {
    while (1);
  }
  
  if (!bno.begin())
  {
    while (1);
  }
  delay(100);
  bno.setMode(OPERATION_MODE_NDOF);

  xTaskCreatePinnedToCore(task10ms,  "task10ms",  4096, NULL, 10, &thp[0], 1);
  xTaskCreatePinnedToCore(task50ms,  "task50ms",  4096, NULL, 5,  &thp[1], 1); 
  xTaskCreatePinnedToCore(task500ms, "task500ms", 4096, NULL, 5,  &thp[2], 1); 
  
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
      if(CAN.packetId() == 0x04)
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
          
          CAN.beginPacket(0x34);
          CAN.write((calib_data.accel_offset_x & 0x00ff)>>0);
          CAN.write((calib_data.accel_offset_x & 0xff00)>>8);
          CAN.write((calib_data.accel_offset_y & 0x00ff)>>0);
          CAN.write((calib_data.accel_offset_y & 0xff00)>>8);
          CAN.write((calib_data.accel_offset_z & 0x00ff)>>0);
          CAN.write((calib_data.accel_offset_z & 0xff00)>>8);
          CAN.write((calib_data.mag_offset_x   & 0x00ff)>>0);
          CAN.write((calib_data.mag_offset_x   & 0xff00)>>8);
          CAN.endPacket();
          ets_delay_us(200);

          CAN.beginPacket(0x35);
          CAN.write((calib_data.mag_offset_y   & 0x00ff)>>0);
          CAN.write((calib_data.mag_offset_y   & 0xff00)>>8);
          CAN.write((calib_data.mag_offset_z   & 0x00ff)>>0);
          CAN.write((calib_data.mag_offset_z   & 0xff00)>>8);
          CAN.write((calib_data.gyro_offset_x  & 0x00ff)>>0);
          CAN.write((calib_data.gyro_offset_x  & 0xff00)>>8);
          CAN.write((calib_data.gyro_offset_y  & 0x00ff)>>0);
          CAN.write((calib_data.gyro_offset_y  & 0xff00)>>8);
          CAN.endPacket();
          ets_delay_us(200);

          CAN.beginPacket(0x36);
          CAN.write((calib_data.gyro_offset_z  & 0x00ff)>>0);
          CAN.write((calib_data.gyro_offset_z  & 0xff00)>>8);
          CAN.write((calib_data.accel_radius   & 0x00ff)>>0);
          CAN.write((calib_data.accel_radius   & 0xff00)>>8);
          CAN.write((calib_data.mag_radius     & 0x00ff)>>0);
          CAN.write((calib_data.mag_radius     & 0xff00)>>8);
          CAN.endPacket();
          ets_delay_us(200);
        }
      
        if(buff[0] == 0x01 && flag_buff0x31 && flag_buff0x32 && flag_buff0x33)
        {
          calib_data.accel_offset_x = (int16_t)(buff0x31[1]<<8) | (int16_t)(buff0x31[0]);
          calib_data.accel_offset_y = (int16_t)(buff0x31[3]<<8) | (int16_t)(buff0x31[2]);
          calib_data.accel_offset_z = (int16_t)(buff0x31[5]<<8) | (int16_t)(buff0x31[4]);

          calib_data.mag_offset_x   = (int16_t)(buff0x31[7]<<8) | (int16_t)(buff0x31[6]);
          calib_data.mag_offset_y   = (int16_t)(buff0x32[1]<<8) | (int16_t)(buff0x32[0]);
          calib_data.mag_offset_z   = (int16_t)(buff0x32[3]<<8) | (int16_t)(buff0x32[2]);

          calib_data.gyro_offset_x  = (int16_t)(buff0x32[5]<<8) | (int16_t)(buff0x32[4]);
          calib_data.gyro_offset_y  = (int16_t)(buff0x32[7]<<8) | (int16_t)(buff0x32[6]);
          calib_data.gyro_offset_z  = (int16_t)(buff0x33[1]<<8) | (int16_t)(buff0x33[0]);

          calib_data.accel_radius   = (int16_t)(buff0x33[3]<<8) | (int16_t)(buff0x33[2]);
          calib_data.mag_radius     = (int16_t)(buff0x33[5]<<8) | (int16_t)(buff0x33[4]);

          bno.setSensorOffsets(calib_data);
          flag_buff0x31 = false;
          flag_buff0x32 = false;
          flag_buff0x33 = false;
        }
      }
      if(CAN.packetId() == 0x31)
      {
        memcpy((unsigned char *)buff0x31, buff, 8);
        flag_buff0x31 = true;
      }
      if(CAN.packetId() == 0x32)
      {
        memcpy((unsigned char *)buff0x32, buff, 8);
        flag_buff0x32 = true;
      }
      if(CAN.packetId() == 0x33)
      {
        memcpy((unsigned char *)buff0x33, buff, 8);
        flag_buff0x33 = true;
      }
    }
  }
  delay(1);
}
