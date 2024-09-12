#include <STM32FreeRTOS.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Wire.h>
#include <Dynamixel2Arduino.h>
#include "Adafruit_VL53L0X.h"
#include "cap1203.h" 

#define SW_RIGHT      PA7
#define SW_LEFT       PA6
#define SW_UP         PA4
#define SW_DOWN       PA5
#define VIB_0         PA8
#define VIB_1         PA11
#define VIB_2         PA12

#define SPI_MOSI      PB5
#define SPI_MISO      PB4
#define SPI_SCK       PB3
#define SPI_CS_PIN    PA15
#define SPI_INT       PA13

#define I2C_SDA       PB7
#define I2C_SCL       PB6

#define STM32_TXD     PA9
#define STM32_RXD     PA10

#define CAP_RST       PA1
#define MCP_RST       PA14

#define DXIF_TXD      PA2
#define DXIF_RXD      PA3
#define DXIF_DIR      PA0

#define GOAL_POSITION_ADDR      116
#define PRESENT_POSITION_ADDR   132
#define TIMEOUT                 10

MCP_CAN CAN0(SPI_CS_PIN);
HardwareSerial dxif(DXIF_RXD, DXIF_TXD);
Dynamixel2Arduino dxl(dxif, DXIF_DIR);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
cap1203 cap_sens(&Wire);

volatile unsigned char buff0x1b[3];
volatile unsigned char buff0x1c[4];
volatile unsigned char buff0x20[1];

byte data0x12[4] = {0};
byte data0x13[1] = {0}; 
uint8_t data0x1f[4];

uint8_t vib0_count;
uint8_t vib1_count;
uint8_t vib2_count;

uint32_t servo_angle = 2047;
bool servo_enable = false;
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
using namespace ControlTableItem;

void mcpISR(){
  //Serial.println("isr");
  long unsigned int id;
  unsigned char len = 0;
  unsigned char buff[8];

  CAN0.readMsgBuf(&id, &len, buff);
  
  if((id & 0x80000000) == 0x80000000){return;}

  if(id == 0x1b)
  {
    memcpy((unsigned char *)buff0x1b, buff, 3);
    vib0_count = (buff0x1b[0]!=0) ? buff0x1b[0] : vib0_count;
    vib1_count = (buff0x1b[1]!=0) ? buff0x1b[1] : vib1_count;
    vib2_count = (buff0x1b[2]!=0) ? buff0x1b[2] : vib2_count;
  }
  if(id == 0x1c)
  {
    if(servo_enable)
    {
      memcpy((unsigned char *)buff0x1c, buff, 4);
      servo_angle = ((uint32_t)buff0x1c[3]) << 24 | ((uint32_t)buff0x1c[2]) << 16 | ((uint16_t)buff0x1c[1]) << 8 | ((uint16_t)buff0x1c[0]) << 0;
    }
  }
  if(id == 0x20)
  {
    if(servo_enable)
    {
      memcpy((unsigned char *)buff0x20, buff, 1);
      if(buff0x20[0] == 0x00)
      {
        dxl.torqueOff(DXL_ID);
      }
      if(buff0x20[0] == 0x01)
      {
        dxl.torqueOn(DXL_ID);
      }
    }
  }
}
void task10ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    //Serial.println("10ms");
    if(vib0_count>0){digitalWrite(VIB_0, HIGH);vib0_count--;}
    else{digitalWrite(VIB_0, LOW);}
    if(vib1_count>0){digitalWrite(VIB_1, HIGH);vib1_count--;}
    else{digitalWrite(VIB_1, LOW);}
    if(vib2_count>0){digitalWrite(VIB_2, HIGH);vib2_count--;}
    else{digitalWrite(VIB_2, LOW);}

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void task20ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 20;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    taskENTER_CRITICAL();
    uint16_t tof;
    tof = lox.readRangeResult();
    data0x12[0] = (tof & 0x00ff) >> 0;
    data0x12[1] = (tof & 0xff00) >> 8;
    data0x12[2] = cap_sens.getSensorInput1DeltaCountReg();
    data0x12[3] = cap_sens.getSensorInputStatusReg();
    cap_sens.setMainControlReg(false, false, false);
    CAN0.sendMsgBuf(0x12, 0, 4, data0x12);
    delayMicroseconds(500);
    
    
    data0x13[0] = digitalRead(SW_RIGHT) << 0
                | digitalRead(SW_LEFT)  << 1
                | digitalRead(SW_UP)    << 2
                | digitalRead(SW_DOWN)  << 3;
    data0x13[0] = (data0x13[0])&0x0f;
    CAN0.sendMsgBuf(0x13, 0, 1, data0x13);
    delayMicroseconds(500);

    if(servo_enable)
    {
      //dxl.write(DXL_ID, GOAL_POSITION_ADDR, (uint8_t*)&servo_angle, 4, TIMEOUT);
      dxl.setGoalPosition(DXL_ID, servo_angle);
      delayMicroseconds(500);
      //dxl.read(DXL_ID, PRESENT_POSITION_ADDR, 4, (uint8_t*)&angle, , TIMEOUT);
      uint32_t angle = dxl.getPresentPosition(DXL_ID);
      
      data0x1f[0] = (angle & 0x000000ff) >> 0;
      data0x1f[1] = (angle & 0x0000ff00) >> 8;
      data0x1f[2] = (angle & 0x00ff0000) >> 16;
      data0x1f[3] = (angle & 0xff000000) >> 24;
      CAN0.sendMsgBuf(0x1f, 0, 4, data0x1f);
    }
    taskEXIT_CRITICAL();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup()
{
  SPI.setMISO(SPI_MISO);
  SPI.setMOSI(SPI_MOSI);
  SPI.setSCLK(SPI_SCK);

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200);
  dxl.begin(115200);

  pinMode(SW_RIGHT, INPUT_PULLUP);
  pinMode(SW_LEFT, INPUT_PULLUP);
  pinMode(SW_UP, INPUT_PULLUP);
  pinMode(SW_DOWN, INPUT_PULLUP);
  pinMode(VIB_0, OUTPUT);
  pinMode(VIB_1, OUTPUT);
  pinMode(VIB_2, OUTPUT);
  pinMode(CAP_RST, OUTPUT);
  pinMode(MCP_RST, OUTPUT);

  digitalWrite(CAP_RST, LOW);
  digitalWrite(MCP_RST, HIGH);
  delay(10);
  digitalWrite(CAP_RST, HIGH);
  digitalWrite(MCP_RST, LOW);
  delay(10);
  digitalWrite(CAP_RST, LOW);
  digitalWrite(MCP_RST, HIGH);
  delay(10);

  vib0_count = 0;
  vib1_count = 0;
  vib2_count = 0;

  servo_angle = 2047;

  while(!lox.begin())
  {
    delay(10);
  }
  lox.startRangeContinuous(8);

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  if(dxl.ping(DXL_ID) == true)
  {
    servo_enable = true;
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.torqueOn(DXL_ID);
    dxl.setGoalPosition(DXL_ID, 2047);
  }

  cap_sens.begin();
  cap_sens.setSensitivityControlReg(BASE_DEF, DELTA_4X);

  CAN0.begin(MCP_STDEXT, CAN_1000KBPS, MCP_20MHZ);
  
  CAN0.init_Mask(0,0,0x7ff0000);
  CAN0.init_Filt(0,0,0x01b0000);
  CAN0.init_Filt(1,0,0x01c0000);
  
  CAN0.init_Mask(1,0,0x7ff0000);
  CAN0.init_Filt(2,0,0x0200000);
  
  pinMode(SPI_INT, INPUT);
  CAN0.setMode(MCP_NORMAL);
  
  attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);

  xTaskCreate(task10ms, "task10ms", configMINIMAL_STACK_SIZE, NULL, 9,  NULL);
  xTaskCreate(task20ms, "task20ms", configMINIMAL_STACK_SIZE, NULL, 9,  NULL);

  vTaskStartScheduler(); 
}

void loop()
{
  delay(1);
}
