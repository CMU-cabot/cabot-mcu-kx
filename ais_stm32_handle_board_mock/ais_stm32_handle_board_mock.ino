#include <STM32FreeRTOS.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Wire.h>
#include <Dynamixel2Arduino.h>
#include "Adafruit_VL53L0X.h"
#include "cap1203.h" 

#define SW_RIGHT      PF1
#define SW_LEFT       PB0
#define SW_UP         PF0
#define SW_DOWN       PB1
#define VIB_0         PA8
#define VIB_1         PA11
#define VIB_2         PA12

#define SPI_MOSI      PB5
#define SPI_MISO      PB4
#define SPI_SCK       PB3
#define SPI_CS_PIN    PA6
#define SPI_INT       PA7

#define I2C_SDA       PB7
#define I2C_SCL       PB6

#define DXIF_TXD      PA2
#define DXIF_RXD      PA3
#define DXIF_DIR      PA0

MCP_CAN CAN0(SPI_CS_PIN);
HardwareSerial dxif(DXIF_RXD, DXIF_TXD);
Dynamixel2Arduino dxl(dxif, DXIF_DIR);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
cap1203 cap_sens(&Wire);

volatile unsigned char buff0x1b[3];
volatile unsigned char buff0x1c[2];
volatile unsigned char buff0x20[1];

uint8_t vib0_count;
uint8_t vib1_count;
uint8_t vib2_count;

uint16_t servo_angle = 0;
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
      memcpy((unsigned char *)buff0x1c, buff, 2);
      servo_angle = ((uint16_t)buff0x1c[1]) << 8 | ((uint16_t)buff0x1c[0]) << 0;
      dxl.setGoalPosition(DXL_ID, servo_angle);
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
    uint16_t tof;
    byte data0x12[4] = {0};
    tof = lox.readRange();
    data0x12[0] = (tof & 0x00ff) >> 0;
    data0x12[1] = (tof & 0xff00) >> 8;
    data0x12[2] = cap_sens.getSensorInput1DeltaCountReg();
    data0x12[3] = cap_sens.getSensorInputStatusReg();
    cap_sens.setMainControlReg(false, false, false);
    CAN0.sendMsgBuf(0x12, 0, 4, data0x12);
    vTaskDelay(1);
    
    byte data0x13[1] = {0}; 
    data0x13[0] = digitalRead(SW_RIGHT) << 0
                | digitalRead(SW_LEFT)  << 1
                | digitalRead(SW_UP)    << 2
                | digitalRead(SW_DOWN)  << 3;
    data0x13[0] = (~data0x13[0])&0x0f;
    CAN0.sendMsgBuf(0x13, 0, 1, data0x13);
    vTaskDelay(1);

    uint16_t angle = dxl.getPresentPosition(DXL_ID);
    uint8_t data0x1f[2];
    data0x1f[0] = (angle & 0x00ff) >> 0;
    data0x1f[1] = (angle & 0xff00) >> 8;
    CAN0.sendMsgBuf(0x1f, 0, 2, data0x1f);
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
  dxl.begin(57600);

  pinMode(SW_RIGHT, INPUT_PULLUP);
  pinMode(SW_LEFT, INPUT_PULLUP);
  pinMode(SW_UP, INPUT_PULLUP);
  pinMode(SW_DOWN, INPUT_PULLUP);
  pinMode(VIB_0, OUTPUT);
  pinMode(VIB_1, OUTPUT);
  pinMode(VIB_2, OUTPUT);

  vib0_count = 0;
  vib1_count = 0;
  vib2_count = 0;

  servo_angle = 0;

  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_20MHZ);
  CAN0.setMode(MCP_NORMAL);
  pinMode(SPI_INT, INPUT);

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  if(dxl.ping(DXL_ID) == true)
  {
    servo_enable = true;
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.torqueOn(DXL_ID);
    dxl.setGoalPosition(DXL_ID, 2048);
  }

  lox.begin();
  lox.startRangeContinuous();

  cap_sens.begin();
  cap_sens.setSensitivityControlReg(BASE_DEF, DELTA_4X);

  xTaskCreate(task10ms, "task10ms", configMINIMAL_STACK_SIZE, NULL, 9,  NULL);
  xTaskCreate(task20ms, "task20ms", configMINIMAL_STACK_SIZE, NULL, 9,  NULL);
  
  attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);
  
  vTaskStartScheduler(); 
}

void loop()
{
  delay(1);
}
