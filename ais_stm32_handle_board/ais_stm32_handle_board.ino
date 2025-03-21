#include <STM32FreeRTOS.h>
#include <mcp2515.h>
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

#define CHATA_THRESHOLD         3

#define VIB_DUTY                127 //0~255

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

#define CAN_FILTER0     0x0090      // 0b0 001 0010 000 filter for ADDR_VIB (major=1, minor=2), ADDR_SERVO_* (major=1, minor=3)
#define CAN_FILTER1     0x0480      // 0b1 001 0000 000 filter for ADDR_CAP_WR*
#define CAN_MASK0       0x07f0      // 0b1 111 1110 000 mask by priority, major, minor (upper 3bit)
#define CAN_MASK1       0x07f8      // 0b1 111 1111 000 mask by priority, major, minor

#ifndef DEBUG                 // you can set DEBUG=1 to print debug message via Serial
#define DEBUG 0
#endif

MCP2515 mcp2515(SPI_CS_PIN);
HardwareSerial dxif(DXIF_RXD, DXIF_TXD);
Dynamixel2Arduino dxl(dxif, DXIF_DIR);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
cap1203 cap_sens(&Wire);

volatile SemaphoreHandle_t semaphoreCanISR;
volatile SemaphoreHandle_t semaphoreSerialIO;
volatile SemaphoreHandle_t semaphoreCanIO;
volatile SemaphoreHandle_t semaphoreCap1203IO;

volatile unsigned char buff_vib[3];
volatile unsigned char buff_tgt[4];
volatile unsigned char buff_en[1];
volatile unsigned char buff_wr1[1];
volatile unsigned char buff_wr2[1];
volatile unsigned char buff_wr3[1];
volatile unsigned char buff_wr4[1];
volatile unsigned char buff_wr5[1];

byte data_tofcap[4] = {0};
byte data_tact[1] = {0}; 
byte data_capstat[3] = {0};
uint8_t data_pos[4];

uint8_t vib0_count;
uint8_t vib1_count;
uint8_t vib2_count;

uint8_t vib0_duty = 0;
uint8_t vib1_duty = 0;
uint8_t vib2_duty = 0;

uint8_t vib0_duty_cnt = 0;
uint8_t vib1_duty_cnt = 0;
uint8_t vib2_duty_cnt = 0;

bool sw_right = false;
bool sw_left  = false;
bool sw_up    = false;
bool sw_down  = false;

int sw_r_on_cnt  = 0;
int sw_r_off_cnt = 0;
int sw_l_on_cnt  = 0;
int sw_l_off_cnt = 0;
int sw_u_on_cnt  = 0;
int sw_u_off_cnt = 0;
int sw_d_on_cnt  = 0;
int sw_d_off_cnt = 0;

volatile bool onoff_recived = false;
uint32_t servo_angle = 2047;
bool servo_enable = false;
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
using namespace ControlTableItem;

void debug_println(char *str) {
  if (!DEBUG) return;
  xSemaphoreTake(semaphoreSerialIO, portMAX_DELAY);
  Serial.println(str);
  xSemaphoreGive(semaphoreSerialIO);
}

void debug_println(int num) {
  if (!DEBUG) return;
  xSemaphoreTake(semaphoreSerialIO, portMAX_DELAY);
  Serial.println(num);
  xSemaphoreGive(semaphoreSerialIO);
}

void mcpISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(semaphoreCanISR, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int task_read_count = 0;

void task_read(void *pvParameters) {
  while(1) {
    xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
    uint8_t irq = mcp2515.getInterrupts();
    // ensure that the interrupt is cleared and will get next mcpISR call
    mcp2515.clearInterrupts();
    xSemaphoreGive(semaphoreCanIO);

    struct can_frame recvMsg;
    // read from RXB0 and RXB1 if any available data
    // then wait for the next interrupt
    if (irq & MCP2515::CANINTF_RX0IF) {
      debug_println("task_read0");
      // frame contains received from RXB0 message
      xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
      mcp2515.readMessage(MCP2515::RXB0, &recvMsg);
      xSemaphoreGive(semaphoreCanIO);
      process_message(recvMsg);
    }
    if (irq & MCP2515::CANINTF_RX1IF) {
      debug_println("task_read1");
      // frame contains received from RXB1 message
      xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
      mcp2515.readMessage(MCP2515::RXB1, &recvMsg);
      xSemaphoreGive(semaphoreCanIO);
      process_message(recvMsg);
    }
    task_read_count++;
    if (task_read_count > 1000)
    {
      task_read_count = 0;
      debug_println("task_read wait");
    }
    // wait for a tick, just in case the interrupt is not fired somehow
    xSemaphoreTake(semaphoreCanISR, 1);
  }
}

void process_message(struct can_frame recvMsg) {
  if(recvMsg.can_id == ADDR_VIB)
  {
    buff_vib[0] = recvMsg.data[0];
    buff_vib[1] = recvMsg.data[1];
    buff_vib[2] = recvMsg.data[2];
      
    vib0_count = (buff_vib[0]!=0) ? buff_vib[0] : vib0_count;
    vib1_count = (buff_vib[1]!=0) ? buff_vib[1] : vib1_count;
    vib2_count = (buff_vib[2]!=0) ? buff_vib[2] : vib2_count;
  }
  else if(recvMsg.can_id == ADDR_SERVO_TGT)
  {
    if(servo_enable)
    {
      buff_tgt[0] = recvMsg.data[0];
      buff_tgt[1] = recvMsg.data[1];
      buff_tgt[2] = recvMsg.data[2];
      buff_tgt[3] = recvMsg.data[3];
        
      servo_angle = ((uint32_t)buff_tgt[3]) << 24 | ((uint32_t)buff_tgt[2]) << 16 | ((uint16_t)buff_tgt[1]) << 8 | ((uint16_t)buff_tgt[0]) << 0;
    }
  }
  else if(recvMsg.can_id == ADDR_SERVO_EN)
  {
    if(servo_enable)
    {
      buff_en[0] = recvMsg.data[0];
      onoff_recived = true;
    }
  }
  else if(recvMsg.can_id == ADDR_CAP_WR1)
  {
    buff_wr1[0] = recvMsg.data[0];
    xSemaphoreTake(semaphoreCap1203IO, portMAX_DELAY);
    cap_sens.setCalibrationStatusReg(buff_wr1[0]);
    xSemaphoreGive(semaphoreCap1203IO);
  }
  else if(recvMsg.can_id == ADDR_CAP_WR2)
  {
    buff_wr2[0] = recvMsg.data[0];
    xSemaphoreTake(semaphoreCap1203IO, portMAX_DELAY);
    cap_sens.setNegativeDeltaCountReg(buff_wr2[0]);
    xSemaphoreGive(semaphoreCap1203IO);
  }
  else if(recvMsg.can_id == ADDR_CAP_WR3)
  {
    buff_wr3[0] = recvMsg.data[0];
    xSemaphoreTake(semaphoreCap1203IO, portMAX_DELAY);
    cap_sens.setSensorInputEnableReg(buff_wr3[0]);
    xSemaphoreGive(semaphoreCap1203IO);
  }
  else if(recvMsg.can_id == ADDR_CAP_WR4)
  {
    buff_wr4[0] = recvMsg.data[0];
    xSemaphoreTake(semaphoreCap1203IO, portMAX_DELAY);
    cap_sens.setConfigurationReg(buff_wr4[0]);
    xSemaphoreGive(semaphoreCap1203IO);
  }
  else if(recvMsg.can_id == ADDR_CAP_WR5)
  {
    buff_wr5[0] = recvMsg.data[0];
    xSemaphoreTake(semaphoreCap1203IO, portMAX_DELAY);
    cap_sens.setConfiguration2Reg(buff_wr5[0]);
    xSemaphoreGive(semaphoreCap1203IO);
  }
}

void task2ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 2;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    bool sw_r = digitalRead(SW_RIGHT);
    bool sw_l = digitalRead(SW_LEFT);
    bool sw_u = digitalRead(SW_UP);
    bool sw_d = digitalRead(SW_DOWN);

    if(sw_r == HIGH)
    {
      sw_r_on_cnt++;
      sw_r_off_cnt = 0;
      if(sw_r_on_cnt > CHATA_THRESHOLD)
      {
        sw_right = true;
        sw_r_on_cnt = 0;
      }
    }
    else
    {
      sw_r_off_cnt++;
      sw_r_on_cnt = 0;
      if(sw_r_off_cnt > CHATA_THRESHOLD)
      {
        sw_right = false;
        sw_r_off_cnt = 0;
      }
    }

    if(sw_l == HIGH)
    {
      sw_l_on_cnt++;
      sw_l_off_cnt = 0;
      if(sw_l_on_cnt > CHATA_THRESHOLD)
      {
        sw_left = true;
        sw_l_on_cnt = 0;
      }
    }
    else
    {
      sw_l_off_cnt++;
      sw_l_on_cnt = 0;
      if(sw_l_off_cnt > CHATA_THRESHOLD)
      {
        sw_left = false;
        sw_l_off_cnt = 0;
      }
    }

    if(sw_u == HIGH)
    {
      sw_u_on_cnt++;
      sw_u_off_cnt = 0;
      if(sw_u_on_cnt > CHATA_THRESHOLD)
      {
        sw_up = true;
        sw_u_on_cnt = 0;
      }
    }
    else
    {
      sw_u_off_cnt++;
      sw_u_on_cnt = 0;
      if(sw_u_off_cnt > CHATA_THRESHOLD)
      {
        sw_up = false;
        sw_u_off_cnt = 0;
      }
    }

    if(sw_d == HIGH)
    {
      sw_d_on_cnt++;
      sw_d_off_cnt = 0;
      if(sw_d_on_cnt > CHATA_THRESHOLD)
      {
        sw_down = true;
        sw_d_on_cnt = 0;
      }
    }
    else
    {
      sw_d_off_cnt++;
      sw_d_on_cnt = 0;
      if(sw_d_off_cnt > CHATA_THRESHOLD)
      {
        sw_down = false;
        sw_d_off_cnt = 0;
      }
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
    struct can_frame sendMsg;
    uint16_t tof;
    tof = lox.readRangeResult();
    sendMsg.can_id = ADDR_TOFCAP;
    sendMsg.can_dlc = 4;
    sendMsg.data[0] = (tof & 0x00ff) >> 0;
    sendMsg.data[1] = (tof & 0xff00) >> 8;
    xSemaphoreTake(semaphoreCap1203IO, portMAX_DELAY);
    sendMsg.data[2] = cap_sens.getSensorInput1DeltaCountReg();
    sendMsg.data[3] = cap_sens.getSensorInputStatusReg();
    cap_sens.setMainControlReg(false, false, false);
    xSemaphoreGive(semaphoreCap1203IO);
    xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
    mcp2515.sendMessage(&sendMsg);
    xSemaphoreGive(semaphoreCanIO);
    delayMicroseconds(500);

    sendMsg.can_id = ADDR_CAP_STAT;
    sendMsg.can_dlc = 3;
    xSemaphoreTake(semaphoreCap1203IO, portMAX_DELAY);
    sendMsg.data[0] = cap_sens.getGeneralStatusReg();
    sendMsg.data[1] = cap_sens.getNoiseFlagStatsReg();
    sendMsg.data[2] = cap_sens.getCalibrationStatusReg();
    xSemaphoreGive(semaphoreCap1203IO);
    xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
    mcp2515.sendMessage(&sendMsg);
    xSemaphoreGive(semaphoreCanIO);
    delayMicroseconds(500);
    
    data_tact[0] = sw_right << 0
                 | sw_left  << 1
                 | sw_up    << 2
                 | sw_down  << 3;
    data_tact[0] = (data_tact[0])&0x0f;
    
    sendMsg.can_id = ADDR_TACT;
    sendMsg.can_dlc = 1;
    sendMsg.data[0] = data_tact[0];
    xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
    mcp2515.sendMessage(&sendMsg);
    xSemaphoreGive(semaphoreCanIO);
    delayMicroseconds(500);

    debug_println(tof);

    if(servo_enable)
    {
      if(onoff_recived == true)
      {
        if(buff_en[0] == 0x00)
        {
          dxl.torqueOff(DXL_ID);
        }
        if(buff_en[0] == 0x01)
        {
          dxl.torqueOn(DXL_ID); 
        }
        onoff_recived == false;
      }
      
      dxl.setGoalPosition(DXL_ID, servo_angle);
      delayMicroseconds(500);
      uint32_t angle = dxl.getPresentPosition(DXL_ID);
      
      sendMsg.can_id = ADDR_SERVO_POS;
      sendMsg.can_dlc = 4;
      sendMsg.data[0] = (angle & 0x000000ff) >> 0;
      sendMsg.data[1] = (angle & 0x0000ff00) >> 8;
      sendMsg.data[2] = (angle & 0x00ff0000) >> 16;
      sendMsg.data[3] = (angle & 0xff000000) >> 24;
      xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
      mcp2515.sendMessage(&sendMsg);
      xSemaphoreGive(semaphoreCanIO);
    }
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

  pinMode(SW_RIGHT, INPUT);
  pinMode(SW_LEFT, INPUT);
  pinMode(SW_UP, INPUT);
  pinMode(SW_DOWN, INPUT);
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

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_20MHZ);
  
  mcp2515.setFilterMask(MCP2515::MASK0, false, CAN_MASK0);
  mcp2515.setFilter(MCP2515::RXF0, false, CAN_FILTER0);

  mcp2515.setFilterMask(MCP2515::MASK1, false, CAN_MASK1);
  mcp2515.setFilter(MCP2515::RXF2, false, CAN_FILTER1);

  mcp2515.setNormalMode();
  pinMode(SPI_INT, INPUT);

  semaphoreCanISR = xSemaphoreCreateBinary();
  semaphoreSerialIO = xSemaphoreCreateMutex();
  semaphoreCanIO = xSemaphoreCreateMutex();
  semaphoreCap1203IO = xSemaphoreCreateMutex();

  debug_println("CAN OK");
  attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);

  xTaskCreate(task2ms,  "task2ms",  configMINIMAL_STACK_SIZE, NULL, 5,  NULL);
  xTaskCreate(task10ms, "task10ms", configMINIMAL_STACK_SIZE, NULL, 9,  NULL);
  xTaskCreate(task20ms, "task20ms", configMINIMAL_STACK_SIZE, NULL, 9,  NULL);
  xTaskCreate(task_read,"task_read",configMINIMAL_STACK_SIZE, NULL, 10, NULL);

  vTaskStartScheduler(); 
}

void loop()
{
  
  delay(1);
}
