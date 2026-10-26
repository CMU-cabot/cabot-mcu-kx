#include "firmware_version.h"
#include <FirmwareService.h>
#include "can_transport.h"
#include "handle_protocol.h"
#include "handle_servo.h"
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

CanTransport can_transport(mcp2515, semaphoreCanIO);
HandleServo handle_servo(dxl);
HandleProtocol handle_protocol(can_transport, handle_servo, cap_sens,
                               semaphoreCap1203IO, vib0_count, vib1_count, vib2_count);

static bool send_firmware_response(void*, uint32_t id, const uint8_t* data, uint8_t length) {
  can_frame frame = {};
  frame.can_id = id;
  frame.can_dlc = length;
  memcpy(frame.data, data, length);
  return can_transport.send(frame);
}

static bool can_enter_update(void*) { return true; }

static void enter_bootloader(void*) {
  delay(2); // preserve ACK transmission interval before resetting this board
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  RTC->BKP0R = 0x314c4243UL; // bootloader's CBL1 marker
  __DSB();
  NVIC_SystemReset();
}

static const char app_version[] = HANDLE_FIRMWARE_VERSION;
static_assert(sizeof(app_version) > 1 && sizeof(app_version) <= 65, "version must be 1-64 bytes");
FirmwareService firmware_service({CAN_ID_CONTROL, CAN_ID_RESPONSE, app_version, nullptr,
                                  send_firmware_response, can_enter_update, enter_bootloader});

void process_message(const can_frame& frame) {
  if (!firmware_service.handle(frame.can_id, frame.can_dlc, frame.data)) {
    handle_protocol.handle(frame);
  }
}

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
    uint8_t irq = can_transport.interrupts();

    struct can_frame recvMsg;
    // read from RXB0 and RXB1 if any available data
    // then wait for the next interrupt
    if (irq & MCP2515::CANINTF_RX0IF) {
      debug_println("task_read0");
      // frame contains received from RXB0 message
      bool received = can_transport.receive(MCP2515::RXB0, recvMsg);
      if (received) process_message(recvMsg);
    }
    if (irq & MCP2515::CANINTF_RX1IF) {
      debug_println("task_read1");
      // frame contains received from RXB1 message
      bool received = can_transport.receive(MCP2515::RXB1, recvMsg);
      if (received) process_message(recvMsg);
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

int task2ms_count = 0;

void task2ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 2;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    task2ms_count += xFrequency;
    if (task2ms_count > 1000) {
      task2ms_count = 0;
      debug_println("task2ms");
    }
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

int task10ms_count = 0;

void task10ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    task10ms_count += xFrequency;
    if (task10ms_count > 1000) {
      task10ms_count = 0;
      debug_println("task10ms");
    }
    if(vib0_count>0){digitalWrite(VIB_0, HIGH);vib0_count--;}
    else{digitalWrite(VIB_0, LOW);}
    if(vib1_count>0){digitalWrite(VIB_1, HIGH);vib1_count--;}
    else{digitalWrite(VIB_1, LOW);}
    if(vib2_count>0){digitalWrite(VIB_2, HIGH);vib2_count--;}
    else{digitalWrite(VIB_2, LOW);}

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

int task20ms_count = 0;

void task20ms(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 20;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    task20ms_count += xFrequency;
    if (task20ms_count > 1000) {
      task20ms_count = 0;
      debug_println("task20ms");
    }
    uint16_t tof;
    tof = lox.readRangeResult();
    xSemaphoreTake(semaphoreCap1203IO, portMAX_DELAY);
    uint8_t delta = cap_sens.getSensorInput1DeltaCountReg();
    uint8_t touch_status = cap_sens.getSensorInputStatusReg();
    cap_sens.setMainControlReg(false, false, false);
    xSemaphoreGive(semaphoreCap1203IO);
    handle_protocol.send_tof_touch(tof, delta, touch_status);
    delayMicroseconds(500);

    xSemaphoreTake(semaphoreCap1203IO, portMAX_DELAY);
    uint8_t general = cap_sens.getGeneralStatusReg();
    uint8_t noise = cap_sens.getNoiseFlagStatsReg();
    uint8_t calibration = cap_sens.getCalibrationStatusReg();
    xSemaphoreGive(semaphoreCap1203IO);
    handle_protocol.send_touch_status(general, noise, calibration);
    delayMicroseconds(500);
    
    data_tact[0] = sw_right << 0
                 | sw_left  << 1
                 | sw_up    << 2
                 | sw_down  << 3;
    data_tact[0] = (data_tact[0])&0x0f;
    
    handle_protocol.send_buttons(data_tact[0]);
    delayMicroseconds(500);

    // debug_println(tof);

    uint32_t angle;
    if (handle_servo.update(angle)) {
      handle_protocol.send_position(angle);
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
  

  while(!lox.begin())
  {
    delay(10);
  }
  lox.startRangeContinuous(8);

  handle_servo.initialize();

  cap_sens.begin();
  cap_sens.setSensitivityControlReg(BASE_DEF, DELTA_4X);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_20MHZ);
  
  mcp2515.setFilterMask(MCP2515::MASK0, false, CAN_MASK0);
  mcp2515.setFilter(MCP2515::RXF0, false, CAN_FILTER0);

  mcp2515.setFilterMask(MCP2515::MASK1, false, CAN_MASK1);
  mcp2515.setFilter(MCP2515::RXF2, false, CAN_FILTER1);
  mcp2515.setFilter(MCP2515::RXF3, false, CAN_ID_CONTROL);

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
