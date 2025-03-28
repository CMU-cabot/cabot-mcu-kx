#include <STM32FreeRTOS.h>
#include <mcp2515.h>
#include <SPI.h>
#include <Wire.h>

#define SW_0          PA0
#define SW_1          PA1
#define SW_EW         PA12
#define LED_0         PA3
#define LED_1         PA4
#define G_24V         PA6
#define G_12V_PC      PA7
#define G_12V_D455_1  PA11
#define G_12V_D455_2  PA14
#define G_12V_D455_3  PB0
#define G_5V_MCU      PA8
#define PWM_FAN       PB1
//On the prototype board, "PC_ENABLE" is assigned to the resetSW(SW2).
//To simulate a PC shutdown, press resetSW.
#define PC_ENABLE     PA2
#define MUX_RST       PA5

#define SPI_MOSI      PB5
#define SPI_MISO      PB4
#define SPI_SCK       PB3
#define SPI_CS_PIN    PA15
#define SPI_INT       PA13

#define I2C_SDA       PB7
#define I2C_SCL       PB6

#define STM32_TXD     PA9
#define STM32_RXD     PA10

#define SMBUS_BATT    0x0b    // Battry adderess
#define SMBUS_MUX     0x70    // Multiplexer address

#define SMBUS_VOLTAGE 0x09    // SMBUS battery voltage address
#define SMBUS_CULLENT 0x0a    // SMBUS battery current address
#define SMBUS_CHARGE  0x0d    // SMBUS battery remaining capacity address
#define SMBUS_TEMP    0x08    // SMBUS battery temperature address
#define SMBUS_SERIAL  0x1c    // SMBUS battery serial No. address

//for mass production model
#define ADDR_EMS      0x100   // 0b0 010 0000 000  Emergency stop
#define ADDR_STAT     0x101   // 0b0 010 0000 001  Power board status
#define ADDR_IND      0x102   // 0b0 010 0000 010  Power board indicator
#define ADDR_SEQ      0x103   // 0b0 010 0000 011  Power board sequence status
#define ADDR_ODRIVE   0x108   // 0b0 010 0001 000  +24V_Odrive control
#define ADDR_PC       0x109   // 0b0 010 0001 001  +12_PC control(poweron/shutdown sequence control)
#define ADDR_D455_1   0x10a   // 0b0 010 0001 010  +12V_D455_1 control
#define ADDR_D455_2   0x10b   // 0b0 010 0001 011  +12V_D455_2 control
#define ADDR_D455_3   0x10c   // 0b0 010 0001 100  +12V_D455_3 control
#define ADDR_MCU      0x10d   // 0b0 010 0001 101  +5V_MCU control
#define ADDR_PWM      0x10e   // 0b0 010 0001 110  FAN pwm control
#define ADDR_BAT_1    0x518   // 0b1 010 0011 000  Battery1 status
#define ADDR_BAT_2    0x519   // 0b1 010 0011 001  Battery2 status
#define ADDR_BAT_3    0x51a   // 0b1 010 0011 010  Battery3 status
#define ADDR_BAT_4    0x51b   // 0b1 010 0011 011  Battery4 status
#define ADDR_BAT_SN   0x520   // 0b1 010 0100 000  Battery serial No.
#define CAN_FILTER    0x0108  // 0b0 010 0001 000 filter for ADDR_ODRIVE ~ ADDR_PWM (major=2, minor=1)
#define CAN_MASK      0x07f8  // 0b1 111 1111 000 mask by priority, major, minor
#define SHUTDOWN_PC   60000   // pc shutdown wait time[ms](Not used)
#define SHUTDOWN_FRC  120000  // force shutdown time[ms]

#ifndef DEBUG                 // you can set DEBUG=1 to print debug message via Serial
#define DEBUG 0
#endif

MCP2515 mcp2515(SPI_CS_PIN);

volatile SemaphoreHandle_t semaphoreSequence;
volatile SemaphoreHandle_t semaphoreCanISR;
volatile SemaphoreHandle_t semaphoreSerialIO;
volatile SemaphoreHandle_t semaphoreCanIO;

volatile bool flag_power_on   = false;
volatile bool flag_shutdown   = false;
volatile bool flag_emergency  = false;
volatile bool flag_sequencing = false;

volatile unsigned char buff_odrive[1];
volatile unsigned char buff_pc[1];
volatile unsigned char buff_d455_1[1];
volatile unsigned char buff_d455_2[1];
volatile unsigned char buff_d455_3[1];
volatile unsigned char buff_mcu[1];
volatile unsigned char buff_pwm[1];

int poweron_cnt   = 0;
int shutdown_cnt  = 0;

//shutdown sequence count
int32_t sequence_cnt  = 0;

//debug
int reset_count = 0;

void debug_print(char *str) {
  if (!DEBUG) return;
  xSemaphoreTake(semaphoreSerialIO, portMAX_DELAY);
  Serial.print(str);
  xSemaphoreGive(semaphoreSerialIO);
}

void debug_println(char *str) {
  if (!DEBUG) return;
  xSemaphoreTake(semaphoreSerialIO, portMAX_DELAY);
  Serial.println(str);
  xSemaphoreGive(semaphoreSerialIO);
}

void debug_print(int num) {
  if (!DEBUG) return;
  xSemaphoreTake(semaphoreSerialIO, portMAX_DELAY);
  Serial.print(num);
  xSemaphoreGive(semaphoreSerialIO);
}

void debug_println(int num) {
  if (!DEBUG) return;
  xSemaphoreTake(semaphoreSerialIO, portMAX_DELAY);
  Serial.println(num);
  xSemaphoreGive(semaphoreSerialIO);
}

void setChannel(uint8_t ch)
{
  Wire.beginTransmission(SMBUS_MUX);
  switch(ch) {
      case 0:
        Wire.write(0x01);
        break;
      case 1:
        Wire.write(0x02);
        break;
      case 2:
        Wire.write(0x04);
        break;
      case 3:
        Wire.write(0x08);
        break;
    }
    Wire.endTransmission();
    delayMicroseconds(5);
}

uint16_t readWord(uint8_t addr)
{
  uint16_t ret = 0;
  Wire.beginTransmission(SMBUS_BATT);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(SMBUS_BATT, 2, true);
  int first = Wire.read();
  int second = Wire.read();
  if (first < 0 || second < 0) {
    return 0xFFFF;  // least possible value
  }
  ret = (0xFF & first) | (0xFF & second) << 8;
  delayMicroseconds(5);
  return ret;
}

int16_t readWordSigned(uint8_t addr)
{
  int16_t ret = 0;
  Wire.beginTransmission(SMBUS_BATT);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(SMBUS_BATT, 2, true);
  int first = Wire.read();
  int second = Wire.read();
  if (first < 0 || second < 0) {
    return 0x7FFF;  // least possible value
  }
  ret = (0xFF & first) | (0xFF & second) << 8;
  delayMicroseconds(5);
  return ret;
}

int checkSlave(uint8_t addr)
{
  int err = 0;
  Wire.beginTransmission(addr);
  err = Wire.endTransmission( );
  if(err != 0)
  {
    digitalWrite(MUX_RST, HIGH);
    delayMicroseconds(500);
    digitalWrite(MUX_RST, LOW);
    delayMicroseconds(500);
    digitalWrite(MUX_RST, HIGH);
    delayMicroseconds(500);
    
    Wire.end();
    delayMicroseconds(500);
    Wire.begin();
    delayMicroseconds(500);
    reset_count++;
  }
  return err;
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
  // process recvMsg
  if(recvMsg.can_id == ADDR_ODRIVE)
  {
    buff_odrive[0] = recvMsg.data[0];
    (buff_odrive[0]&0x01 == 0x01) ? (digitalWrite(G_24V, HIGH)) : (digitalWrite(G_24V, LOW));
    if(buff_odrive[0]&0x01 == 0x01 && flag_emergency == true)
    {
      flag_emergency = false;
    }
  }
  else if(recvMsg.can_id == ADDR_PC)
  {
    buff_pc[0] = recvMsg.data[0];
    if(buff_pc[0] == 0x00)
    {
      task_shutdown();
    }
    if(buff_pc[0] == 0x01)
    {
      task_poweron();
    }
  }
  else if(recvMsg.can_id == ADDR_D455_1)
  {
    buff_d455_1[0] = recvMsg.data[0];
    (buff_d455_1[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_1, HIGH)) : (digitalWrite(G_12V_D455_1, LOW));
  }
  else if(recvMsg.can_id == ADDR_D455_2)
  {
    buff_d455_2[0] = recvMsg.data[0];
    (buff_d455_2[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_2, HIGH)) : (digitalWrite(G_12V_D455_2, LOW));
  }
  else if(recvMsg.can_id == ADDR_D455_3)
  {
    buff_d455_3[0] = recvMsg.data[0];
    (buff_d455_3[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_3, HIGH)) : (digitalWrite(G_12V_D455_3, LOW));
  }
  else if(recvMsg.can_id == ADDR_MCU)
  {
    buff_mcu[0] = recvMsg.data[0];
    (buff_mcu[0]&0x01 == 0x01) ? (digitalWrite(G_5V_MCU, HIGH)) : (digitalWrite(G_5V_MCU, LOW));
  }
  else if(recvMsg.can_id == ADDR_PWM)
  {
    buff_pwm[0] = recvMsg.data[0];
    analogWrite(PWM_FAN, 255 - buff_pwm[0]);
  }
}

void task_emergency()
{
  if(flag_emergency == false)
  {
    debug_println("emergency");
    digitalWrite(G_24V, LOW);
    flag_emergency = true;
    struct can_frame sendMsg;
    sendMsg.can_id = ADDR_EMS;
    sendMsg.can_dlc = 1;
    sendMsg.data[0] = 0x01;

    xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
    mcp2515.sendMessage(&sendMsg);
    xSemaphoreGive(semaphoreCanIO);
  }
}

void task_poweron()
{
  if(flag_sequencing == false && flag_power_on == false)
  {
    debug_println("poweron");
    flag_sequencing = true;
    digitalWrite(LED_0, HIGH);
    flag_power_on = true;
    xSemaphoreGive(semaphoreSequence);
  }
}

void task_shutdown()
{
  if(flag_sequencing == false && flag_power_on == true)
  {
    debug_println("shutdown");
    flag_sequencing = true;
    digitalWrite(LED_1, HIGH);
    flag_shutdown  = true;
    xSemaphoreGive(semaphoreSequence);
  }
}

//poweron/shutdown sequence task
void task_sequence(void *pvParameters)
{
  while(1)
  {
    if (xSemaphoreTake(semaphoreSequence, portMAX_DELAY) != pdTRUE) {
      // this may not happen
      continue;
    }
    debug_println("sequence");

    struct can_frame sendMsg;
    sendMsg.can_id = ADDR_SEQ;
    sendMsg.can_dlc = 1;
    
    if(flag_power_on == true && flag_shutdown == false)
    {
      sendMsg.data[0] = 0x01;
      xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
      mcp2515.sendMessage(&sendMsg);
      xSemaphoreGive(semaphoreCanIO);

      digitalWrite(G_24V, HIGH);
      vTaskDelay(100);
      digitalWrite(G_12V_PC, HIGH);
      vTaskDelay(100);
      digitalWrite(G_12V_D455_1, HIGH);
      vTaskDelay(100);
      digitalWrite(G_12V_D455_2, HIGH);
      vTaskDelay(100);
      digitalWrite(G_12V_D455_3, HIGH);
      vTaskDelay(100);
      digitalWrite(G_5V_MCU, HIGH);
      vTaskDelay(100);
      analogWrite(PWM_FAN, 255);
      vTaskDelay(100);

      flag_power_on  = true;
      flag_shutdown  = false;
    }
    else if(flag_power_on == true && flag_shutdown == true)
    {
      sendMsg.data[0] = 0x00;
      xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
      mcp2515.sendMessage(&sendMsg);
      xSemaphoreGive(semaphoreCanIO);
      
      //shutdown sequence
      while(sequence_cnt<SHUTDOWN_FRC)
      {
        //pc shutdown detect
        if(digitalRead(PC_ENABLE) == LOW)
        {
          vTaskDelay(5000);
          break;
        }

        vTaskDelay(1);
        sequence_cnt++;
      }
      
      digitalWrite(LED_0, LOW);
      digitalWrite(G_24V, LOW);
      digitalWrite(G_12V_PC, LOW);
      digitalWrite(G_12V_D455_1, LOW);
      digitalWrite(G_12V_D455_2, LOW);
      digitalWrite(G_12V_D455_3, LOW);
      digitalWrite(G_5V_MCU, LOW);
      analogWrite(PWM_FAN, 255);
      digitalWrite(LED_1, LOW);

      sequence_cnt = 0;
      flag_power_on  = false;
      flag_shutdown  = false;
      flag_emergency = false;
    }
    
    flag_sequencing = false;
    vTaskDelay(1);
  }
}

void task_send(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 500;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    debug_println("send");
    struct can_frame sendMsg;
    sendMsg.can_id = ADDR_IND;
    sendMsg.can_dlc = 1;
    sendMsg.data[0] = digitalRead(LED_0) << 0
                    | digitalRead(LED_1) << 1;

    xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
    mcp2515.sendMessage(&sendMsg);
    xSemaphoreGive(semaphoreCanIO);
    vTaskDelay(1);

    uint16_t voltage  = 0;
    int16_t current  = 0;
    uint16_t charge   = 0;
    uint16_t temp     = 0;
    uint16_t sn       = 0;
    byte battery_sn[8]= {0};
    
    for(int i=0;i<4;i++)
    {
      setChannel(i);
      
      int err = 0;
      err = checkSlave(SMBUS_MUX);
      vTaskDelay(5);
      // debug_print("mux  ");
      // debug_print(i);
      // debug_print(" ");
      // debug_println(err);
      
      err = checkSlave(SMBUS_BATT);
      vTaskDelay(5);
      // debug_print("batt ");
      // debug_print(i);
      // debug_print(" ");
      // debug_println(err);

      // debug_print("reset ");
      // debug_println(reset_count);

      voltage = readWord(SMBUS_VOLTAGE); //Voltage[mV]
      current = readWordSigned(SMBUS_CULLENT); //Current[mA]
      charge  = readWord(SMBUS_CHARGE);  //RelativeStateOfCharge[%]
      temp    = readWord(SMBUS_TEMP);    //Temperature[0.1k]
      sn      = readWord(SMBUS_SERIAL);  //SerialNumber

      sendMsg.can_id = ADDR_BAT_1+i;
      sendMsg.can_dlc = 8;
      sendMsg.data[0] = (voltage&0x00ff)  >> 0;
      sendMsg.data[1] = (voltage&0xff00)  >> 8;
      sendMsg.data[2] = (current&0x00ff)  >> 0;
      sendMsg.data[3] = (current&0xff00)  >> 8;
      sendMsg.data[4] = (charge&0x00ff)   >> 0;
      sendMsg.data[5] = (charge&0xff00)   >> 8;
      sendMsg.data[6] = (temp&0x00ff)     >> 0;
      sendMsg.data[7] = (temp&0xff00)     >> 8;

      battery_sn[i*2]   = (sn&0x00ff) >> 0;
      battery_sn[i*2+1] = (sn&0xff00) >> 8;

      xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
      mcp2515.sendMessage(&sendMsg);
      xSemaphoreGive(semaphoreCanIO);
      vTaskDelay(1);
      
    }

    sendMsg.can_id = ADDR_STAT;
    sendMsg.can_dlc = 1;
    sendMsg.data[0] = digitalRead(G_5V_MCU)     << 0
                    | digitalRead(G_12V_D455_3) << 1
                    | digitalRead(G_12V_D455_2) << 2
                    | digitalRead(G_12V_D455_1) << 3
                    | digitalRead(G_12V_PC)     << 4
                    | digitalRead(G_24V)        << 5;

    xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
    mcp2515.sendMessage(&sendMsg);
    xSemaphoreGive(semaphoreCanIO);
    vTaskDelay(1);

    sendMsg.can_id = ADDR_BAT_SN;
    sendMsg.can_dlc = 8;
    sendMsg.data[0] = battery_sn[0];
    sendMsg.data[1] = battery_sn[1];
    sendMsg.data[2] = battery_sn[2];
    sendMsg.data[3] = battery_sn[3];
    sendMsg.data[4] = battery_sn[4];
    sendMsg.data[5] = battery_sn[5];
    sendMsg.data[6] = battery_sn[6];
    sendMsg.data[7] = battery_sn[7];

    xSemaphoreTake(semaphoreCanIO, portMAX_DELAY);
    mcp2515.sendMessage(&sendMsg);
    xSemaphoreGive(semaphoreCanIO);
    
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

  Serial.setRx(STM32_RXD);
  Serial.setTx(STM32_TXD);
  Serial.begin(115200);

  pinMode(SW_0, INPUT);
  pinMode(SW_1, INPUT);
  pinMode(SW_EW, INPUT);
  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(G_24V, OUTPUT);
  pinMode(G_12V_PC, OUTPUT);
  pinMode(G_12V_D455_1, OUTPUT);
  pinMode(G_12V_D455_2, OUTPUT);
  pinMode(G_12V_D455_3, OUTPUT);
  pinMode(G_5V_MCU, OUTPUT);
  pinMode(PWM_FAN, OUTPUT);
  pinMode(PC_ENABLE,INPUT);
  pinMode(MUX_RST, OUTPUT);

  //SMBus multiplexer reset
  digitalWrite(MUX_RST, HIGH);
  delay(10);
  digitalWrite(MUX_RST, LOW);
  delay(10);
  digitalWrite(MUX_RST, HIGH);
  delay(10);

  digitalWrite(LED_0, LOW);
  digitalWrite(LED_1, LOW);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_20MHZ);
  
  mcp2515.setFilterMask(MCP2515::MASK0, false, CAN_MASK);
  mcp2515.setFilter(MCP2515::RXF0, false, CAN_FILTER);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x07ff);  // filter only ID=0

  mcp2515.setNormalMode();
  pinMode(SPI_INT, INPUT);

  semaphoreSequence = xSemaphoreCreateBinary();
  semaphoreCanISR = xSemaphoreCreateBinary();
  semaphoreSerialIO = xSemaphoreCreateMutex();
  semaphoreCanIO = xSemaphoreCreateMutex();

  xTaskCreate(task_sequence,  "task_sequence",  configMINIMAL_STACK_SIZE, NULL, 5,  NULL);
  xTaskCreate(task_send,      "task_send",      configMINIMAL_STACK_SIZE, NULL, 9,  NULL);
  xTaskCreate(task_read,      "task_read",      configMINIMAL_STACK_SIZE, NULL, 10, NULL);

  attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR,       FALLING);
  vTaskStartScheduler(); 
}

int loop_count = 0;

void loop()
{
  loop_count++;
  if (loop_count > 1000) {
    loop_count = 0;
    debug_println("loop");
  }

  if(digitalRead(SW_EW) == LOW && flag_emergency == false && flag_power_on == true)
  {
    task_emergency();
  }

  if(digitalRead(SW_0) == LOW && flag_sequencing == false && flag_power_on == false)
  {
    if(poweron_cnt >= 500)
    {
      poweron_cnt = 0;
      task_poweron();
    }
    else
    {
      poweron_cnt++;
    }
  }
  else if(digitalRead(SW_0) != LOW)
  {
    poweron_cnt = 0;
  }
  
  if(digitalRead(SW_1) == LOW && flag_sequencing == false && flag_power_on == true)
  {
    if(shutdown_cnt >= 2000)
    {
      shutdown_cnt = 0;
      task_shutdown();
    }
    else
    {
      shutdown_cnt++;
    }
  }
  else if(digitalRead(SW_1) != LOW)
  {
    shutdown_cnt = 0;
  }
  
  delay(1);
}
