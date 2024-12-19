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

#define SMBUS_BATT    0x0b
#define SMBUS_MUX     0x70

#define SMBUS_VOLTAGE 0x09
#define SMBUS_CULLENT 0x0a
#define SMBUS_CHARGE  0x0d 
#define SMBUS_TEMP    0x08
#define SMBUS_SERIAL  0x1c

//for mass production model
#define ADDR_EMS      0x100
#define ADDR_STAT     0x101
#define ADDR_IND      0x102
#define ADDR_SEQ      0x103
#define ADDR_ODRIVE   0x108
#define ADDR_PC       0x109
#define ADDR_D455_1   0x10a
#define ADDR_D455_2   0x10b
#define ADDR_D455_3   0x10c
#define ADDR_MCU      0x10d
#define ADDR_PWM      0x10e
#define ADDR_BAT_1    0x518
#define ADDR_BAT_2    0x519
#define ADDR_BAT_3    0x51a
#define ADDR_BAT_4    0x51b
#define ADDR_BAT_SN   0x520
#define CAN_FILTER    0x01080000
#define CAN_MASK      0x07f80000
#define SHUTDOWN_PC   60000   //pc shutdown wait time[ms]
#define SHUTDOWN_FRC  120000  //force shutdown time[ms]

MCP2515 mcp2515(SPI_CS_PIN);

volatile SemaphoreHandle_t semaphoreSequence;

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
  ret = Wire.read();
  ret |= Wire.read()<<8;
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

void mcpISR(){
  Serial.println("isr");
  struct can_frame recvMsg;

  if(mcp2515.readMessage(&recvMsg) == MCP2515::ERROR_OK)
  {
    if(recvMsg.can_id == ADDR_ODRIVE)
    {
      buff_odrive[0] = recvMsg.data[0];
      (buff_odrive[0]&0x01 == 0x01) ? (digitalWrite(G_24V, HIGH)) : (digitalWrite(G_24V, LOW));
      if(buff_odrive[0]&0x01 == 0x01 && flag_emergency == true)
      {
        flag_emergency = false;
      }
    }
    if(recvMsg.can_id == ADDR_PC)
    {
      buff_pc[0] = recvMsg.data[0];
      if(buff_pc[0] == 0x00)
      {
        shutdownISR();
      }
      if(buff_pc[0] == 0x01)
      {
        poweronISR();
      }
    }
    if(recvMsg.can_id == ADDR_D455_1)
    {
      buff_d455_1[0] = recvMsg.data[0];
      (buff_d455_1[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_1, HIGH)) : (digitalWrite(G_12V_D455_1, LOW));
    }
    if(recvMsg.can_id == ADDR_D455_2)
    {
      buff_d455_2[0] = recvMsg.data[0];
      (buff_d455_2[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_2, HIGH)) : (digitalWrite(G_12V_D455_2, LOW));
    }
    if(recvMsg.can_id == ADDR_D455_3)
    {
      buff_d455_3[0] = recvMsg.data[0];
      (buff_d455_3[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_3, HIGH)) : (digitalWrite(G_12V_D455_3, LOW));
    }
    if(recvMsg.can_id == ADDR_MCU)
    {
      buff_mcu[0] = recvMsg.data[0];
      (buff_mcu[0]&0x01 == 0x01) ? (digitalWrite(G_5V_MCU, HIGH)) : (digitalWrite(G_5V_MCU, LOW));
    }
    if(recvMsg.can_id == ADDR_PWM)
    {
      buff_pwm[0] = recvMsg.data[0];
      analogWrite(PWM_FAN, 255 - buff_pwm[0]);
    }
  }
  mcp2515.clearInterrupts();  
}

void emergencyISR()
{
  if(flag_emergency == false)
  {
    Serial.println("emergency");
    taskENTER_CRITICAL();
    digitalWrite(G_24V, LOW);
    flag_emergency = true;
    struct can_frame sendMsg;
    sendMsg.can_id = ADDR_EMS;
    sendMsg.can_dlc = 1;
    sendMsg.data[0] = 0x01;
    mcp2515.sendMessage(&sendMsg);
    taskEXIT_CRITICAL();
  }
}

void poweronISR()
{
  if(flag_sequencing == false && flag_power_on == false)
  {
    Serial.println("poweron");
    flag_sequencing = true;
    digitalWrite(LED_0, HIGH);
    flag_power_on = true;
    xSemaphoreGiveFromISR(semaphoreSequence, NULL);
  }
}

void shutdownISR()
{
  if(flag_sequencing == false && flag_power_on == true)
  {
    Serial.println("shutdown");
    flag_sequencing = true;
    digitalWrite(LED_1, HIGH);
    flag_shutdown  = true;
    xSemaphoreGiveFromISR(semaphoreSequence, NULL);
  }
}

//poweron/shutdown sequence task
void task_sequence(void *pvParameters)
{
  while(1)
  {
    xSemaphoreTake(semaphoreSequence, portMAX_DELAY);
    Serial.println("sequence");
    struct can_frame sendMsg;
    sendMsg.can_id = ADDR_SEQ;
    sendMsg.can_dlc = 1;
    
    if(flag_power_on == true && flag_shutdown == false)
    {
      sendMsg.data[0] = 0x01;
      mcp2515.sendMessage(&sendMsg);
      
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
      mcp2515.sendMessage(&sendMsg);
      
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
    //taskENTER_CRITICAL();
    Serial.println("send");
    struct can_frame sendMsg;
    sendMsg.can_id = ADDR_IND;
    sendMsg.can_dlc = 1;
    sendMsg.data[0] = digitalRead(LED_0) << 0
                    | digitalRead(LED_1) << 1;
    detachInterrupt(digitalPinToInterrupt(SPI_INT));
    mcp2515.sendMessage(&sendMsg);
    attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);
    if(digitalRead(SPI_INT) == LOW){mcpISR();}
    vTaskDelay(1);
    //delayMicroseconds(500);

    uint16_t voltage  = 0;
    uint16_t current  = 0;
    uint16_t charge   = 0;
    uint16_t temp     = 0;
    uint16_t sn       = 0;
    byte battery_sn[8]= {0};
    
    for(int i=0;i<4;i++)
    {
      setChannel(i);
      
      int err = 0;
      err = checkSlave(SMBUS_MUX);
      Serial.print("mux  ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(err);
      
      err = checkSlave(SMBUS_BATT);
      Serial.print("batt ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(err);

      Serial.print("reset ");
      Serial.println(reset_count);

      voltage = readWord(SMBUS_VOLTAGE); //Voltage[mV]
      current = readWord(SMBUS_CULLENT); //Current[mA]
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

      detachInterrupt(digitalPinToInterrupt(SPI_INT));
      mcp2515.sendMessage(&sendMsg);
      attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);
      if(digitalRead(SPI_INT) == LOW){mcpISR();}
      vTaskDelay(1);
      
      //delayMicroseconds(500);
    }

    sendMsg.can_id = ADDR_STAT;
    sendMsg.can_dlc = 1;
    sendMsg.data[0] = digitalRead(G_5V_MCU)     << 0
                    | digitalRead(G_12V_D455_3) << 1
                    | digitalRead(G_12V_D455_2) << 2
                    | digitalRead(G_12V_D455_1) << 3
                    | digitalRead(G_12V_PC)     << 4
                    | digitalRead(G_24V)        << 5;
    detachInterrupt(digitalPinToInterrupt(SPI_INT));
    mcp2515.sendMessage(&sendMsg);
    attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);
    if(digitalRead(SPI_INT) == LOW){mcpISR();}
    vTaskDelay(1);
    //delayMicroseconds(500);

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
    detachInterrupt(digitalPinToInterrupt(SPI_INT));
    mcp2515.sendMessage(&sendMsg);
    attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);
    if(digitalRead(SPI_INT) == LOW){mcpISR();}
    
    //taskEXIT_CRITICAL();
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

  mcp2515.setNormalMode();
  pinMode(SPI_INT, INPUT);

  semaphoreSequence = xSemaphoreCreateBinary();

  xTaskCreate(task_sequence,  "task_sequence",  configMINIMAL_STACK_SIZE, NULL, 5,  NULL);
  xTaskCreate(task_send,      "task_send",      configMINIMAL_STACK_SIZE, NULL, 9,  NULL);

  attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR,       FALLING);
  //attachInterrupt(digitalPinToInterrupt(SW_EW),   &emergencyISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(SW_0),    &poweronISR,   FALLING);
  //attachInterrupt(digitalPinToInterrupt(SW_1),    &shutdownISR,  FALLING);
  //attachInterrupt(digitalPinToInterrupt(SW_2),    &rebootISR,    FALLING);
  vTaskStartScheduler(); 
}

void loop()
{
  //Serial.println("loop");
  if(digitalRead(SW_EW) == LOW && flag_emergency == false && flag_power_on == true)
  {
    emergencyISR();
  }

  if(digitalRead(SW_0) == LOW && flag_sequencing == false && flag_power_on == false)
  {
    if(poweron_cnt >= 500)
    {
      poweron_cnt = 0;
      poweronISR();
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
      shutdownISR();
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
  
  //Serial.println(poweron_cnt);
  //Serial.println(shutdown_cnt);
  //Serial.println(reboot_cnt);
  
  delay(1);
}
