#include <STM32FreeRTOS.h>
#include <mcp_can.h>
#include <SPI.h>
#include <Wire.h>

#define SW_0          PA0
#define SW_1          PA1
#define SW_2          PA2
#define SW_EW         PA12
#define LED_0         PA3
#define LED_1         PA4
#define LED_2         PA5
#define G_24V         PA6
#define G_12V_PC      PA7
#define G_12V_D455_1  PA11
#define G_12V_D455_2  PA14
#define G_12V_D455_3  PB0
#define G_5V_MCU      PA8
#define PWM_FAN       PB1

#define SPI_MOSI      PB5
#define SPI_MISO      PB4
#define SPI_SCK       PB3
#define SPI_CS_PIN    PA15
#define SPI_INT       PA13

#define I2C_SDA       PB7
#define I2C_SCL       PB6

#define STM32_TXD     PA9
#define STM32_RXD     PA10

#define ADDR_BATT     0x0b
#define ADDR_MUX      0x70

#define ADDR_VOLTAGE  0x09
#define ADDR_CULLENT  0x0a
#define ADDR_CHARGE   0x0d 
#define ADDR_TEMP     0x08
#define ADDR_SERIAL   0x1c

MCP_CAN CAN0(SPI_CS_PIN);

volatile SemaphoreHandle_t semaphoreSequence;

volatile bool flag_power_on   = false;
volatile bool flag_shutdown   = false;
volatile bool flag_rebooting  = false;
volatile bool flag_emergency  = false;
volatile bool flag_sequencing = false;

volatile unsigned char buff0x15[1];
volatile unsigned char buff0x16[1];
volatile unsigned char buff0x17[1];
volatile unsigned char buff0x18[1];
volatile unsigned char buff0x19[1];
volatile unsigned char buff0x1a[1];
volatile unsigned char buff0x1e[1];

void setChannel(uint8_t ch)
{
  Wire.beginTransmission(ADDR_MUX);
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
}

uint16_t readWord(uint8_t addr)
{
  uint16_t ret = 0;
  Wire.beginTransmission(ADDR_BATT);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR_BATT, 2, true);
  ret = Wire.read();
  ret |= Wire.read()<<8;
  return ret;
}

void mcpISR(){
  //Serial.println("isr");
  long unsigned int id;
  unsigned char len = 0;
  unsigned char buff[8];
  
  CAN0.readMsgBuf(&id, &len, buff);
  
  if((id & 0x80000000) == 0x80000000){return;}

  if(id == 0x15)
  {
    memcpy((unsigned char *)buff0x15, buff, 1);
   (buff0x15[0]&0x01 == 0x01) ? (digitalWrite(G_24V, HIGH)) : (digitalWrite(G_24V, LOW));
   if(buff0x15[0]&0x01 == 0x01 && flag_emergency == true)
   {
    flag_emergency = false;
   }
  }
  if(id == 0x16)
  {
    memcpy((unsigned char *)buff0x16, buff, 1);
    if(buff0x16[0] == 0x00)
    {
      shutdownISR();
    }
    if(buff0x16[0] == 0x01)
    {
      poweronISR();
    }
    if(buff0x16[0] == 0xff)
    {
      rebootISR();
    }
  }
  if(id == 0x17)
  {
    memcpy((unsigned char *)buff0x17, buff, 1);
    (buff0x17[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_1, HIGH)) : (digitalWrite(G_12V_D455_1, LOW));
  }
  if(id == 0x18)
  {
    memcpy((unsigned char *)buff0x18, buff, 1);
    (buff0x18[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_2, HIGH)) : (digitalWrite(G_12V_D455_2, LOW));
  }
  if(id == 0x19)
  {
    memcpy((unsigned char *)buff0x19, buff, 1);
    (buff0x19[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_3, HIGH)) : (digitalWrite(G_12V_D455_3, LOW));
  }
  if(id == 0x1a)
  {
    memcpy((unsigned char *)buff0x1a, buff, 1);
    (buff0x1a[0]&0x01 == 0x01) ? (digitalWrite(G_5V_MCU, HIGH)) : (digitalWrite(G_5V_MCU, LOW));
  }
  if(id == 0x1e)
  {
    memcpy((unsigned char *)buff0x1e, buff, 1);
    analogWrite(PWM_FAN, 255 - buff0x1e[0]);
  }
  
}

void emergencyISR()
{
  if(flag_emergency == false)
  {
    digitalWrite(G_24V, LOW);
    flag_emergency = true;
    byte data[1] = {0};
    data[0] = 0x01;
    CAN0.sendMsgBuf(0x01, 0, 1, data);
  }
}

void poweronISR()
{
  if(flag_sequencing == false && flag_power_on == false)
  {
    flag_sequencing = true;
    digitalWrite(LED_0, HIGH);
    flag_power_on = true;
    byte data[1] = {0};
    data[0] = 0x01;
    CAN0.sendMsgBuf(0x02, 0, 1, data);
    xSemaphoreGiveFromISR(semaphoreSequence, NULL);
  }
}

void shutdownISR()
{
  if(flag_sequencing == false && flag_power_on == true)
  {
    flag_sequencing = true;
    digitalWrite(LED_1, HIGH);
    flag_shutdown  = true;
    byte data[1] = {0};
    data[0] = 0x00;
    CAN0.sendMsgBuf(0x02, 0, 1, data);
    xSemaphoreGiveFromISR(semaphoreSequence, NULL);
  }
}

void rebootISR()
{
  if(flag_sequencing == false && flag_power_on == true)
  {
    flag_sequencing = true;
    digitalWrite(LED_2, HIGH);
    flag_rebooting  = true;
    byte data[1] = {0};
    data[0] = 0xff;
    CAN0.sendMsgBuf(0x02, 0, 1, data);
    xSemaphoreGiveFromISR(semaphoreSequence, NULL);
  }
}

//shutdown/reboot sequence task
void task_sequence(void *pvParameters)
{
  while(1)
  {
    xSemaphoreTake(semaphoreSequence, portMAX_DELAY);
    byte data[1] = {0};
    
    if(flag_power_on == true && flag_shutdown == false && flag_rebooting == false)
    {
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
      flag_rebooting = false;
    }
    else if(flag_power_on == true && flag_shutdown == true && flag_rebooting == false)
    {
      data[0] = 0x01;
      //CAN0.sendMsgBuf(0x02, 0, 1, data);
      digitalWrite(LED_0, LOW);
      vTaskDelay(5000);
      digitalWrite(G_24V, LOW);
      digitalWrite(G_12V_PC, LOW);
      digitalWrite(G_12V_D455_1, LOW);
      digitalWrite(G_12V_D455_2, LOW);
      digitalWrite(G_12V_D455_3, LOW);
      digitalWrite(G_5V_MCU, LOW);
      analogWrite(PWM_FAN, 255);
      digitalWrite(LED_1, LOW);
    
      flag_power_on  = false;
      flag_shutdown  = false;
      flag_rebooting = false;
      flag_emergency = false;
    }
    else if(flag_power_on == true && flag_shutdown == false && flag_rebooting == true)
    {
      data[0] = 0x01;
      //CAN0.sendMsgBuf(0x02, 0, 1, data);
      digitalWrite(LED_0, LOW);
      vTaskDelay(5000);
      digitalWrite(G_24V, LOW);
      digitalWrite(G_12V_PC, LOW);
      digitalWrite(G_12V_D455_1, LOW);
      digitalWrite(G_12V_D455_2, LOW);
      digitalWrite(G_12V_D455_3, LOW);
      digitalWrite(G_5V_MCU, LOW);
      analogWrite(PWM_FAN, 255);

      vTaskDelay(1000);

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
      digitalWrite(LED_0, HIGH);
      digitalWrite(LED_2, LOW);
    
      flag_power_on  = true;
      flag_shutdown  = false;
      flag_rebooting = false;
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
    byte indicator[1] = {0};
    indicator[0] = digitalRead(LED_0) << 0
                 | digitalRead(LED_1) << 1
                 | digitalRead(LED_2) << 2;
    CAN0.sendMsgBuf(0x03, 0, 1, indicator);
    vTaskDelay(1);
    //delayMicroseconds(500);

    uint16_t voltage  = 0;
    uint16_t current  = 0;
    uint16_t charge   = 0;
    uint16_t temp     = 0;
    uint16_t sn       = 0;
    byte battery[8]   = {0};
    byte battery_sn[8]= {0};
    
    for(int i=0;i<4;i++)
    {
      setChannel(i);
      voltage = readWord(ADDR_VOLTAGE); //Voltage[mV]
      current = readWord(ADDR_CULLENT); //Current[mA]
      charge  = readWord(ADDR_CHARGE);  //RelativeStateOfCharge[%]
      temp    = readWord(ADDR_TEMP);    //Temperature[0.1k]
      sn      = readWord(ADDR_SERIAL);  //SerialNumber
      
      battery [0] = (voltage&0x00ff)  >> 0;
      battery [1] = (voltage&0xff00)  >> 8;
      battery [2] = (current&0x00ff)  >> 0;
      battery [3] = (current&0xff00)  >> 8;
      battery [4] = (charge&0x00ff)   >> 0;
      battery [5] = (charge&0xff00)   >> 8;
      battery [6] = (temp&0x00ff)     >> 0;
      battery [7] = (temp&0xff00)     >> 8;

      battery_sn[i*2]   = (sn&0x00ff) >> 0;
      battery_sn[i*2+1] = (sn&0xff00) >> 8;
      
      CAN0.sendMsgBuf(0x05+i, 0, 8, battery);
      vTaskDelay(1);
      //delayMicroseconds(500);
    }
    
    byte data[1] = {0};
    data[0] =  digitalRead(G_5V_MCU)     << 0
             | digitalRead(G_12V_D455_3) << 1
             | digitalRead(G_12V_D455_2) << 2
             | digitalRead(G_12V_D455_1) << 3
             | digitalRead(G_12V_PC)     << 4
             | digitalRead(G_24V)        << 5;
    CAN0.sendMsgBuf(0x14, 0, 1, data);
    vTaskDelay(1);
    //delayMicroseconds(500);

    CAN0.sendMsgBuf(0x1d, 0, 8, battery_sn);
    
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

  //Serial2.setRx(PNUM_NOT_DEFINED);
  //Serial2.setTx(PNUM_NOT_DEFINED);
  //Serial.println("booting");

  pinMode(SW_0, INPUT);
  pinMode(SW_1, INPUT);
  pinMode(SW_2, INPUT);
  pinMode(SW_EW, INPUT);
  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(G_24V, OUTPUT);
  pinMode(G_12V_PC, OUTPUT);
  pinMode(G_12V_D455_1, OUTPUT);
  pinMode(G_12V_D455_2, OUTPUT);
  pinMode(G_12V_D455_3, OUTPUT);
  pinMode(G_5V_MCU, OUTPUT);
  pinMode(PWM_FAN, OUTPUT);

  CAN0.begin(MCP_STDEXT, CAN_1000KBPS, MCP_20MHZ);
  CAN0.init_Mask(0,0,0x07fc0000);
  CAN0.init_Filt(0,0,0x00140000);
  CAN0.init_Filt(1,0,0x00180000);
  CAN0.init_Mask(1,0,0x07ff0000);
  CAN0.init_Filt(2,0,0x001e0000);
  CAN0.setMode(MCP_NORMAL);
  pinMode(SPI_INT, INPUT);

  semaphoreSequence = xSemaphoreCreateBinary();

  xTaskCreate(task_sequence,  "task_sequence",  configMINIMAL_STACK_SIZE, NULL, 9,  NULL);
  xTaskCreate(task_send,      "task_send",      configMINIMAL_STACK_SIZE, NULL, 9,  NULL);

  attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR,       FALLING);
  //attachInterrupt(digitalPinToInterrupt(SW_EW),   &emergencyISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SW_0),    &poweronISR,   FALLING);
  attachInterrupt(digitalPinToInterrupt(SW_1),    &shutdownISR,  FALLING);
  //attachInterrupt(digitalPinToInterrupt(SW_2),    &rebootISR,    FALLING);
  vTaskStartScheduler(); 
}

void loop()
{
  if(digitalRead(SW_EW) == LOW)
  {
    if(flag_emergency == false && flag_power_on == true)
    {
      digitalWrite(G_24V, LOW);
      flag_emergency = true;
      byte data[1] = {0};
      data[0] = 0x01;
      CAN0.sendMsgBuf(0x01, 0, 1, data);
    }
  }
  if(digitalRead(SW_2) == LOW)
  {
    if(flag_sequencing == false && flag_power_on == true)
    {
      flag_sequencing = true;
      digitalWrite(LED_2, HIGH);
      flag_rebooting  = true;
      byte data[1] = {0};
      data[0] = 0xff;
      CAN0.sendMsgBuf(0x02, 0, 1, data);
      xSemaphoreGiveFromISR(semaphoreSequence, NULL);
    }
  }
  delay(1);
}
