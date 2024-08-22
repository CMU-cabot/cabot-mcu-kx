#include <STM32FreeRTOS.h>
#include <mcp_can.h>
#include <SPI.h>

#define SW_0          PA0
#define SW_1          PA1
#define SW_2          PA12
#define SW_EW         PB0
#define LED_0         PA3
#define LED_1         PA4
#define LED_2         PA5
#define G_24V         PA11
#define G_12V_PC      PA8
#define G_12V_D455_1  PF1
#define G_12V_D455_2  PF0
#define G_12V_D455_3  PB1
#define G_5V_MCU      PB6
#define PWM_FAN       PB7

#define SPI_MOSI      PB5
#define SPI_MISO      PB4
#define SPI_SCK       PB3
#define SPI_CS_PIN    PA6
#define SPI_INT       PA7

MCP_CAN CAN0(SPI_CS_PIN);

volatile SemaphoreHandle_t semaphoreSequence;

volatile bool flag_power_on   = false;
volatile bool flag_shutdown   = false;
volatile bool flag_rebooting  = false;
volatile bool flag_emergency  = false;
volatile bool flag_sequencing = false;

volatile unsigned char buff0x14[1];
volatile unsigned char buff0x15[1];
volatile unsigned char buff0x16[1];
volatile unsigned char buff0x17[1];
volatile unsigned char buff0x18[1];
volatile unsigned char buff0x19[1];
volatile unsigned char buff0x1d[1];

void mcpISR(){
  //Serial.println("isr");
  long unsigned int id;
  unsigned char len = 0;
  unsigned char buff[8];
  
  CAN0.readMsgBuf(&id, &len, buff);
  
  if((id & 0x80000000) == 0x80000000){return;}

  if(id == 0x15)
  {
    memcpy((unsigned char *)buff0x14, buff, 1);
   (buff0x14[0]&0x01 == 0x01) ? (digitalWrite(G_24V, HIGH)) : (digitalWrite(G_24V, LOW));
   if(buff0x14[0]&0x01 == 0x01 && flag_emergency == true)
   {
    flag_emergency = false;
   }
  }
  if(id == 0x16)
  {
    memcpy((unsigned char *)buff0x15, buff, 1);
    if(buff0x15[0] == 0x00)
    {
      shutdownISR();
    }
    if(buff0x15[0] == 0x01)
    {
      poweronISR();
    }
    if(buff0x15[0] == 0xff)
    {
      rebootISR();
    }
  }
  if(id == 0x17)
  {
    memcpy((unsigned char *)buff0x16, buff, 1);
    (buff0x16[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_1, HIGH)) : (digitalWrite(G_12V_D455_1, LOW));
  }
  if(id == 0x18)
  {
    memcpy((unsigned char *)buff0x17, buff, 1);
    (buff0x17[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_2, HIGH)) : (digitalWrite(G_12V_D455_2, LOW));
  }
  if(id == 0x19)
  {
    memcpy((unsigned char *)buff0x18, buff, 1);
    (buff0x18[0]&0x01 == 0x01) ? (digitalWrite(G_12V_D455_3, HIGH)) : (digitalWrite(G_12V_D455_3, LOW));
  }
  if(id == 0x1a)
  {
    memcpy((unsigned char *)buff0x19, buff, 1);
    (buff0x19[0]&0x01 == 0x01) ? (digitalWrite(G_5V_MCU, HIGH)) : (digitalWrite(G_5V_MCU, LOW));
  }
  if(id == 0x1e)
  {
    memcpy((unsigned char *)buff0x1d, buff, 1);
    analogWrite(PWM_FAN, buff0x1d[0]);
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
      analogWrite(PWM_FAN, 0);
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
      analogWrite(PWM_FAN, 0);

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
    byte indicator[1] = {0};
    indicator[0] = digitalRead(LED_0) << 0
                 | digitalRead(LED_1) << 1
                 | digitalRead(LED_2) << 2;
    CAN0.sendMsgBuf(0x03, 0, 1, indicator);
    vTaskDelay(1);             
    
    byte battery_1[8] = {0x70,0x62,0xd0,0x07,0x32,0x00,0xa0,0x0c};
    byte battery_2[8] = {0x71,0x62,0xd1,0x07,0x33,0x00,0xa1,0x0c};
    byte battery_3[8] = {0x72,0x62,0xd2,0x07,0x34,0x00,0xa2,0x0c};
    byte battery_4[8] = {0x73,0x62,0xd3,0x07,0x35,0x00,0xa3,0x0c};
    CAN0.sendMsgBuf(0x05, 0, 8, battery_1);
    vTaskDelay(1);
    CAN0.sendMsgBuf(0x06, 0, 8, battery_2);
    vTaskDelay(1);
    CAN0.sendMsgBuf(0x07, 0, 8, battery_3);
    vTaskDelay(1);
    CAN0.sendMsgBuf(0x08, 0, 8, battery_4);
    vTaskDelay(1);
    
    byte data[1] = {0};
    data[0] =  digitalRead(G_5V_MCU)     << 0
             | digitalRead(G_12V_D455_3) << 1
             | digitalRead(G_12V_D455_2) << 2
             | digitalRead(G_12V_D455_1) << 3
             | digitalRead(G_12V_PC)     << 4
             | digitalRead(G_24V)        << 5;
    CAN0.sendMsgBuf(0x14, 0, 1, data);
    vTaskDelay(1);

    byte battery_sn[8] = {0x5e,0x0b,0x5f,0x0b,0x60,0x0b,0x61,0x0b};
    CAN0.sendMsgBuf(0x1d, 0, 8, battery_sn);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup()
{
  SPI.setMISO(SPI_MISO);
  SPI.setMOSI(SPI_MOSI);
  SPI.setSCLK(SPI_SCK);

  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200);
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

  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_20MHZ);
  CAN0.setMode(MCP_NORMAL);
  pinMode(SPI_INT, INPUT);

  semaphoreSequence = xSemaphoreCreateBinary();

  xTaskCreate(task_sequence,  "task_sequence",  configMINIMAL_STACK_SIZE, NULL, 9,  NULL);
  xTaskCreate(task_send,      "task_send",      configMINIMAL_STACK_SIZE, NULL, 9,  NULL);

  attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR,       FALLING);
  //attachInterrupt(digitalPinToInterrupt(SW_EW),   &emergencyISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SW_0),    &poweronISR,   FALLING);
  attachInterrupt(digitalPinToInterrupt(SW_1),    &shutdownISR,  FALLING);
  attachInterrupt(digitalPinToInterrupt(SW_2),    &rebootISR,    FALLING);
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
  delay(1);
}
