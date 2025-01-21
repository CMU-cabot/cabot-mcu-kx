#include <STM32FreeRTOS.h>
#include <mcp2515.h>
#include <SPI.h>
#include <Wire.h>

#define DIP_SW_0      PA2
#define DIP_SW_1      PA3
#define DIP_SW_2      PA4
#define DIP_SW_3      PA5

#define SPI_MOSI      PB5
#define SPI_MISO      PB4
#define SPI_SCK       PB3
#define SPI_CS_PIN    PA15
#define SPI_INT       PA13

#define MCP_RST       PA14

#define I2C_SDA       PB7
#define I2C_SCL       PB6

#define STM32_TXD     PA9
#define STM32_RXD     PA10

#define ADT_RTS       PA1
#define SENS_ADT      0x48

#define ADDR_BASE     0x21

MCP2515 mcp2515(SPI_CS_PIN);

char node_id = ADDR_BASE;
struct can_frame sendMsg;
struct can_frame recvMsg;

int checkSlave(uint8_t addr)
{
  int err = 0;
  Wire.beginTransmission(addr);
  err = Wire.endTransmission( );
  if(err != 0)
  {
    Wire.end();
    delayMicroseconds(500);
    Wire.begin();
    delayMicroseconds(500);
  }
  return err;
}

void mcpISR(){
  long unsigned int id;
  unsigned char len = 0;
  unsigned char buff[8];
}

void task_measure(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 500;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
     int16_t temp;
     int err = checkSlave(SENS_ADT);
     Serial.print("ADT ");
     Serial.print(err);
     Wire.requestFrom(SENS_ADT, 2);
     temp = (((uint16_t)(Wire.read())<<8) | Wire.read())>>3;
     Serial.print("  temp ");
     Serial.println(temp);

     sendMsg.can_id = node_id;
     sendMsg.can_dlc = 2;
     sendMsg.data[0] = (temp & 0x00ff)>>0;
     sendMsg.data[1] = (temp & 0xff00)>>8;
     detachInterrupt(digitalPinToInterrupt(SPI_INT));
     mcp2515.sendMessage(&sendMsg);
     attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);
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

  pinMode(DIP_SW_0, INPUT);
  pinMode(DIP_SW_1, INPUT);
  pinMode(DIP_SW_2, INPUT);
  pinMode(DIP_SW_3, INPUT);
  
  pinMode(MCP_RST, OUTPUT);
  pinMode(ADT_RTS, OUTPUT);

  digitalWrite(MCP_RST, HIGH);
  digitalWrite(ADT_RTS, LOW);
  delay(10);
  digitalWrite(MCP_RST, LOW);
  digitalWrite(ADT_RTS, HIGH);
  delay(10);
  digitalWrite(MCP_RST, HIGH);
  digitalWrite(ADT_RTS, LOW);
  delay(10);

  char id = 0;
  id = digitalRead(DIP_SW_0) << 0
     | digitalRead(DIP_SW_1) << 1
     | digitalRead(DIP_SW_2) << 2
     | digitalRead(DIP_SW_3) << 3;
  node_id += id;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_20MHZ);
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x07ff0000);
  mcp2515.setFilter(MCP2515::RXF0, false, 0x00000000);
  mcp2515.setNormalMode();
  pinMode(SPI_INT, INPUT);

  xTaskCreate(task_measure,  "task_measure",  configMINIMAL_STACK_SIZE, NULL, 10,  NULL);

  attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);
  vTaskStartScheduler(); 
}

void loop()
{
  delay(1);
}
