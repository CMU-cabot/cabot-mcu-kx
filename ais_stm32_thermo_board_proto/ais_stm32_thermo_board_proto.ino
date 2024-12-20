#include <STM32FreeRTOS.h>
#include <mcp_can.h>
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
#define ADDR_ADT      0x48

MCP_CAN CAN0(SPI_CS_PIN);

volatile bool flag_power_on   = false;
volatile bool flag_shutdown   = false;
volatile bool flag_rebooting  = false;
volatile bool flag_emergency  = false;
volatile bool flag_sequencing = false;

char node_id = 0x21;

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
  
  CAN0.readMsgBuf(&id, &len, buff);
}

void task_measure(void *pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xFrequency = 500;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
     int16_t temp;
     int err = checkSlave(ADDR_ADT);
     Serial.print("ADT ");
     Serial.print(err);
     Wire.requestFrom(ADDR_ADT, 2);
     temp = (((uint16_t)(Wire.read())<<8) | Wire.read())>>3;
     Serial.print("  temp ");
     Serial.println(temp);

     byte send_data[2] = {0};
     send_data[0] = (temp & 0x00ff)>>0;
     send_data[1] = (temp & 0xff00)>>8;
     detachInterrupt(digitalPinToInterrupt(SPI_INT));
     CAN0.sendMsgBuf(node_id, 0, 2, send_data);
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

  CAN0.begin(MCP_STDEXT, CAN_1000KBPS, MCP_20MHZ);
  CAN0.init_Mask(0,0,0x07ff0000);
  CAN0.init_Filt(0,0,0x00000000);
  CAN0.setMode(MCP_NORMAL);
  pinMode(SPI_INT, INPUT);

  xTaskCreate(task_measure,  "task_measure",  configMINIMAL_STACK_SIZE, NULL, 10,  NULL);

  attachInterrupt(digitalPinToInterrupt(SPI_INT), &mcpISR, FALLING);
  vTaskStartScheduler(); 
}

void loop()
{
  delay(1);
}
