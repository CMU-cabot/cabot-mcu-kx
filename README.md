# cabot mcu firmware
### IDE
Arduino IDE 1.8.19

### Boards
Arduino_Core_STM32 2.7.1  
https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json  
arduino-esp32 2.0.17  
https://espressif.github.io/arduino-esp32/package_esp32_index.json  

### Libraries
Adafruit_BME280_Library 2.2.4  
https://github.com/adafruit/Adafruit_BME280_Library  
Adafruit_BNO055 1.6.3  
https://github.com/adafruit/Adafruit_BNO055  
Adafruit_BusIO 1.16.0  
https://github.com/adafruit/Adafruit_BusIO  
Adafruit_Sensor 1.1.14  
https://github.com/adafruit/Adafruit_Sensor  
Adafruit_VL53L0X 1.2.4  
https://github.com/adafruit/Adafruit_VL53L0X  
CAN 0.3.1  
https://github.com/sandeepmistry/arduino-CAN  
Dynamixel2Arduino 0.5.3  
https://github.com/ROBOTIS-GIT/dynamixel2arduino  
MCP_CAN 1.5.1  
https://github.com/coryjfowler/MCP_CAN_lib  
STM32FreeRTOS 10.3.2  
https://github.com/stm32duino/STM32FreeRTOS  

### Settings
for STM32 based board  
![STM32設定](https://github.com/user-attachments/assets/5e92fedd-792b-4af9-8a2d-fd6fe42dea43)  
for ESP32 based board  
![ESP32設定](https://github.com/user-attachments/assets/db395fd7-5742-4789-a4cb-e3f2e399044a)  

### docker environment

- build
```
docker compose build
```

- run container for each project
```
docker compose run --rm esp32 bash
docker compose run --rm stm32-handle bash
docker compose run --rm stm32-power bash
docker compose run --rm stm32-thermo bash
```

- build and upload
  - you may need to specify the port with `-p` option
```
./build.sh
./build.sh upload

or

./build.sh all
```
