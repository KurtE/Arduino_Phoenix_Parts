# Overview
This configuration was designed to work with a single particular board - [Espressif ESP32](https://www.espressif.com/en/products/hardware/esp32/overview). The ESP32 chipset supports bluetooth and supports the PS3 controller as an input device. 
The ESP32 comes in many pre-built board configurations, but the one used for this config was the [M5StickC board](https://github.com/m5stack/M5StickC) which packages the ESP32 with a small battery, screen, USB-C and access to a few IO pins.

# Requirements
## ESP32 Arduino Core
https://github.com/espressif/arduino-esp32

- All the instructions for adding the board to the Arduino config and installing can be found in the Arduino Core repo docs

## ESP32 PS3 Library (develop branch)
https://github.com/jvpernis/esp32-ps3.git

- ```git clone https://github.com/jvpernis/esp32-ps3.git```
- ```git checkout develop```
