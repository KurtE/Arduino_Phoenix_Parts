// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022 or 23
#define write send
#define read receive
#endif
#include <Wire.h>
#include "i2cEEProm.h"

I2CEEPromClass I2CEEPROM;

// ##########  SETTINGS ###########################################
#define DEVICEADDRESS   0x50 // 0b0101 0 a2 a1 a0

// ##########  IMPLEMENTATION #####################################
void I2CEEPromClass::begin()
{
  Wire.begin();
}
// ----------------------------------------------------------------
void I2CEEPromClass::writeTo(uint16_t wAddr, uint8_t b)
{

    Wire.beginTransmission(DEVICEADDRESS);
    Wire.write( (wAddr >> 8) & 0xFF );
    Wire.write( (wAddr >> 0) & 0xFF );
    Wire.write(b);
    Wire.endTransmission();
    delay(5);
}
// ----------------------------------------------------------------
void I2CEEPromClass::writeTo(uint16_t wAddr, uint8_t *pb, uint8_t cb)
{

    Wire.beginTransmission(DEVICEADDRESS);
    Wire.write( (wAddr >> 8) & 0xFF );
    Wire.write( (wAddr >> 0) & 0xFF );
    while (cb--) 
    {
        Wire.write(*pb++);
    }
    Wire.endTransmission();
    delay(5);
}

// ----------------------------------------------------------------
uint8_t I2CEEPromClass::readFrom(uint16_t wAddr) 
{
  uint8_t u8retVal = 0;
  Wire.beginTransmission(DEVICEADDRESS);
  Wire.write( (wAddr >> 8) & 0xFF );
  Wire.write( (wAddr >> 0) & 0xFF );
  Wire.endTransmission();
  delay(5);
  Wire.requestFrom(DEVICEADDRESS, 1);
  u8retVal = Wire.read();
  return u8retVal ;
}

