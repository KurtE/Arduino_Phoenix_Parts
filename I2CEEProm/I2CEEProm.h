#ifndef _I2CEEPROM_h_
#define _I2CEEPROM_h_
#include <wire.h>
// SETTINGS 
#define EE24LC512MAXBYTES        64000

class I2CEEPromClass /*: public TwoWire - Change when MPIDE is 1.0 */  
{
  public:
  void begin();
  
  void writeTo(uint16_t, uint8_t);
  void writeTo(uint16_t, uint8_t*, uint8_t);
  
  uint8_t readFrom(uint16_t);

};    

extern I2CEEPromClass I2CEEPROM;

#endif
