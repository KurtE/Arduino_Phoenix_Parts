// This is a completely mucked up version of the PinChangeInt.h library...
#ifndef PCInt_h
#define	PCInt_h
#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
  #include <pins_arduino.h>
#endif
/*
 * an extension to the interrupt support for arduino.
 * add pin change interrupts to the external interrupts, giving a way
 * for users to have interrupts drive off of any pin.
 * Refer to avr-gcc header files, arduino source and atmega datasheet.
 */

/*
 * Theory: all IO pins on Atmega168 are covered by Pin Change Interrupts.
 * The PCINT corresponding to the pin must be enabled and masked, and
 * an ISR routine provided.  Since PCINTs are per port, not per pin, the ISR
 * must use some logic to actually implement a per-pin interrupt service.
 */

/* Pin to interrupt map:
// #define NO_PINCHANGE_0   // No Pin changes on 0-7 - Typically Analog pins A0-A5.
// #define NO_PINCHANGE_1   // No Pin changes on 9-15 - On non-mega: D8-13
// #define NO_PINCHANGE_2   // No Pin Changes on 16-23 - non-mega: D0-7
 * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
 * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
 */

// Data associated with each Pin Change interrupt... 
typedef void (*PCINTFUNCPTR)(uint8_t, uint8_t, unsigned long);



volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
#ifdef PCINT3_vect
  ,&PCMSK3
#endif
};

#define CNTPINCHANGES  (sizeof(port_to_pcmask)/sizeof(port_to_pcmask[0])*8)
static uint8_t PCintMode[CNTPINCHANGES];              // what mode RISING, FALLING, CHange
static uint8_t bUserData[CNTPINCHANGES];              // Data passed back to the user to identify interrupt...
volatile static PCINTFUNCPTR PCintFunc[CNTPINCHANGES] = {  NULL };  // Pointer to function to call


volatile static uint8_t PCintLast[3];

/*
 * attach an interrupt to a specific pin using pin change interrupts.
 */
 void PCattachInterrupt(uint8_t pin, PCINTFUNCPTR userFunc, uint8_t ident, uint8_t mode) {
  // See if this pin has a PCICR register associated with it.
   volatile uint8_t *pcmask = digitalPinToPCMSK(pin);
   if (!pcmask)
     return;    // nope
 
   // Now get the slot number...
   uint8_t bPCIRBit = digitalPinToPCICRbit(pin);
   uint8_t bPCMSKBit = digitalPinToPCMSKbit(pin);  // this gets us to the right pin to update...     
   uint8_t slot = bPCIRBit * 8 + bPCMSKBit;    // Have our slot now   

   // Now lets save away the passed in information...
   PCintMode[slot] = mode;
   PCintFunc[slot] = userFunc;
   bUserData[slot] = ident;
  
  // Now lets enable the interrupt
  *pcmask |= (0x01 << bPCMSKBit);    // Set the appropriate bit in the Pin Change Mask
  // enable the interrupt
  PCICR |= (0x01 << bPCIRBit);      // Enable the interrupt associated with that pin
}

void PCdetachInterrupt(uint8_t pin) {
  // See if this pin has a PCICR register associated with it.
   volatile uint8_t *pcmask = digitalPinToPCMSK(pin);
   if (!pcmask)
     return;    // nope
 
   // Now get the slot number...
   uint8_t bPCIRBit = digitalPinToPCICRbit(pin);
   uint8_t bPCMSKBit = digitalPinToPCMSKbit(pin);  // this gets us to the right pin to update...     
   //uint8_t slot = bPCIRBit * 8 + bPCMSKBit;    // Have our slot now   

  // disable the mask.
  *pcmask &= ~(0x01 << bPCMSKBit);
  // if that's the last one, disable the interrupt.
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << bPCIRBit);
  }
}

// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
static void PCint(uint8_t port) {
  uint8_t bPinMask;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  unsigned long ulTime;

  // get the pin states for the indicated port.
  ulTime = micros();  // get the time of the interrupt as quickly as we can.
  
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  // mask is pcint pins that have changed.
  bPinMask = 0x01;
  for (uint8_t i=0; i < 8; i++) {
    if (bPinMask & mask) {
      pin = port * 8 + i;
      // Trigger interrupt if mode is CHANGE, or if mode is RISING and
      // the bit is currently high, or if mode is FALLING and bit is low.
      if ((PCintMode[pin] == CHANGE
          || ((PCintMode[pin] == RISING) && (curr & bPinMask))
          || ((PCintMode[pin] == FALLING) && !(curr & bPinMask)))
          && (PCintFunc[pin] != NULL)) {
        PCintFunc[pin](bUserData[pin], curr & bPinMask, ulTime);
      }
    }
    bPinMask <<= 1;
  }
}

#if defined(PCINT0_vect) && !defined(NO_PINCHANGE_0)
ISR(PCINT0_vect)  {
  PCint(0);
}
#endif 

#if defined(PCINT1_vect) && !defined(NO_PINCHANGE_1)
ISR(PCINT1_vect) {
  PCint(1);
}
#endif

#if defined(PCINT2_vect) && !defined(NO_PINCHANGE_2)
ISR(PCINT2_vect) {
  PCint(2);
}
#endif

#if defined(PCINT3_vect) && !defined(NO_PINCHANGE_3)
ISR(PCINT3_vect) {
  PCint(3);
}
#endif

#endif