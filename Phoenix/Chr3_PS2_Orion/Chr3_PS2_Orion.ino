

//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!     
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Lynxmotion BotBoarduino 
//
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

#define DEFINE_HEX_GLOBALS
#if ARDUINO>99
#include <Arduino.h>
#else
#endif
#include <Wire.h>
#include <EEPROM.h>
#include <PS2X_lib.h>

#include <SPI.h>
#include <Orion.h>

#include <I2CEEProm.h>
#include "Hex_Cfg.h"
#include <Phoenix.h>
#include <Phoenix_driver_Orion.h>
#include <Phoenix_Input_PS2.h>

// Have a little initial setup code that allows us to check BTNS and hang for update of arc32...
#define OPT_SKETCHSETUP
void SketchSetup(void) {
  // Lets enable buttons, 
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  if (!digitalRead(2) || !digitalRead(4)) {
    // Looks like we should just hang here!
    pinMode(A0, OUTPUT);
    for(;;) {
      digitalWrite(A0, !digitalRead(A0));
      delay(250);
    }  
  }
}

#include <Phoenix_Code.h>

