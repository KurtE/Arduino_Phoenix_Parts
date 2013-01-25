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
// and is specifically configured for the Arbotix Robocontroller board
//
//=============================================================================
// Warning:: This configuration does not check voltages, so you should be careful to
// not allow the lipo to discharge too far. 
//
// This configuration should hopefully run on a stock PhantomX, without any
// of my changes.
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
#include <EEPROM.h>
#include <ax12.h>
#include <BioloidEx.h>

#include "Hex_Cfg.h"

#include <Phoenix.h>
#include <Phoenix_Input_Commander.h>
#include <Phoenix_Driver_AX12.h>
#include <Phoenix_Code.h>

