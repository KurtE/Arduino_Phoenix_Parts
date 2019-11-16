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
#include <Arduino.h>

#include <EEPROM.h>
#include "Hex_CFG.h"
#include "Phoenix.h"
#include <Phoenix_Input_PS3.h>
#include <Phoenix_Driver_Maestro.h>
#include <Phoenix_Code.h>
