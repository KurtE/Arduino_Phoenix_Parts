#include <SPI.h>
#include <Orion.h>


#include <BMSerial.h>

#include <BMPS2.h>


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
// and is specifically configured for the Orion Robotics Orion sheild.
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



#include "Quad_Cfg.h"

#include <Phoenix.h>
//  Speed, Steps, Lifted, Front Down, Lifted Factor, Half Height, On Ground, 
//     Quad extra: COGAngleStart, COGAngleStep, CogRadius, COGCCW
//                      { RR, <RM> RF, LR, <LM>, LF}
#define ADD_GAITS
#define PYPOSE_GAIT_SPEED 98
PHOENIXGAIT APG_EXTRA[] = { 
  {PYPOSE_GAIT_SPEED, 8, 2, 1, 2, 6, 1, 0, 0,0, true, {7, 1, 3, 5}},   // ripple
  {PYPOSE_GAIT_SPEED, 4, 2, 1, 2, 2, 1, 0, 0, 0, true,{3, 1, 1, 3}},  // Amble
  {PYPOSE_GAIT_SPEED, 6, 3, 2, 2, 3, 2, 0, 0,0, true, {1, 4, 4, 1}} }; // Smooth Amble 


#include <Phoenix_Input_Orion_PS2.h>
#include <Phoenix_driver_Orion.h>
#include <Phoenix_Code.h>

