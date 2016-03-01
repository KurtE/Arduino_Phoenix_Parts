//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//
//Hardware setup: RC version
//
//====================================================================

// Include the Pin Change code.  Can also tell it not to define some Interrupt vectors...
#define NO_PINCHANGE_0   // No Pin changes on 0-7 - Typically Analog pins A0-A5.
// #define NO_PINCHANGE_1   // No Pin changes on 9-15 - On non-mega: D8-13
// #define NO_PINCHANGE_2   // No Pin Changes on 16-23 - non-mega: D0-7
#include "PCInt.h"

//[CONSTANTS]
enum {RCC_RLR=0, RCC_RUD, RCC_LUD, RCC_LLR, RCC_SWITCH, RCC_KNOB};
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4


#define cTravelDeadZone 4      //The deadzone for the analog input from the remote
#define  MAXRCERRORCNT  5     // How many times through the loop will we go before shutting off robot?

//=============================================================================
// Global - Local to this file only...
//=============================================================================

// Define an instance of the Input Controller...
InputController   g_InputController;       // Our Input controller 
word              g_awRCTimes[RCPIN_COUNT] = {0,0,0, 0, 0, 0};                // to the reciver's channels in the order listed here
static unsigned long g_alRisingEdge[RCPIN_COUNT];
uint8_t           g_bRCValidBits;
boolean           g_fRCDataChanged = false;
boolean           _fSwitchOn;
boolean          _fSwitchOnPrev;


static short      g_BodyYOffset; 
static short      g_bRCErrorCnt;
static short       g_BodyYShift;
static byte        ControlMode;
static bool        DoubleHeightOn;
static bool        DoubleTravelOn;
static bool        WalkMethod;
byte            GPSeq;             //Number of the sequence

// some external or forward function references.
extern void PinChanged(uint8_t iRCChannel, uint8_t bPinState, unsigned long ulTime);        // one of our pins changed state. 

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void InputController::Init(void)
{
    g_BodyYOffset = 0;
    g_BodyYShift = 0;
    g_bRCErrorCnt = 0;  // error count

    ControlMode = WALKMODE;
    DoubleHeightOn = false;
    DoubleTravelOn = false;
    WalkMethod = false;

    // Lets setup the Pin Change Interrupts here... 
    g_bRCValidBits = 0;
    for (byte i=0; i<RCPIN_COUNT; i++)
    {
        uint8_t pin = (byte)pgm_read_byte(&caRCPins[i]);
        pinMode(pin, INPUT);     //set the pin to input
        digitalWrite(pin, HIGH); //use the internal pullup resistor
        PCattachInterrupt(pin, PinChanged, i, CHANGE); // attach a PinChange Interrupt to our first pin
    }


}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void InputController::AllowControllerInterrupts(boolean fAllow)
{
  // With the RC controller, we will enable/disable the Pin Change interrupts.
}

//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
//process any commands.
//==============================================================================
void InputController::ControlInput(void)
{
    // See if we have valid data...
    // Will start of just trying out seeing if we have a complete set of data...
#ifdef DEBUG_RC    
    if (g_fRCDataChanged) {
        Serial.print(g_bRCValidBits, HEX);
        Serial.print(": ");
        for (int i=0; i < RCPIN_COUNT; i++) {
            Serial.print(g_awRCTimes[i], DEC);
            Serial.print(" ");
        }
        Serial.println();
        g_fRCDataChanged = false;
    }    
#endif    
    
    if (g_bRCValidBits == RC_VALID_PIN_MASK) {
        // Have valid data so make sure the robot is on...
        g_bRCErrorCnt = 0;    // clear out error count...
        
        if (!g_InControlState.fRobotOn) {
            g_InControlState.fRobotOn = 1;
            _fSwitchOn = (g_awRCTimes[RCC_SWITCH] > 1500);
        }

	if (g_awRCTimes[RCC_KNOB] <  1300) {
	  if (ControlMode != WALKMODE) {
            MSound(1, 50, 2000);  //sound SOUND_PIN, [50\4000]
	    ControlMode = WALKMODE;
          }
        } else if (g_awRCTimes[RCC_KNOB] < 1500) {
          if (ControlMode != TRANSLATEMODE) {
            MSound(1, 50, 2000);  //sound SOUND_PIN, [50\4000]
	    ControlMode = TRANSLATEMODE;
          }
        } else if (g_awRCTimes[RCC_KNOB] < 1700) {
	  if (ControlMode != ROTATEMODE) {
            MSound(1, 50, 2000);  //sound SOUND_PIN, [50\4000]
	    ControlMode = ROTATEMODE;
          }
        } else {
	  if (ControlMode != SINGLELEGMODE) {
	    ControlMode=SINGLELEGMODE;
            MSound(1, 50, 2000);  //sound SOUND_PIN, [50\4000]
	    g_InControlState.SelectedLeg = 0;
          }
        }
        
        _fSwitchOnPrev = _fSwitchOn;
        _fSwitchOn = (g_awRCTimes[RCC_SWITCH] > 1500);

        if (_fSwitchOn != _fSwitchOnPrev) {           // Select Button Test
            if (ControlMode == WALKMODE) {
                //Switch gates
                g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
                if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
                    MSound(1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                } else {
                    MSound (2, 50, 2000, 50, 2250); 
                    g_InControlState.GaitType = 0;
                }
                GaitSelect();
            } else if (ControlMode == SINGLELEGMODE) {
                MSound (1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                if (g_InControlState.SelectedLeg<5)
                    g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
                else
                    g_InControlState.SelectedLeg=0;
            }
      }
      
      // Body Height...
      g_InControlState.BodyPos.y = (max(g_awRCTimes[RCC_LUD], 1100)-1100) / 6;

      if (ControlMode == WALKMODE) {
          g_InControlState.TravelLength.x = -(g_awRCTimes[RCC_RLR] - 1500) / 3;
          g_InControlState.TravelLength.z = -(g_awRCTimes[RCC_RUD] - 1500) / 3;	
          g_InControlState.TravelLength.y = -(g_awRCTimes[RCC_LLR] - 1500)/10;
      }
      //Body move	
      else if (ControlMode == TRANSLATEMODE) {
          g_InControlState.BodyPos.x = (g_awRCTimes[RCC_RLR]-1500)/6;
	  g_InControlState.BodyPos.z = (g_awRCTimes[RCC_RUD]-1500)/6;
	  g_InControlState.BodyRot1.y = (g_awRCTimes[RCC_LLR]-1500)/2;
      }
			
      // Body rotate	
      else if (ControlMode == ROTATEMODE) {
          g_InControlState.BodyRot1.x = (g_awRCTimes[RCC_RUD]-1500)/2;
	  g_InControlState.BodyRot1.y = (g_awRCTimes[RCC_LLR]-1500)/2;
	  g_InControlState.BodyRot1.z = -(g_awRCTimes[RCC_RLR]-1500)/2;
      }
      //Single Leg Mode
      else if (ControlMode == SINGLELEGMODE) {
          g_InControlState.SLLeg.x = (g_awRCTimes[RCC_RLR]-1500) / 3;
	  g_InControlState.SLLeg.z = -(g_awRCTimes[RCC_RUD]-1500) / 3;
	  g_InControlState.SLLeg.y = -(g_awRCTimes[RCC_LLR]-1500)/ 10;
      }

      //Calculate walking time delay
      g_InControlState.InputTimeDelay = 128 - max(max(abs((g_awRCTimes[RCC_RLR]-1500)/6),abs((g_awRCTimes[RCC_RUD]-1500)/6)), 
            abs((g_awRCTimes[RCC_LLR]-1500)/6));
  
    } else {
      // We lost contact with RC... Allow a couple of errors, then turn robot off...
      if (g_bRCErrorCnt < MAXRCERRORCNT)
          g_bRCErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.
      else if (g_InControlState.fRobotOn) {
         //Turn off
          g_InControlState.BodyPos.x = 0;
          g_InControlState.BodyPos.y = 0;
          g_InControlState.BodyPos.z = 0;
          g_InControlState.BodyRot1.x = 0;
          g_InControlState.BodyRot1.y = 0;
          g_InControlState.BodyRot1.z = 0;
          g_InControlState.TravelLength.x = 0;
          g_InControlState.TravelLength.z = 0;
          g_InControlState.TravelLength.y = 0;
          g_BodyYOffset = 0;
          g_BodyYShift = 0;
          g_InControlState.SelectedLeg = 255;
          g_InControlState.fRobotOn = 0;
      }
    }
}


// Our Interrupt call back functinos...
void PinChanged(uint8_t iRCChannel, uint8_t bPinState, unsigned long ulTime)        // one of our pins changed state. 
{
  if (bPinState) { // rising edge
    g_alRisingEdge[iRCChannel] = ulTime;    // get the current timer...
  } else {
    word w =  ulTime - g_alRisingEdge[iRCChannel];
    if ((w < RC_VALID_MIN) || (w > RC_VALID_MAX)) {
        // Received some bad data, lets remember this and setup such that caller can do something appropriate.
        g_bRCValidBits = 0;     // We have no valid data...
    } else {
       g_bRCValidBits |= 1 << iRCChannel;
       if (w != g_awRCTimes[iRCChannel]) {
        g_awRCTimes[iRCChannel] = w;
        g_fRCDataChanged = true;
      }
     }
  }
}


