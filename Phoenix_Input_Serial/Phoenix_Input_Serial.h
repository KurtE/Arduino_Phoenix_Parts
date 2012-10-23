//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: Serial version - This contoll input, uses the same format of input as the old Powerpod serial test program.  
// Obviously it can be hacked up to almost any format
// 
//NEW IN V1.1
//	- added speaker constant
//	- added variable for number of gaits in code
//	- Changed BodyRot to 1 decimal percision
//	- Added variable Center Point of Rotation for the body
//
//	Walk method 1:
//	- Left Stick	Walk/Strafe
//	- Right Stick	Rotate
//
//	Walk method 2:
//	- Left Stick	Disable
//	- Right Stick	Walk/Rotate
//
//
//
// Packet format:
// DualShock(0) : Checksum of other byte
// DualShock(1) 
//   bit7 - Left Button test
//   bit6 - Down Button test
//   bit5 - Right Button test
//   bit4 - Up Button test
//   bit3 - Start Button test
//   bit2 - R3 Button test (Horn)
//   bit1 - L3 Button test
//   bit0 - Select Button test
// DualShock(2)
//	bit7 - Square Button test
//	bit6 - Cross Button test
//	bit5 - Circle Button test
//	bit4 - Triangle Button test
//	bit3 - R1 Button test
//	bit2 - L1 Button test
//	bit1 - R2 Button test
//	bit0 - L2 Button test
// DualShock(3) - Right stick Left/right
// DualShock(4) - Right Stick Up/Down
// DualShock(5) - Left Stick Left/right
// DualShock(6) - Left Stick Up/Down
// Note: The actual usages are from PS2 control
//PS2 CONTROLS:
//	[Common Controls]
//	- Start			Turn on/off the bot
//	- L1			Toggle Shift mode
//	- L2			Toggle Rotate mode
//	- Circle		Toggle Single leg mode
//   - Square        Toggle Balance mode
//	- Triangle		Move body to 35 mm from the ground (walk pos) 
//					and back to the ground
//	- D-Pad up		Body up 10 mm
//	- D-Pad down	Body down 10 mm
//	- D-Pad left	decrease speed with 50mS
//	- D-Pad right	increase speed with 50mS
//
//	[Walk Controls]
//	- select		Switch gaits
//	- Left Stick	(Walk mode 1) Walk/Strafe
//				 	(Walk mode 2) Disable
//	- Right Stick	(Walk mode 1) Rotate, 		
//					(Walk mode 2) Walk/Rotate
//	- R1			Toggle Double gait travel speed
//	- R2			Toggle Double gait travel length
//
//	[Shift Controls]
//	- Left Stick	Shift body X/Z
//	- Right Stick	Shift body Y and rotate body Y
//
//	[Rotate Controls]
//	- Left Stick	Rotate body X/Z
//	- Right Stick	Rotate body Y	
//
//	[Single leg Controls]
//	- select		Switch legs
//	- Left Stick	Move Leg X/Z (relative)
//	- Right Stick	Move Leg Y (absolute)
//	- R2			Hold/release leg position
//
//	[GP Player Controls]
//	- select		Switch Sequences
//	- R2			Start Sequence
//
//====================================================================
// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

//[CONSTANTS]
// Default to Serial but allow to be defined to something else
#ifndef SerSerial
#define SerSerial Serial
#endif

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 38400
#endif

#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4

#define SERB_PAD_LEFT    0x8000    //   bit7 - Left Button test
#define SERB_PAD_DOWN    0x4000    //   bit6 - Down Button test
#define SERB_PAD_RIGHT   0x2000    //   bit5 - Right Button test
#define SERB_PAD_UP      0x1000    //   bit4 - Up Button test
#define SERB_START       0x800      //   bit3 - Start Button test
#define SERB_R3          0x400    //   bit2 - R3 Button test (Horn)
#define SERB_L3          0x200    //   bit1 - L3 Button test
#define SERB_SELECT      0x100    //   bit0 - Select Button test
// DualShock(2)
#define SERB_SQUARE      0x80    //	bit7 - Square Button test
#define SERB_CROSS       0x40    //	bit6 - Cross Button test
#define SERB_CIRCLE      0x20    //	bit5 - Circle Button test
#define SERB_TRIANGLE    0x10    //	bit4 - Triangle Button test
#define SERB_R1          0x8    //	bit3 - R1 Button test
#define SERB_L1          0x4    //	bit2 - L1 Button test
#define SERB_R2          0x2    //	bit1 - R2 Button test
#define SERB_L2          0x1    //	bit0 - L2 Button test

#define  SER_RX          3             // DualShock(3) - Right stick Left/right
#define  SER_RY          4            // DualShock(4) - Right Stick Up/Down
#define  SER_LX          5            // DualShock(5) - Left Stick Left/right
#define  SER_LY          6            // DualShock(6) - Left Stick Up/Down


#define cTravelDeadZone 4      //The deadzone for the analog input from the remote
#define  MAXPS2ERRORCNT  5     // How many times through the loop will we go before shutting off robot?

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================

// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller 

static short       g_BodyYOffset; 
static word        g_wSerialErrorCnt;
static short       g_BodyYShift;
static byte        ControlMode;
static word        g_wButtonsPrev;
static bool        DoubleHeightOn;
static bool        DoubleTravelOn;
static bool        WalkMethod;
byte               GPSeq;             //Number of the sequence
short              g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet

// some external or forward function references.
extern void SerTurnRobotOff(void);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================
// If both PS2 and XBee are defined then we will become secondary to the xbee
void InputController::Init(void)
{
  int error;

  // May need to init the Serial port here...
  SerSerial.begin(SERIAL_BAUD);
  
  g_BodyYOffset = 0;    
  g_BodyYShift = 0;
  g_wSerialErrorCnt = 0;  // error count

  ControlMode = WALKMODE;
  DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;

  g_InControlState.SpeedControl = 100;    // Sort of migrate stuff in from Devon.
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void InputController::AllowControllerInterrupts(boolean fAllow)
{
  // We don't need to do anything...
  
}

#define ButtonPressed(wMask) (((wButtons & wMask) == 0) && ((g_wButtonsPrev & wMask) != 0))

//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
//process any commands.
//==============================================================================
void InputController::ControlInput(void)
{
  byte abDualShock[7];  // we will to receive 7 bytes of data with the first byte being the checksum
  unsigned long ulLastChar;
  boolean fAdjustLegPositions = false;
  word wButtons;

  SerSerial.print("Rd");

  // we will loop through reading our 7 bytes
  ulLastChar = millis();
  for (byte i=0; i < 7; i++) {
    while  (SerSerial.available() == 0) {
      if ((millis() - ulLastChar) > 200) {
        // We may have lost the serial communications 
        if (g_wSerialErrorCnt < MAXPS2ERRORCNT)
          g_wSerialErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.
        else if (g_InControlState.fHexOn)
          SerTurnRobotOff();
        return;  // 
      }      
    }

    abDualShock[i] = SerSerial.read();
    ulLastChar = millis();
  }  

  // Lets check the checksum...
  if (abDualShock[0] == (abDualShock[1] ^ abDualShock[2] ^ abDualShock[3] ^ abDualShock[4] ^ abDualShock[5] ^ abDualShock[6])) {

    wButtons = (abDualShock[1] << 8) | abDualShock[2];

    // In an analog mode so should be OK...
    g_wSerialErrorCnt = 0;    // clear out error count...

    if (ButtonPressed(SERB_START)) {// OK lets try "0" button for Start. 
      if (g_InControlState.fHexOn) {
        SerTurnRobotOff();
      } 
      else {
        //Turn on
        g_InControlState.fHexOn = 1;
        fAdjustLegPositions = true;
      }
    }

    if (g_InControlState.fHexOn) {
      // [SWITCH MODES]

      //Translate mode
      if (ButtonPressed(SERB_L1)) {// L1 Button Test
        MSound( 1, 50, 2000);  
        if (ControlMode != TRANSLATEMODE )
          ControlMode = TRANSLATEMODE;
        else {
          if (g_InControlState.SelectedLeg==255) 
            ControlMode = WALKMODE;
          else
            ControlMode = SINGLELEGMODE;
        }
      }

      //Rotate mode
      if (ButtonPressed(SERB_L2)) {    // L2 Button Test
        MSound( 1, 50, 2000);
        if (ControlMode != ROTATEMODE)
          ControlMode = ROTATEMODE;
        else {
          if (g_InControlState.SelectedLeg == 255) 
            ControlMode = WALKMODE;
          else
            ControlMode = SINGLELEGMODE;
        }
      }

      //Single leg mode fNO
      if (ButtonPressed(SERB_CIRCLE)) {// O - Circle Button Test
        if (abs(g_InControlState.TravelLength.x)<cTravelDeadZone && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
          && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone )   {
          if (ControlMode != SINGLELEGMODE) {
            ControlMode = SINGLELEGMODE;
            if (g_InControlState.SelectedLeg == 255)  //Select leg if none is selected
              g_InControlState.SelectedLeg=cRF; //Startleg
          } 
          else {
            ControlMode = WALKMODE;
            g_InControlState.SelectedLeg=255;
          }
        }
      }      

#ifdef OPT_GPPLAYER
      // GP Player Mode X
      if (ButtonPressed(SERB_CROSS)) { // X - Cross Button Test
        MSound(1, 50, 2000);  
        if (ControlMode != GPPLAYERMODE) {
          ControlMode = GPPLAYERMODE;
          GPSeq=0;
        } 
        else
          ControlMode = WALKMODE;
      }
#endif // OPT_GPPLAYER

      //[Common functions]
      //Switch Balance mode on/off 
      if (ButtonPressed(SERB_SQUARE)) { // Square Button Test
        g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
        if (g_InControlState.BalanceMode) {
          MSound(1, 250, 1500); 
        } 
        else {
          MSound( 2, 100, 2000, 50, 4000);
        }
      }

      //Stand up, sit down  
      if (ButtonPressed(SERB_TRIANGLE)) { // Triangle - Button Test
        if (g_BodyYOffset>0) 
          g_BodyYOffset = 0;
        else
          g_BodyYOffset = 35;
        fAdjustLegPositions = true;
      }

      if (ButtonPressed(SERB_PAD_UP)) {// D-Up - Button Test
        g_BodyYOffset += 10;

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
        if (g_BodyYOffset > MAX_BODY_Y)
          g_BodyYOffset = MAX_BODY_Y;
      }

      if (ButtonPressed(SERB_PAD_DOWN) && g_BodyYOffset) {// D-Down - Button Test
        if (g_BodyYOffset > 10)
          g_BodyYOffset -= 10;
        else
          g_BodyYOffset = 0;      // constrain don't go less than zero.

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
      }

      if (ButtonPressed(SERB_PAD_RIGHT)) { // D-Right - Button Test
        if (g_InControlState.SpeedControl>0) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
          MSound( 1, 50, 2000);  
        }
      }

      if (ButtonPressed(SERB_PAD_LEFT)) { // D-Left - Button Test
        if (g_InControlState.SpeedControl<2000 ) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
          MSound( 1, 50, 2000); 
        }
      }

      //[Walk functions]
      if (ControlMode == WALKMODE) {
        //Switch gates
        if (ButtonPressed(SERB_SELECT)            // Select Button Test
        && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
        && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
          && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
          g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
          if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
            MSound( 1, 50, 2000); 
          } 
          else {
            MSound(2, 50, 2000, 50, 2250); 
            g_InControlState.GaitType = 0;
          }
          GaitSelect();
        }

        //Double leg lift height
        if (ButtonPressed(SERB_R1)) { // R1 Button Test
          MSound( 1, 50, 2000); 
          DoubleHeightOn = !DoubleHeightOn;
          if (DoubleHeightOn)
            g_InControlState.LegLiftHeight = 80;
          else
            g_InControlState.LegLiftHeight = 50;
        }

        //Double Travel Length
        if (ButtonPressed(SERB_R2)) {// R2 Button Test
          MSound(1, 50, 2000); 
          DoubleTravelOn = !DoubleTravelOn;
        }

        // Switch between Walk method 1 && Walk method 2
        if (ButtonPressed(SERB_R3)) { // R3 Button Test
          MSound(1, 50, 2000); 
          WalkMethod = !WalkMethod;
        }

        //Walking
        if (WalkMethod)  //(Walk Methode) 
          g_InControlState.TravelLength.z = (abDualShock[SER_RY]-128); //Right Stick Up/Down  

        else {
          g_InControlState.TravelLength.x = -(abDualShock[SER_LX] - 128);
          g_InControlState.TravelLength.z = (abDualShock[SER_LY] - 128);
        }

        if (!DoubleTravelOn) {  //(Double travel length)
          g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
          g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
        }

        g_InControlState.TravelLength.y = -(abDualShock[SER_RX] - 128)/4; //Right Stick Left/Right 
      }

      //[Translate functions]
      g_BodyYShift = 0;
      if (ControlMode == TRANSLATEMODE) {
        g_InControlState.BodyPos.x = (abDualShock[SER_LX] - 128)/2;
        g_InControlState.BodyPos.z = -(abDualShock[SER_LY] - 128)/3;
        g_InControlState.BodyRot1.y = (abDualShock[SER_RX] - 128)*2;
        g_BodyYShift = (-(abDualShock[SER_RY] - 128)/2);
      }

      //[Rotate functions]
      if (ControlMode == ROTATEMODE) {
        g_InControlState.BodyRot1.x = (abDualShock[SER_LY] - 128);
        g_InControlState.BodyRot1.y = (abDualShock[SER_RX] - 128)*2;
        g_InControlState.BodyRot1.z = (abDualShock[SER_LX] - 128);
        g_BodyYShift = (-(abDualShock[SER_RY] - 128)/2);
      }

      //[Single leg functions]
      if (ControlMode == SINGLELEGMODE) {
        //Switch leg for single leg control
        if (ButtonPressed(SERB_SELECT)) { // Select Button Test
          MSound(1, 50, 2000); 
          if (g_InControlState.SelectedLeg<5)
            g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
          else
            g_InControlState.SelectedLeg=0;
        }

        g_InControlState.SLLeg.x= (abDualShock[SER_LX] - 128)/2; //Left Stick Right/Left
        g_InControlState.SLLeg.y= (abDualShock[SER_RY] - 128)/10; //Right Stick Up/Down
        g_InControlState.SLLeg.z = (abDualShock[SER_LY] - 128)/2; //Left Stick Up/Down

        // Hold single leg in place
        if (ButtonPressed(SERB_R2)) { // R2 Button Test
          MSound(1, 50, 2000);  
          g_InControlState.fSLHold = !g_InControlState.fSLHold;
        }
      }

#ifdef OPT_GPPLAYER
      //[GPPlayer functions]
      if (ControlMode == GPPLAYERMODE) {

        // Lets try some speed control... Map all values if we have mapped some before
        // or start mapping if we exceed some minimum delta from center
        // Have to keep reminding myself that commander library already subtracted 128...
        if (g_ServoDriver.FIsGPSeqActive() ) {
          if ((g_sGPSMController != 32767)  
            || (abDualShock[SER_RY] > (128+16)) || (abDualShock[SER_RY] < (128-16)))
          {
            // We are in speed modify mode...
            short sNewGPSM = map(abDualShock[SER_RY], 0, 255, -200, 200);
            if (sNewGPSM != g_sGPSMController) {
              g_sGPSMController = sNewGPSM;
              g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
            }

          }
        }

        //Switch between sequences
        if (ButtonPressed(SERB_SELECT)) { // Select Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
            if (GPSeq < 5) {  //Max sequence
              MSound(1, 50, 1500);
              GPSeq = GPSeq+1;
            } 
            else {
              MSound(2, 50, 2000, 50, 2250);
              GPSeq=0;
            }
          }
        }
        //Start Sequence
        if (ButtonPressed(SERB_R2))// R2 Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
            g_ServoDriver.GPStartSeq(GPSeq);
            g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
          }
          else {
            g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
            MSound (2, 50, 2000, 50, 2000);
          }


      }
#endif // OPT_GPPLAYER

      //Calculate walking time delay
      g_InControlState.InputTimeDelay = 128 - max(max(abs(abDualShock[SER_LX] - 128), abs(abDualShock[SER_LY] - 128)), abs(abDualShock[SER_RX] - 128));
    }

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
    if (fAdjustLegPositions)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file

  // remember which buttons were set here
  g_wButtonsPrev = wButtons;

  } 
  else {
    // We may have lost the PS2... See what we can do to recover...
    if (g_wSerialErrorCnt < MAXPS2ERRORCNT)
      g_wSerialErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.
    else if (g_InControlState.fHexOn)
      SerTurnRobotOff();
  }
}

//==============================================================================
// SerTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void SerTurnRobotOff(void)
{
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
  g_InControlState.fHexOn = 0;
  AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
}






