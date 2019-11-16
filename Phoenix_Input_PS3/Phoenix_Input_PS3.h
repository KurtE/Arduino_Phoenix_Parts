//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 12-11-20019
//Programmer: Jeroen Janssen (aka Xan)
//            Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//            Andrew Klimovski - Added ESP32 and PS3 support
//Hardware setup: PS3 version
// 
//NEW IN V1.0
//- First Release
//
//Walk method 1:
//- Left StickWalk/Strafe
//- Right StickRotate
//
//Walk method 2:
//- Left StickDisable
//- Right StickWalk/Rotate
//
//
//PS3 CONTROLS:
//[Common Controls]
//- StartTurn on/off the bot
//- L1Toggle Shift mode
//- L2Toggle Rotate mode
//- CircleToggle Single leg mode
//   - Square        Toggle Balance mode
//- TriangleMove body to 35 mm from the ground (walk pos) 
//and back to the ground
//- D-Pad upBody up 10 mm
//- D-Pad downBody down 10 mm
//- D-Pad leftdecrease speed with 50mS
//- D-Pad rightincrease speed with 50mS
//
// Optional: L3 button down, Left stick can adjust leg positions...
// or if OPT_SINGLELEG is not defined may try using Circle

//
//
//[Walk Controls]
//- selectSwitch gaits
//- Left Stick(Walk mode 1) Walk/Strafe
// (Walk mode 2) Disable
//- Right Stick(Walk mode 1) Rotate, 
//(Walk mode 2) Walk/Rotate
//- R1Toggle Double gait travel speed
//- R2Toggle Double gait travel length
//
//[Shift Controls]
//- Left StickShift body X/Z
//- Right StickShift body Y and rotate body Y
//
//[Rotate Controls]
//- Left StickRotate body X/Z
//- Right StickRotate body Y
//
//[Single leg Controls]
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls]
//- selectSwitch Sequences
//- R2Start Sequence
//
//====================================================================
// [Include files]
#include <Arduino.h> // Arduino 1.0
#include <M5StickC.h>
#include <Ps3Controller.h>

//[CONSTANTS]
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4


#define cTravelDeadZone 4      //The deadzone for the analog input from the remote
#define  MAXPS3ERRORCNT  5     // How many times through the loop will we go before shutting off robot?

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

#define CTRL_CLK    2

#ifndef singleTravelLength
#define singleTravelLength 10	//1 decimal place
#endif

#ifndef doubleTravelLength
#define doubleTravelLength 20	//1 decimal place
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================


// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller 


static short       g_BodyYOffset; 
static short       g_sPS3ErrorCnt;
static short       g_BodyYShift;
static byte        ControlMode;
static bool        DoubleHeightOn;
static bool        DoubleTravelOn;
static bool        WalkMethod;
byte               GPSeq;                //Number of the sequence
short              g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet

// some external or forward function references.
extern void PS3TurnRobotOff(void);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS3 controller
//process any commands.
//==============================================================================

// If both PS3 and XBee are defined then we will become secondary to the xbee
void InputController::Init(void)
{
  int error;

  // Setup the PS3 controller
  //Ps3.attach(ControlInput);
  //Ps3.attach(joystickCallback);
  //Ps3.attachOnConnect(connectCallback);
  //Ps3.attachOnDisconnect(disconnectCallback);
  
  //extern char ps3ControllerMacAddr[];
  //Ps3.begin(ps3ControllerMacAddr);
  //Ps3.begin("2c:81:58:a9:8d:76");
  Ps3.begin();
  
#ifdef DBGSerial
	DBGSerial.print("PS3 Init: ");
	DBGSerial.println(error, DEC);
#endif
  g_BodyYOffset = 0;    
  g_BodyYShift = 0;
  g_sPS3ErrorCnt = 0;  // error count

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

//==============================================================================
// This is The main code to input function to read inputs from the PS3 and then
//process any commands.
//==============================================================================
#ifdef OPT_DYNAMIC_ADJUST_LEGS  
boolean g_fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)
#endif

void InputController::ControlInput(void)
{
  boolean fAdjustLegPositions = false;

  if (Ps3.isConnected()){
#ifdef DBGSerial
#ifdef DEBUG_PS3_INPUT
	if (g_fDebugOutput) {
		DBGSerial.print("PS3 Input: ");
		DBGSerial.print(Ps3.data.button, HEX);
		DBGSerial.print(":");
		DBGSerial.print(Ps3.data.analog.stick.lx, DEC);
		DBGSerial.print(" ");
		DBGSerial.print(Ps3.data.analog.stick.ly, DEC);
		DBGSerial.print(" ");
		DBGSerial.print(Ps3.data.analog.stick.rx, DEC);
		DBGSerial.print(" ");
		DBGSerial.println(Ps3.data.analog.stick.ry, DEC);
	}
#endif
#endif

#ifdef OPT_DYNAMIC_ADJUST_LEGS  
    boolean fAdjustLegPositions = false;
    short sLegInitXZAdjust = 0;
    short sLegInitAngleAdjust = 0;
#endif
    // In an analog mode so should be OK...
    g_sPS3ErrorCnt = 0;    // clear out error count...

    if (Ps3.data.button.start) {// OK lets press start button
      if (g_InControlState.fRobotOn) {
        PS3TurnRobotOff();
      } 
      else {
        //Turn on
        g_InControlState.fRobotOn = 1;
        fAdjustLegPositions = true;
      }
    }

    if (g_InControlState.fRobotOn) {
      // [SWITCH MODES]

      //Translate mode
      if (Ps3.data.button.l1) {// L1 Button Test
        MSound( 1, 50, 2000);  
        if (ControlMode != TRANSLATEMODE )
          ControlMode = TRANSLATEMODE;
        else {
#ifdef OPT_SINGLELEG
          if (g_InControlState.SelectedLeg==255) 
            ControlMode = WALKMODE;
          else
#endif
            ControlMode = SINGLELEGMODE;
        }
      }

      //Rotate mode
      if (Ps3.data.button.l2) {    // L2 Button Test
        MSound( 1, 50, 2000);
        if (ControlMode != ROTATEMODE)
          ControlMode = ROTATEMODE;
        else {
#ifdef OPT_SINGLELEG
          if (g_InControlState.SelectedLeg == 255) 
            ControlMode = WALKMODE;
          else
#endif
            ControlMode = SINGLELEGMODE;
        }
      }

      //Single leg mode fNO
#ifdef OPT_SINGLELEG
      if (Ps3.data.button.circle) {// O - Circle Button Test
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
#endif
#ifdef OPT_GPPLAYER
      // GP Player Mode X
      if (Ps3.data.button.cross) { // X - Cross Button Test
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
      if (Ps3.data.button.square) { // Square Button Test
        g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
        if (g_InControlState.BalanceMode) {
          MSound(1, 250, 1500); 
        } 
        else {
          MSound( 2, 100, 2000, 50, 4000);
        }
      }

      //Stand up, sit down  
      if (Ps3.data.button.triangle) { // Triangle - Button Test
        if (g_BodyYOffset>0) 
          g_BodyYOffset = 0;
        else
          g_BodyYOffset = 35;
        fAdjustLegPositions = true;
      }

      if (Ps3.data.button.up) {// D-Up - Button Test
        g_BodyYOffset += 10;

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
        if (g_BodyYOffset > MAX_BODY_Y)
          g_BodyYOffset = MAX_BODY_Y;
      }

      if ((Ps3.data.button.down) && g_BodyYOffset) {// D-Down - Button Test
        if (g_BodyYOffset > 10)
          g_BodyYOffset -= 10;
        else
          g_BodyYOffset = 0;      // constrain don't go less than zero.

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
      }

      if (Ps3.data.button.right) { // D-Right - Button Test
        if (g_InControlState.SpeedControl>0) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
          MSound( 1, 50, 2000);  
        }
      }

      if (Ps3.data.button.left) { // D-Left - Button Test
        if (g_InControlState.SpeedControl<2000 ) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
          MSound( 1, 50, 2000); 
        }
      }
      
      // We are optionally going to allow the user to modify the Initial Leg positions, when they
      // press the L3 button.
      byte lx = Ps3.data.analog.stick.lx;
      byte ly = Ps3.data.analog.stick.ly;
#ifdef OPT_DYNAMIC_ADJUST_LEGS  
#ifdef OPT_SINGLELEG
      if (Ps3.data.button.l3) {    // L3 pressed, use this to modify leg positions.
#else
      if (Ps3.data.button.circle) {// O - Circle Button Test 
#endif      
        sLegInitXZAdjust = ((int)lx-128)/10;        // play with this.
        sLegInitAngleAdjust = ((int)ly-128)/8;
        lx = 0;
        ly = 0;
      }
#endif

      //[Walk functions]
      if (ControlMode == WALKMODE) {
        //Switch gates
        if (Ps3.data.button.select            // Select Button Test
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
        if (Ps3.data.button.r1) { // R1 Button Test
          MSound( 1, 50, 2000); 
          DoubleHeightOn = !DoubleHeightOn;
          if (DoubleHeightOn)
            g_InControlState.LegLiftHeight = 80;
          else
            g_InControlState.LegLiftHeight = 50;
        }

        //Double Travel Length
        if (Ps3.data.button.r2) {// R2 Button Test
          MSound(1, 50, 2000); 
          DoubleTravelOn = !DoubleTravelOn;
        }

        // Switch between Walk method 1 && Walk method 2
        if (Ps3.data.button.r3) { // R3 Button Test
          MSound(1, 50, 2000); 
          WalkMethod = !WalkMethod;
        }

        //Walking
        if (WalkMethod)  //(Walk Methode) 
          g_InControlState.TravelLength.z = (Ps3.data.analog.stick.ry-128); //Right Stick Up/Down  

        else {
          g_InControlState.TravelLength.x = -(lx - 128);
          g_InControlState.TravelLength.z = (ly - 128);
		  
		  g_InControlState.TravelLength.x = (int)(g_InControlState.TravelLength.x/(float)(doubleTravelLength/10.0));
          g_InControlState.TravelLength.z = (int)(g_InControlState.TravelLength.z/(float)(doubleTravelLength/10.0));
        }

        if (!DoubleTravelOn) {  //(Double travel length)
          g_InControlState.TravelLength.x = (int)(g_InControlState.TravelLength.x/(float)(singleTravelLength/10.0));
          g_InControlState.TravelLength.z = (int)(g_InControlState.TravelLength.z/(float)(singleTravelLength/10.0));
        }

        g_InControlState.TravelLength.y = -(Ps3.data.analog.stick.rx - 128)/4; //Right Stick Left/Right 
      }

      //[Translate functions]
      g_BodyYShift = 0;
      if (ControlMode == TRANSLATEMODE) {
        g_InControlState.BodyPos.x = (lx - 128)/2;
        g_InControlState.BodyPos.z = -(ly - 128)/3;
        g_InControlState.BodyRot1.y = (Ps3.data.analog.stick.rx - 128)*2;
        g_BodyYShift = (-(Ps3.data.analog.stick.ry - 128)/2);
      }

      //[Rotate functions]
      if (ControlMode == ROTATEMODE) {
        g_InControlState.BodyRot1.x = (ly - 128);
        g_InControlState.BodyRot1.y = (Ps3.data.analog.stick.rx - 128)*2;
        g_InControlState.BodyRot1.z = (lx - 128);
        g_BodyYShift = (-(Ps3.data.analog.stick.ry - 128)/2);
      }

      //[Single leg functions]
#ifdef OPT_SINGLELEG
      if (ControlMode == SINGLELEGMODE) {
        //Switch leg for single leg control
        if (Ps3.data.button.select) { // Select Button Test
          MSound(1, 50, 2000); 
          if (g_InControlState.SelectedLeg<(CNT_LEGS-1))
            g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
          else
            g_InControlState.SelectedLeg=0;
        }

        g_InControlState.SLLeg.x= (lx - 128)/2; //Left Stick Right/Left
        g_InControlState.SLLeg.y= (Ps3.data.analog.stick.ry - 128)/10; //Right Stick Up/Down
        g_InControlState.SLLeg.z = (ly - 128)/2; //Left Stick Up/Down

        // Hold single leg in place
        if (Ps3.data.button.r2) { // R2 Button Test
          MSound(1, 50, 2000);  
          g_InControlState.fSLHold = !g_InControlState.fSLHold;
        }
      }
#endif
#ifdef OPT_GPPLAYER
      //[GPPlayer functions]
      if (ControlMode == GPPLAYERMODE) {

        // Lets try some speed control... Map all values if we have mapped some before
        // or start mapping if we exceed some minimum delta from center
        // Have to keep reminding myself that commander library already subtracted 128...
        if (g_ServoDriver.FIsGPSeqActive() ) {
          if ((g_sGPSMController != 32767)  
            || (Ps3.data.analog.stick.ry > (128+16)) || (Ps3.data.analog.stick.ry < (128-16)))
          {
            // We are in speed modify mode...
            short sNewGPSM = map(Ps3.data.analog.stick.ry, 0, 255, -200, 200);
            if (sNewGPSM != g_sGPSMController) {
              g_sGPSMController = sNewGPSM;
              g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
            }

          }
        }

        //Switch between sequences
        if (Ps3.data.button.select) { // Select Button Test
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
        if (Ps3.data.button.r2)// R2 Button Test
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
      g_InControlState.InputTimeDelay = 128 - max(max(abs(lx - 128), abs(ly - 128)), abs(Ps3.data.analog.stick.rx - 128));
    }

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
    
#ifdef OPT_DYNAMIC_ADJUST_LEGS  
    if (sLegInitXZAdjust || sLegInitAngleAdjust) {
      // User asked for manual leg adjustment - only do when we have finished any previous adjustment

      if (!g_InControlState.ForceGaitStepCnt) {
        if (sLegInitXZAdjust)
          g_fDynamicLegXZLength = true;

        sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
        // Handle maybe change angles...
        if (sLegInitAngleAdjust) 
            RotateLegInitAngles(sLegInitAngleAdjust);
        // Give system time to process previous calls
        AdjustLegPositions(sLegInitXZAdjust);
      }
    }    
#endif
    
    if (fAdjustLegPositions)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
  } 
  else {
    // We may have lost the PS3... See what we can do to recover...
    if (g_sPS3ErrorCnt < MAXPS3ERRORCNT)
      g_sPS3ErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.
    else if (g_InControlState.fRobotOn)
      PS3TurnRobotOff();
    Ps3.begin(ps3ControllerMacAddr);
  }
}

//==============================================================================
// PS3TurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void PS3TurnRobotOff(void)
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
#ifdef OPT_SINGLELEG
  g_InControlState.SelectedLeg = 255;
#endif  
  g_InControlState.fRobotOn = 0;
  AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
}
