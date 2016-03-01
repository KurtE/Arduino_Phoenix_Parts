//#define DEBUG_COMMANDER

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
//Hardware setup: Arbotix Commander version - Try to emulate most of PS2, but PS2 has 16 buttons and Commander 
// has 10. so some things may not be there, others may be doubled up.
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
// Quick and Dirty description of controls... WIP
// In most cases I try to mention what button on the PS2 things coorespond to..
// On/OFF - Turning the commander 2 on and off (PS2 start button)
// R1 - options (Change walk gait, Change Leg in Single Leg, Change GP sequence) (Select on PS2)
// R2 - Toggle walk method...  Run Sequence in GP mode
// R3 - Walk method (Not done yet) - (PS2 R3)
// L4 - Ballance mode on and off
// L5 - Stand/Sit (Triangle on PS2)
// L6+Right Joy UP/DOWN - Body up/down - (PS2 Dpad Up/Down)
// L6+Right Joy Left/Right - Speed higher/lower - (PS2 DPad left/right)
// Right Top(S7) - Cycle through options of Normal walk/Double Height/Double Travel) - (PS2 R1, R2)
// Left Top(S8) - Cycle through modes (Walk, Translate, Rotate, Single Leg) (PS2: Circle, X, L1, L2)

// Note: Left some descriptions of PS2 stuff, especially parts still left to Map/Implement.
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
//[Single leg Controls] - Need to check...
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls] - How to do sequences???
//- selectSwitch Sequences
//- R2Start Sequence
//
//====================================================================
// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
//[CONSTANTS]
enum {
  WALKMODE=0, TRANSLATEMODE, ROTATEMODE, 
#ifdef OPT_SINGLELEG      
  SINGLELEGMODE, 
#endif
#ifdef OPT_GPPLAYER
  GPPLAYERMODE, 
#endif  
  MODECNT};
enum {
  NORM_NORM=0, NORM_LONG, HIGH_NORM, HIGH_LONG};


#define cTravelDeadZone 6      //The deadzone for the analog input from the remote

#define ARBOTIX_TO  1250        // if we don't get a valid message in this number of mills turn off

#ifndef XBeeSerial
SoftwareSerial XBeeSerial(cXBEE_IN, cXBEE_OUT);
#endif
#ifndef USER
#define USER 13
#endif

//=============================================================================
// I have included a modified version of the commander object here as I wish to
// decouple the usage of the joysticks from the names as well as on other robots
// I may not be using Serial as the the serial port I am communicating with
// So I also removed the Southpaw code.
//=============================================================================
/*
  Commander.h - Library for interfacing with ArbotiX Commander
 Copyright (c) 2009-2012 Michael E. Ferguson.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* bitmasks for buttons array */
#define BUT_R1      0x01
#define BUT_R2      0x02
#define BUT_R3      0x04
#define BUT_L4      0x08
#define BUT_L5      0x10
#define BUT_L6      0x20
#define BUT_RT      0x40
#define BUT_LT      0x80

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif


/* the Commander will send out a frame at about 30hz, this class helps decipher the output. */
class Commander
{    
public:
  Commander(); 
  void begin(unsigned long baud);
  int ReadMsgs();         // must be called regularly to clean out Serial buffer

  // joystick values are -125 to 125
  signed char rightV;      // vertical stick movement = forward speed
  signed char rightH;      // horizontal stick movement = sideways or angular speed
  signed char leftV;      // vertical stick movement = tilt    
  signed char leftH;      // horizontal stick movement = pan (when we run out of pan, turn body?)

  // buttons are 0 or 1 (PRESSED), and bitmapped
  unsigned char buttons;  // 
  unsigned char ext;      // Extended function set

    // Hooks are used as callbacks for button presses -- NOT IMPLEMENT YET

private:
  // internal variables used for reading messages
  unsigned char vals[7];  // temporary values, moved after we confirm checksum
  int index;              // -1 = waiting for new packet
  int checksum;
};


//=============================================================================
// Global - Local to this file only...
//=============================================================================
Commander command = Commander();
unsigned long g_ulLastMsgTime;
short  g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet
boolean g_fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)
#ifdef USEMULTI
//==============================================================================
//
// Lets define our Sub-class of the InputControllerClass
//
//==============================================================================
class CommanderInputController : 
public InputController
{
public:
  CommanderInputController();        // A reall simple constructor...

  virtual void     Init(void);
  virtual void     ControlInput(void);
  virtual void     AllowControllerInterrupts(boolean fAllow);

};


CommanderInputController g_CommanderController;


//==============================================================================
// Constructor. See if there is a simple way to have one or more Input
//     controllers. Maybe register at construction time
//==============================================================================
CommanderInputController::CommanderInputController()
{
  RegisterInputController(this);
}

#else
#define CommanderInputController InputController
// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller 
#endif



static short   g_BodyYOffset; 
static short   g_BodyYShift;
static byte    ControlMode;
static byte    HeightSpeedMode;
//static bool  DoubleHeightOn;
static bool    DoubleTravelOn;
static byte    bJoystickWalkMode;
byte           GPSeq;             //Number of the sequence

static byte    buttonsPrev;
static byte    extPrev;

// some external or forward function references.
extern void CommanderTurnRobotOff(void);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void CommanderInputController::Init(void)
{
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
#ifdef DBGSerial  
  DBGSerial.print("Commander Init: ");
  DBGSerial.println(XBEE_BAUD, DEC);
#endif  
  command.begin(XBEE_BAUD);
  GPSeq = 0;  // init to something...

  ControlMode = WALKMODE;
  HeightSpeedMode = NORM_NORM;
  //    DoubleHeightOn = false;
  DoubleTravelOn = false;
  bJoystickWalkMode = 0;
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void CommanderInputController::AllowControllerInterrupts(boolean fAllow)
{
  // We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
void CommanderInputController::ControlInput(void)
{
  // See if we have a new command available...
  if(command.ReadMsgs() > 0){
    // If we receive a valid message than turn robot on...
    boolean fAdjustLegPositions = false;
    short sLegInitXZAdjust = 0;
    short sLegInitAngleAdjust = 0;
    
    if (!g_InControlState.fRobotOn ) {
        g_InControlState.fRobotOn = true;
        fAdjustLegPositions = true;
    }

    // [SWITCH MODES]

    // Cycle through modes...
    if ((command.buttons & BUT_LT) && !(buttonsPrev & BUT_LT)) {
      if (++ControlMode >= MODECNT) {
        ControlMode = WALKMODE;    // cycled back around...
        MSound( 2, 50, 2000, 50, 3000); 
      } 
      else {
        MSound( 1, 50, 2000);  
      }
#ifdef OPT_SINGLELEG      
      if (ControlMode != SINGLELEGMODE)
        g_InControlState.SelectedLeg=255;
#endif
    }

    //[Common functions]
    //Switch Balance mode on/off 
    if ((command.buttons & BUT_L4) && !(buttonsPrev & BUT_L4)) {
      g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
      if (g_InControlState.BalanceMode) {
        MSound( 1, 250, 1500); 
      } 
      else {
        MSound( 2, 100, 2000, 50, 4000);
      }
    }

    //Stand up, sit down  
    if ((command.buttons & BUT_L5) && !(buttonsPrev & BUT_L5)) {
      if (g_BodyYOffset>0) 
        g_BodyYOffset = 0;
      else
        g_BodyYOffset = 35;
      fAdjustLegPositions = true;
      g_fDynamicLegXZLength = false;
    }

    // We will use L6 with the Right joystick to control both body offset as well as Speed...
    // We move each pass through this by a percentage of how far we are from center in each direction
    // We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
    // which give an idea, but only for those whoes robot has a speaker
    int lx = command.leftH;
    int ly = command.leftV;

    if (command.buttons & BUT_L6 ) {
      // raise or lower the robot on the joystick up /down
      // Maybe should have Min/Max
      int delta = command.rightV/25;   
      if (delta) {
        g_BodyYOffset = max(min(g_BodyYOffset + delta, MAX_BODY_Y), 0);
        fAdjustLegPositions = true;
      }
      
      // Also use right Horizontal to manually adjust the initial leg positions.
      sLegInitXZAdjust = lx/10;        // play with this.
      sLegInitAngleAdjust = ly/8;
      lx = 0;
      ly = 0;

      // Likewise for Speed control
      delta = command.rightH / 16;   // 
      if ((delta < 0) && g_InControlState.SpeedControl) {
        if ((word)(-delta) <  g_InControlState.SpeedControl)
          g_InControlState.SpeedControl += delta;
        else 
          g_InControlState.SpeedControl = 0;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl);  
      }
      if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
        g_InControlState.SpeedControl += delta;
        if (g_InControlState.SpeedControl > 2000)
          g_InControlState.SpeedControl = 2000;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl); 
      }

      command.rightH = 0; // don't walk when adjusting the speed here...
    }

#ifdef DBGSerial
    if ((command.buttons & BUT_R3) && !(buttonsPrev & BUT_R3)) {
      MSound(1, 50, 2000);
      g_fDebugOutput = !g_fDebugOutput;
    }
#endif    
    //[Walk functions]
    if (ControlMode == WALKMODE) {
      //Switch gates
      if (((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1))
        && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
      && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
        && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
        g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
        if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
          MSound( 1, 50, 2000);  
        } 
        else {
          MSound (2, 50, 2000, 50, 2250); 
          g_InControlState.GaitType = 0;
        }
        GaitSelect();
      }

      //Double leg lift height
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound( 1, 50, 2000);  
        HeightSpeedMode = (HeightSpeedMode + 1) & 0x3; // wrap around mode
        DoubleTravelOn = HeightSpeedMode & 0x1;
        if ( HeightSpeedMode & 0x2)
          g_InControlState.LegLiftHeight = 80;
        else
          g_InControlState.LegLiftHeight = 50;
      }

      // Switch between Walk method 1 && Walk method 2
      if ((command.buttons & BUT_R2) && !(buttonsPrev & BUT_R2)) {
#ifdef cTurretRotPin
        if ((++bJoystickWalkMode) > 2)
#else
          if ((++bJoystickWalkMode) > 1)
#endif 
            bJoystickWalkMode = 0;
        MSound (1, 50, 2000 + bJoystickWalkMode*250);
      }

      //Walking
      switch (bJoystickWalkMode) {
      case 0:    
        g_InControlState.TravelLength.x = -lx;
        g_InControlState.TravelLength.z = -ly;
        g_InControlState.TravelLength.y = -(command.rightH)/4; //Right Stick Left/Right 
        break;
      case 1:
        g_InControlState.TravelLength.z = (command.rightV); //Right Stick Up/Down  
        g_InControlState.TravelLength.y = -(command.rightH)/4; //Right Stick Left/Right 
        break;
#ifdef cTurretRotPin
      case 2:
        g_InControlState.TravelLength.x = -lx;
        g_InControlState.TravelLength.z = -ly;

        // Will use Right now stick to control turret.
        g_InControlState.TurretRotAngle1 =  max(min(g_InControlState.TurretRotAngle1+command.rightH/5, cTurretRotMax1), cTurretRotMin1);      // Rotation of turret in 10ths of degree
        g_InControlState.TurretTiltAngle1 =  max(min(g_InControlState.TurretTiltAngle1+command.rightV/5, cTurretTiltMax1), cTurretTiltMin1);  // tilt of turret in 10ths of degree
#endif
      }

      if (!DoubleTravelOn) {  //(Double travel length)
        g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
        g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
      }

    }

    //[Translate functions]
    g_BodyYShift = 0;
    if (ControlMode == TRANSLATEMODE) {
      g_InControlState.BodyPos.x =  SmoothControl(((lx)*2/3), g_InControlState.BodyPos.x, SmDiv);
      g_InControlState.BodyPos.z =  SmoothControl(((ly)*2/3), g_InControlState.BodyPos.z, SmDiv);
      g_InControlState.BodyRot1.y = SmoothControl(((command.rightH)*2), g_InControlState.BodyRot1.y, SmDiv);

      //      g_InControlState.BodyPos.x = (lx)/2;
      //      g_InControlState.BodyPos.z = -(ly)/3;
      //      g_InControlState.BodyRot1.y = (command.rightH)*2;
      g_BodyYShift = (-(command.rightV)/2);
    }

    //[Rotate functions]
    if (ControlMode == ROTATEMODE) {
      g_InControlState.BodyRot1.x = (ly);
      g_InControlState.BodyRot1.y = (command.rightH)*2;
      g_InControlState.BodyRot1.z = (lx);
      g_BodyYShift = (-(command.rightV)/2);
    }
#ifdef OPT_GPPLAYER
    //[GPPlayer functions]
    if (ControlMode == GPPLAYERMODE) {
      // Lets try some speed control... Map all values if we have mapped some before
      // or start mapping if we exceed some minimum delta from center
      // Have to keep reminding myself that commander library already subtracted 128...
      if (g_ServoDriver.FIsGPSeqActive() ) {
        if ((g_sGPSMController != 32767)  
          || (command.rightV > 16) || (command.rightV < -16))
        {
          // We are in speed modify mode...
          if (command.rightV >= 0)
            g_sGPSMController = map(command.rightV, 0, 127, 0, 200);
          else  
            g_sGPSMController = map(command.rightV, -127, 0, -200, 0);
          g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
        }
      }

      //Switch between sequences
      if ((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          if (GPSeq < 5) {  //Max sequence
            MSound (1, 50, 1500);  
            GPSeq = GPSeq+1;
          } 
          else {
            MSound (2, 50, 2000, 50, 2250);
            GPSeq=0;
          }
        }
      }
      //Start Sequence
      if ((command.buttons & BUT_R2) && !(buttonsPrev & BUT_R2)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          g_ServoDriver.GPStartSeq(GPSeq);
          g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
        }
        else {
          g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
          MSound (2, 50, 2000, 50, 2000);
        }
      }

    }
#endif // OPT_GPPLAYER

    //[Single leg functions]
#ifdef OPT_SINGLELEG      
    if (ControlMode == SINGLELEGMODE) {
      //Switch leg for single leg control
      if ((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
        MSound (1, 50, 2000);  
        if (g_InControlState.SelectedLeg<(CNT_LEGS-1))
          g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
        else
          g_InControlState.SelectedLeg=0;
      }

      g_InControlState.SLLeg.x= (byte)((int)lx+128)/2; //Left Stick Right/Left
      g_InControlState.SLLeg.y= (byte)((int)command.rightV+128)/10; //Right Stick Up/Down
      g_InControlState.SLLeg.z = (byte)((int)ly+128)/2; //Left Stick Up/Down

      // Hold single leg in place
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound (1, 50, 2000);  
        g_InControlState.fSLHold = !g_InControlState.fSLHold;
      }
    }
#endif

    //Calculate walking time delay
    g_InControlState.InputTimeDelay = 128 - max(max(abs(lx), abs(ly)), abs(command.rightH));

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = max(g_BodyYOffset + g_BodyYShift,  0);

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

    if (fAdjustLegPositions && !g_fDynamicLegXZLength)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file

    // Save away the buttons state as to not process the same press twice.
    buttonsPrev = command.buttons;
    extPrev = command.ext;
    g_ulLastMsgTime = millis();
  } 
  else {
    // We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
    if (g_InControlState.fRobotOn) {
      if ((millis() - g_ulLastMsgTime) > ARBOTIX_TO)
        CommanderTurnRobotOff();
    }
  }
}

//==============================================================================
// CommanderTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void CommanderTurnRobotOff(void)
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

#ifdef cTurretRotPin
  g_InControlState.TurretRotAngle1 = cTurretRotInit;      // Rotation of turrent in 10ths of degree
  g_InControlState.TurretTiltAngle1 = cTurretTiltInit;    // the tile for the turret
#endif

  g_fDynamicLegXZLength = false; // also make sure the robot is back in normal leg init mode...
}
//================================================================================
#ifdef OPT_TERMINAL_MONITOR_IC
// Optional stuff to allow me to have Input device debug support
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void CommanderInputController::ShowTerminalCommandList(void) 
{
  DBGSerial.println(F("X - Show XBee Info"));
}

//==============================================================================
// ProcessTerminalCommand: The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
void PrintXBeeIDInfo(char *pszID) {
  char ab[20];
  int cbRead;
  while (XBeeSerial.read() != -1)
    ;  // Flush out anything still pending. 
  XBeeSerial.print("AT");  
  XBeeSerial.println(pszID);  // Lets print out the ID;
  XBeeSerial.flush();
  cbRead = XBeeSerial.readBytesUntil('\r', ab, sizeof(ab));
  if (cbRead) {
    DBGSerial.print(pszID);
    DBGSerial.print(": ");
    DBGSerial.write(ab, cbRead);
    DBGSerial.println();
  }
}  

boolean CommanderInputController::ProcessTerminalCommand(byte *psz, byte bLen)
{
  if ((bLen == 1) && ((*psz == 'x') || (*psz == 'X'))) {
    char ab[10];
    int cbRead;
    delay(15);  // see if we have fast command mode enabled.
    XBeeSerial.print(F("+++")); 
    XBeeSerial.flush();
    XBeeSerial.setTimeout(20);  // give a little extra time
    if (XBeeSerial.readBytesUntil('\r', ab, 10) > 0) {
      // Ok we entered command mode, lets print out a few things about the XBee
      PrintXBeeIDInfo("MY");
      PrintXBeeIDInfo("DL");
      PrintXBeeIDInfo("ID");
      PrintXBeeIDInfo("EA");
      PrintXBeeIDInfo("EC");

      XBeeSerial.println("ATCN");  // exit command mode
      cbRead = XBeeSerial.readBytesUntil('\r', ab, sizeof(ab));
    } 
    else {
      DBGSerial.println("XBee Failed to enter command mode");
    }    

    return true;  
  } 
  return false;

}
#endif
//===============================================================================

//==============================================================================
// The below class code is based on the commander class by Michael Ferguson... 
// I included and updated for my own usage...  As I may not always use 
// Serial and I wish to decouple usage of joysticks from the names...
//==============================================================================

/*
  Commander.cpp - Library for interfacing with ArbotiX Commander
 Copyright (c) 2009-2012 Michael E. Ferguson.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

//==============================================================================
// Commander::Commander - Constructor
//==============================================================================
Commander::Commander(){
  index = -1;
}

//==============================================================================
// Commander::begin 
//==============================================================================
void Commander::begin(unsigned long baud){
  char ab[10];
  // Sometimes when we power up the XBee comes up at 9600 in command mode
  // There is an OK<cr>.  So check for this and try to exit
#ifdef NOT_SURE_WHY_NEEDED_SOMETIMES
  XBeeSerial.begin(9600);  
  XBeeSerial.println(F("ATCN"));  // Tell it to bail quickly
  delay(25);
  XBeeSerial.end();
  delay(25);
#endif  
  XBeeSerial.begin(baud);
  pinMode(USER, OUTPUT);
#ifdef CHECK_AND_CONFIG_XBEE
  while (XBeeSerial.read() != -1)
    ;  // flush anything out...
  // First lets see if we have a real short command time
  delay(15);  // see if we have fast command mode enabled.
  XBeeSerial.print(F("+++")); 
  XBeeSerial.flush();
  XBeeSerial.setTimeout(20);  // give a little extra time
  if (XBeeSerial.readBytesUntil('\r', ab, 10) > 0) {
    XBeeSerial.println(F("ATCN"));	          // and exit command mode
    return;  // bail out quick
  }
  // Else see if maybe properly configured but not quick command mode.
  delay(1100);
  while (XBeeSerial.read() != -1)
    ;  // flush anything out...
  XBeeSerial.print(F("+++"));
  XBeeSerial.setTimeout(1100);  // little over a second
  if (XBeeSerial.readBytesUntil('\r', ab, 10) > 0) {
    // Note: we could check a few more things here if we run into issues.  Like: MY!=0
    // or MY != DL
    XBeeSerial.println(F("ATGT 5"));              // Set a quick command mode
    XBeeSerial.println(F("ATWR"));	          // Write out the changes
    XBeeSerial.println(F("ATCN"));	          // and exit command mode
    return;  // It is already at 38400, so assume already init.
  }
  // Failed, so check to see if we can communicate at 9600
  XBeeSerial.end();
  XBeeSerial.begin(9600);
  while (XBeeSerial.read() != -1)
    ;  // flush anything out...

  delay(1100);
  XBeeSerial.print(F("+++"));
  if (XBeeSerial.readBytesUntil('\r', ab, 10) == 0) {
    // failed blink fast
    for(int i=0;i<50;i++) {
      digitalWrite(USER, !digitalRead(USER));
      delay(50);
    }  // Loop awhile
  } 
  else {
    // So we entered command mode, lets set the appropriate stuff. 
    XBeeSerial.println(F("ATBD 5"));  // 38400
    XBeeSerial.print(F("ATID "));
    XBeeSerial.println(DEFAULT_ID, HEX);

    XBeeSerial.print(F("ATMY "));
    XBeeSerial.println(DEFAULT_MY, HEX);

    XBeeSerial.println(F("ATDH 0"));
    XBeeSerial.print(F("ATDL "));
    XBeeSerial.println(DEFAULT_DL, HEX);

    XBeeSerial.println(F("ATGT 5"));    // Set a quick command mode
    XBeeSerial.println(F("ATWR"));	// Write out the changes
    XBeeSerial.println(F("ATCN"));	// and exit command mode
    XBeeSerial.flush();              // make sure all has been output
    // lets do a quick and dirty test
    delay(250);  // Wait a bit for responses..
  }
  XBeeSerial.end();
  delay(10);
  XBeeSerial.begin(38400);
#endif  

}

//==============================================================================
// ReadMsgs
//==============================================================================

/* process messages coming from Commander 
 *  format = 0xFF RIGHT_H RIGHT_V LEFT_H LEFT_V BUTTONS EXT CHECKSUM */
int Commander::ReadMsgs(){
  while(XBeeSerial.available() > 0){
    if(index == -1){         // looking for new packet
      if(XBeeSerial.read() == 0xff){
        index = 0;
        checksum = 0;
      }
    }
    else if(index == 0){
      vals[index] = (unsigned char) XBeeSerial.read();
      if(vals[index] != 0xff){            
        checksum += (int) vals[index];
        index++;
      }
    }
    else{
      vals[index] = (unsigned char) XBeeSerial.read();
      checksum += (int) vals[index];
      index++;
      if(index == 7){ // packet complete
        if(checksum%256 != 255){
#ifdef DEBUG_COMMANDER
#ifdef DBGSerial  
          if (g_fDebugOutput) {
            DBGSerial.println("Packet Error");
          }
#endif          
#endif
          // packet error!
          index = -1;
          return 0;
        }
        else{
          digitalWrite(USER, digitalRead(USER)? LOW : HIGH);
          rightV = (signed char)( (int)vals[0]-128 );
          rightH = (signed char)( (int)vals[1]-128 );
          leftV = (signed char)( (int)vals[2]-128 );
          leftH = (signed char)( (int)vals[3]-128 );
          buttons = vals[4];
          ext = vals[5];
#ifdef DEBUG_COMMANDER
#ifdef DBGSerial  
          if (g_fDebugOutput) {
            DBGSerial.print(buttons, HEX);
            DBGSerial.print(" : ");
            DBGSerial.print(rightV, DEC);
            DBGSerial.print(" ");
            DBGSerial.print(rightH, DEC);
            DBGSerial.print(" ");
            DBGSerial.print(leftV, DEC);
            DBGSerial.print(" ");
            DBGSerial.println(leftH, DEC);
          }
#endif
#endif
        }
        index = -1;
        while (XBeeSerial.read() != -1)
          ;
        return 1;
      }
    }
  }
  return 0;
}
//==============================================================================
//==============================================================================

