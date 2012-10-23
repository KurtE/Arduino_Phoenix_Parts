
//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the Hex Robot.
//    This Header file is specific for T-Hex with 3 DOF
//  
//    This version of the Configuration file is set up to run on the
//    Lynxmotion BotboardDuino board, which is similar to the Arduino Duemilanove
//
//    This version of configuration file assumes that the servos will be controlled
//    by a Lynxmotion Servo controller SSC-32 and the user is using a Lynxmotion 
//    PS2 to control the robot.
//
//Date: March 18, 2012
//Programmer: Kurt (aka KurtE)
//
//
//NEW IN V1.0
//   - First Release
//
//====================================================================
#ifndef HEX_CFG_THEX3_H
#define HEX_CFG_THEX3_H
//[CONDITIONAL COMPILING] - COMMENT IF NOT WANTED
// Define other optional compnents to be included or not...
#define OPT_TERMINAL_MONITOR  

#ifdef OPT_TERMINAL_MONITOR   // turning off terminal monitor will turn these off as well...
#define OPT_SSC_FORWARDER  // only useful if terminal monitor is enabled
//#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
#endif

#define OPT_GPPLAYER

// Which type of control(s) do you want to compile in
#define DBGSerial         Serial
//#define DEBUG_IOPINS

#if defined(UBRR1H)
#define SSCSerial         Serial1
#else
#endif

#define USEPS2

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
// THex-3
//==================================================================================================================================
#define USE_SSC32
//#define	cSSC_BINARYMODE	1			// Define if your SSC-32 card supports binary mode.

//[SERIAL CONNECTIONS]


// Warning I will undefine some components as the non-megas don't have enough memory...
//#undef OPT_FIND_SERVO_OFFSETS 

#define cSSC_BAUD        38400   //SSC32 BAUD rate

//--------------------------------------------------------------------
//[Botboarduino Pin Numbers]
#define SOUND_PIN    5        // Botboarduino JR pin number
#define PS2_DAT      6        
#define PS2_CMD      7
#define PS2_SEL      8
#define PS2_CLK      9
// If we are using a SSC-32 then:
// If were are running on an Arduino Mega we will use one of the hardware serial port, default to Serial1 above.
// If on Non mega, if the IO pins are set to 0, we will overload the hardware Serial port 
// Else we will user SoftwareSerial to talk to the SSC-32
#define cSSC_OUT     12      	//Output pin for (SSC32 RX) on BotBoard (Yellow)
#define cSSC_IN      13      	//Input pin for (SSC32 TX) on BotBoard (Blue)

//====================================================================
//[SSC PIN NUMBERS]
#define cRFCoxaPin      0   //Front Right leg Hip Horizontal
#define cRFFemurPin     1   //Front Right leg Hip Vertical
#define cRFTibiaPin     2   //Front Right leg Knee
#define cRFTarsPin      3   // Tar

#define cRMCoxaPin      4   //Middle Right leg Hip Horizontal
#define cRMFemurPin     5   //Middle Right leg Hip Vertical
#define cRMTibiaPin     6   //Middle Right leg Knee
#define cRMTarsPin      7   // Tar

#define cRRCoxaPin      8   //Rear Right leg Hip Horizontal
#define cRRFemurPin     9   //Rear Right leg Hip Vertical
#define cRRTibiaPin     10   //Rear Right leg Knee
#define cRRTarsPin      11   // Tar

#define cLFCoxaPin      16   //Front Left leg Hip Horizontal
#define cLFFemurPin     17   //Front Left leg Hip Vertical
#define cLFTibiaPin     18   //Front Left leg Knee
#define cLFTarsPin      19   // Tar

#define cLMCoxaPin      20   //Middle Left leg Hip Horizontal
#define cLMFemurPin     21   //Middle Left leg Hip Vertical
#define cLMTibiaPin     22   //Middle Left leg Knee
#define cLMTarsPin      23   // Tar

#define cLRCoxaPin      24   //Rear Left leg Hip Horizontal
#define cLRFemurPin     25   //Rear Left leg Hip Vertical
#define cLRTibiaPin     26   //Rear Left leg Knee
#define cLRTarsPin      27   // Tar


//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1     -550      //Mechanical limits of the Right Rear Leg
#define cRRCoxaMax1     550
#define cRRFemurMin1    -900
#define cRRFemurMax1    550
#define cRRTibiaMin1    -400
#define cRRTibiaMax1    750
#define cRRTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRRTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cRMCoxaMin1     -550      //Mechanical limits of the Right Middle Leg
#define cRMCoxaMax1     550
#define cRMFemurMin1    -900
#define cRMFemurMax1    550
#define cRMTibiaMin1    -400
#define cRMTibiaMax1    750
#define cRMTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRMTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cRFCoxaMin1     -550      //Mechanical limits of the Right Front Leg
#define cRFCoxaMax1     550
#define cRFFemurMin1    -900
#define cRFFemurMax1    550
#define cRFTibiaMin1    -400
#define cRFTibiaMax1    750
#define cRFTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRFTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLRCoxaMin1     -550      //Mechanical limits of the Left Rear Leg
#define cLRCoxaMax1     550
#define cLRFemurMin1    -900
#define cLRFemurMax1    550
#define cLRTibiaMin1    -400
#define cLRTibiaMax1    750
#define cLRTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLRTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLMCoxaMin1     -550      //Mechanical limits of the Left Middle Leg
#define cLMCoxaMax1     550
#define cLMFemurMin1    -900
#define cLMFemurMax1    550
#define cLMTibiaMin1    -400
#define cLMTibiaMax1    750
#define cLMTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLMTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLFCoxaMin1     -550      //Mechanical limits of the Left Front Leg
#define cLFCoxaMax1     550
#define cLFFemurMin1    -900
#define cLFFemurMax1    550
#define cLFTibiaMin1    -400
#define cLFTibiaMax1    750
#define cLFTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLFTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     29    // This is for TH3-R legs
#define cXXFemurLength    76
#define cXXTibiaLength    104
#define cXXTarsLength     85    // 4DOF only...

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength
#define cRRTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cRMCoxaLength     cXXCoxaLength	    //Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength
#define cRMTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength
#define cRFTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength
#define cLRTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLMCoxaLength     cXXCoxaLength	    //Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength
#define cLMTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength
#define cLFTarsLength	  cXXTarsLength	    //4DOF ONLY


//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1   -450   //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    450      //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -450   //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    450      //Default Coxa setup angle, decimals = 1

#define cRROffsetX      -53     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      102     //Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX      -72    //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ      0       //Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX      -60     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -102    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      53      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      102     //Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX      72     //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ      0       //Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX      60      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -102    //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ	 105 
#define CHexInitXZCos45  74        // COS(45) = .7071
#define CHexInitXZSin45  74    // sin(45) = .7071
#define CHexInitY	 26

// Lets try some multi leg positions depending on height settings.
#define CNT_HEX_INITS 3
#define MAX_BODY_Y  90
#ifdef DEFINE_HEX_GLOBALS
const byte g_abHexIntXZ[] PROGMEM = {
  cHexInitXZ, 99, 86};
const byte g_abHexMaxBodyY[] PROGMEM = { 
  20, 50, MAX_BODY_Y};
#else
extern const byte g_abHexIntXZ[] PROGMEM;
extern const byte g_abHexMaxBodyY[] PROGMEM;
#endif

#define cRRInitPosX     CHexInitXZCos45      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin45

#define cRMInitPosX     cHexInitXZ      //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos45      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin45

#define cLRInitPosX     CHexInitXZCos45      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin45

#define cLMInitPosX     cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos45      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin45
//--------------------------------------------------------------------
//[Tars factors used in formula to calc Tarsus angle relative to the ground]
#define cTarsConst	720	//4DOF ONLY
#define cTarsMulti	2	//4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

#endif CFG_HEX_H



