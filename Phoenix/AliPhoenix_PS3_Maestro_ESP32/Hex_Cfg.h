
//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the Hex Robot.
//    This Header file is specific for Phoenix with 3 DOF
//  
//    This version of the Configuration file is set up to run on the
//    Lynxmotion BotboardDuino board, which is similar to the Arduino Duemilanove
//
//    This version of configuration file assumes that the servos will be controlled
//    by a Lynxmotion Servo controller SSC-32 and the user is using a Lynxmotion 
//    PS3 to control the robot.
//
//Date: March 18, 2012
//Programmer: Kurt (aka KurtE)
//
//
//NEW IN V1.0
//   - First Release
//
//====================================================================
#ifndef HEX_CFG_PHOENIX3_H
#define HEX_CFG_PHOENIX3_H
//[CONDITIONAL COMPILING] - COMMENT IF NOT WANTED
// Define other optional compnents to be included or not...

#define OPT_TERMINAL_MONITOR

#ifdef OPT_TERMINAL_MONITOR       // turning off terminal monitor will turn these off as well...
#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
#define DBGSerial         Serial
//#define DEBUG_IOPINS
#endif

//#define OPT_GPPLAYER
#define OPT_BACKGROUND_PROCESS
#define OPT_SINGLELEG

//#define DEFAULT_GAIT_SPEED 100
//#define DEFAULT_SLOW_GAIT 120

//#define cEyesPin 13

// As given by sixaxis pair tool - matches PlayStation master MAC address
char ps3ControllerMacAddr[] = "2c:81:58:a9:8d:76";

// Define other optional compnents to be included or not...
#define cRRCoxaInv 0 
#define cRMCoxaInv 0 
#define cRFCoxaInv 0 
#define cLRCoxaInv 0
#define cLMCoxaInv 0
#define cLFCoxaInv 0

#define cRRFemurInv 0 
#define cRMFemurInv 0 
#define cRFFemurInv 0
#define cLRFemurInv 0
#define cLMFemurInv 0
#define cLFFemurInv 0

#define cRRTibiaInv 1
#define cRMTibiaInv 1
#define cRFTibiaInv 1
#define cLRTibiaInv 1
#define cLMTibiaInv 1
#define cLFTibiaInv 1

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
// AliPhoenix
//==================================================================================================================================
#define USE_MAESTRO

// Warning I will undefine some components as the non-megas don't have enough memory...
//#undef OPT_FIND_SERVO_OFFSETS 
#ifdef USE_MAESTRO
#define cMAESTRO_BAUD    115200   //MAESTRO BAUD rate

//--------------------------------------------------------------------
// Required if using sofware serial connection for Maestro

//#define SOUND_PIN       5       // Botboarduino JR pin number
#define cMAESTRO_OUT     12       //Output pin for (MAESTRO RX)
#define cMAESTRO_IN      13       //Input pin for (MAESTRO TX)

// Serial connection settings for debugging and connection to Maestro servo controller
#ifdef UBRR1H
#define MAESTROSerial         Serial1
#elif defined(ESP32)
#define min _min
#define max _max
#define MAESTROSerial         Serial2
#else
SoftwareSerial MAESTROSerial(cMAESTRO_IN, cMAESTRO_OUT);
#endif

//--------------------------------------------------------------------
//[SERVO PIN NUMBERS - MAESTRO]
// Need to have consectutive pin numbers per leg starting with coxa
#define cRFCoxaPin      15   //Front Right leg Hip Horizontal
#define cRFFemurPin     16   //Front Right leg Hip Vertical
#define cRFTibiaPin     17   //Front Right leg Knee
//#define cRFTarsPin      2   // Tar

#define cRMCoxaPin      9    //Middle Right leg Hip Horizontal
#define cRMFemurPin     10   //Middle Right leg Hip Vertical
#define cRMTibiaPin     11   //Middle Right leg Knee
//#define cRMTarsPin      8  // Tar

#define cRRCoxaPin      3    //Rear Right leg Hip Horizontal
#define cRRFemurPin     4    //Rear Right leg Hip Vertical
#define cRRTibiaPin     5    //Rear Right leg Knee
//#define cRRTarsPin      14  // Tar

#define cLFCoxaPin      12   //Front Left leg Hip Horizontal
#define cLFFemurPin     13   //Front Left leg Hip Vertical
#define cLFTibiaPin     14   //Front Left leg Knee
//#define cLFTarsPin      5   // Tar

#define cLMCoxaPin      6    //Middle Left leg Hip Horizontal
#define cLMFemurPin     7    //Middle Left leg Hip Vertical
#define cLMTibiaPin     8    //Middle Left leg Knee
//#define cLMTarsPin      11   // Tar

#define cLRCoxaPin      0    //Rear Left leg Hip Horizontal
#define cLRFemurPin     1    //Rear Left leg Hip Vertical
#define cLRTibiaPin     2    //Rear Left leg Knee
//#define cLMTarsPin      17   // Tar
#endif

//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1	-260	//Mechanical limits of the Right Rear Leg, decimals = 1
#define cRRCoxaMax1	740
#define cRRFemurMin1	-1010
#define cRRFemurMax1	950
#define cRRTibiaMin1	-1060
#define cRRTibiaMax1	770

#define cRMCoxaMin1	-530	//Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1	530
#define cRMFemurMin1	-1010
#define cRMFemurMax1	950
#define cRMTibiaMin1	-1060
#define cRMTibiaMax1	770

#define cRFCoxaMin1	-580	//Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1	740
#define cRFFemurMin1	-1010
#define cRFFemurMax1	950
#define cRFTibiaMin1	-1060
#define cRFTibiaMax1	770

#define cLRCoxaMin1	-740	//Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1	260
#define cLRFemurMin1	-950
#define cLRFemurMax1	1010
#define cLRTibiaMin1	-770
#define cLRTibiaMax1	1060

#define cLMCoxaMin1	-530	//Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1	530
#define cLMFemurMin1	-950
#define cLMFemurMax1	1010
#define cLMTibiaMin1	-770
#define cLMTibiaMax1	1060

#define cLFCoxaMin1	-740	//Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1	580
#define cLFFemurMin1	-950
#define cLFFemurMax1	1010
#define cLFTibiaMin1	-770
#define cLFTibiaMax1	1060


//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     29    // This is for AliPhoenix legs
#define cXXFemurLength    84
#define cXXTibiaLength    124
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
//[GAIT PARAMETERS]
#define legLiftHeight 30
#define doubleLegLiftHeight 80

#define singleTravelLength 20 //1 decimal place
#define doubleTravelLength 13 //1 decimal place

//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1   -600       //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0          //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    600        //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -600       //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0          //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    600        //Default Coxa setup angle, decimals = 1

// ALIPHOENIX BODY
#define cRROffsetX   -43    //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ  74      //Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX  -65     //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ  0       //Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX  -43     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ  -74     //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX  43      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ  74      //Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX  65      //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ  0       //Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX  43      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ  -74     //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
//AliPhoenix
//#define cHexInitXZ	 105 
//#define CHexInitXZCos60  53        // COS(60) = .5
//#define CHexInitXZSin60  91        // sin(60) = .866
//#define CHexInitY	 15

#define cHexInitXZ   90 
#define CHexInitXZCos60  45        // COS(60) = .5
#define CHexInitXZSin60  78        // sin(60) = .866
#define CHexInitY  30

#define MAX_BODY_Y  170
// Lets try some multi leg positions depending on height settings.
#if 0 // Start first without...
#define CNT_HEX_INITS 3
#define MAX_BODY_Y  170
#ifdef DEFINE_HEX_GLOBALS
const byte g_abHexIntXZ[] PROGMEM = {cHexInitXZ, 99, 86};
const byte g_abHexMaxBodyY[] PROGMEM = {20, 50, MAX_BODY_Y};
#else
extern const byte g_abHexIntXZ[] PROGMEM;
extern const byte g_abHexMaxBodyY[] PROGMEM;
#endif
#endif  // if 0

#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ      //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60
//--------------------------------------------------------------------
//[Tars factors used in formula to calc Tarsus angle relative to the ground]
#define cTarsConst	720	//4DOF ONLY
#define cTarsMulti	2	//4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

#endif CFG_HEX_H
