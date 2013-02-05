//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration for Kurt's Lynxmotion Quad
//    using CH3-R types of legs.
//  
//    This version of the Configuration file is set up to run on the
//    Lynxmotion BotboardDuino board, which is similar  to the Arduino Duemilanove
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
#ifndef HEX_CFG_CHR3_H
#define HEX_CFG_CHR3_H


// Which type of control(s) do you want to compile in

//#define DEBUG

#ifdef __AVR__
#if defined(UBRR1H)
#define SSCSerial         Serial1
#define XBeeSerial        Serial3 //Serial2
#else
// On Botboarduino either define XBee to be on USART or define cXBEE_IN/cXBee_OUT
//#define XBeeSerial        Serial
#define cXBEE_IN           6
#define cXBEE_OUT          7
//#define DontAllowDebug
#endif
#else  // For My Pic32 Mega shield...
#define SSCSerial         Serial1
#define XBeeSerial        Serial3
#endif

//==================================================================================================================================
// Define which input classes we will use. If we wish to use more than one we need to define USEMULTI - This will define a forwarder
//    type implementation, that the Inputcontroller will need to call.  There will be some negotion for which one is in contol.
//
//  If this is not defined, The included Controller should simply implement the InputController Class...
//==================================================================================================================================
#define QUADMODE            // We are building for quad support...

//#define USEMULTI
#define USEXBEE            // only allow to be defined on Megas...
//#define USEPS2
//#define USECOMMANDER
//#define USESERIAL

// Do we want Debug Serial Output?
#define DBGSerial Serial

// Some configurations will not allow this so if one of them undefine it
#if (defined USEXBEE) || (defined USECOMMANDER)
#ifdef DontAllowDebug
#undef DBGSerial
#endif
#endif

#ifdef USESERIAL
#undef DBGSerial
#endif

#ifdef DBGSerial
#define OPT_TERMINAL_MONITOR  // Only allow this to be defined if we have a debug serial port
#endif

#ifdef OPT_TERMINAL_MONITOR
//#define OPT_SSC_FORWARDER  // only useful if terminal monitor is enabled
#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
#endif

//#define OPT_GPPLAYER

#define USE_SSC32
#define	cSSC_BINARYMODE	1			// Define if your SSC-32 card supports binary mode.
#define cSSC_BAUD        38400   //SSC32 BAUD rate

// Debug options
//#define DEBUG_IOPINS    // used to control if we are going to use IO pins for debug support

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
// CHR-3
//==================================================================================================================================

//[Assuming Botboarduino for this one
#define SOUND_PIN    5        // Botboarduino JR pin number

// PS2 definitions
#define PS2_DAT      6        
#define PS2_CMD      7
#define PS2_SEL      8
#define PS2_CLK      9

#define cSSC_OUT     10      	//Output pin for (SSC32 RX) on BotBoard (Yellow)
#define cSSC_IN      11      	//Input pin for (SSC32 TX) on BotBoard (Blue)
// XBee was defined to use a hardware Serial port
#define XBEE_BAUD        38400
#define SERIAL_BAUD    38400

// Define Analog pin and minimum voltage that we will allow the servos to run
#define cVoltagePin  0      // Use our Analog pin jumper here...
#define cTurnOffVol  470     // 4.7v
#define cTurnOnVol   550     // 5.5V - optional part to say if voltage goes back up, turn it back on...

//====================================================================
// Warning I reversed my legs as I put left legs where right should be...
//[SSC32 Pin Numbers]
#define cRRCoxaPin      0   //Rear Right leg Hip Horizontal
#define cRRFemurPin     1   //Rear Right leg Hip Vertical
#define cRRTibiaPin     2   //Rear Right leg Knee
#define cRRTarsPin      3   // Tar

#define cRFCoxaPin      8    //Front Right leg Hip Horizontal
#define cRFFemurPin     9    //Front Right leg Hip Vertical
#define cRFTibiaPin     10   //Front Right leg Knee
#define cRFTarsPin      11   // Tar

#define cLRCoxaPin       16   //Rear Left leg Hip Horizontal
#define cLRFemurPin      17   //Rear Left leg Hip Vertical
#define cLRTibiaPin      18   //Rear Left leg Knee
#define cLRTarsPin       19   // Tar

#define cLFCoxaPin      24   //Front Left leg Hip Horizontal
#define cLFFemurPin     25   //Front Left leg Hip Vertical
#define cLFTibiaPin     26   //Front Left leg Knee
#define cLFTarsPin      27   // Tar

//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1     -650      //Mechanical limits of the Right Rear Leg
#define cRRCoxaMax1     650
#define cRRFemurMin1    -1050
#define cRRFemurMax1    750
#define cRRTibiaMin1    -530
#define cRRTibiaMax1    900
#define cRRTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRRTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cRFCoxaMin1     -650      //Mechanical limits of the Right Front Leg
#define cRFCoxaMax1     650
#define cRFFemurMin1    -1050
#define cRFFemurMax1    750
#define cRFTibiaMin1    -530
#define cRFTibiaMax1    900
#define cRFTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRFTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLRCoxaMin1     -650      //Mechanical limits of the Left Rear Leg
#define cLRCoxaMax1     650
#define cLRFemurMin1    -1050
#define cLRFemurMax1    750
#define cLRTibiaMin1    -530
#define cLRTibiaMax1    900
#define cLRTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLRTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLFCoxaMin1     -650      //Mechanical limits of the Left Front Leg
#define cLFCoxaMax1     650
#define cLFFemurMin1    -1050
#define cLFFemurMax1    750
#define cLFTibiaMin1    -530
#define cLFTibiaMax1    900
#define cLFTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLFTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     29    // This is for CH3-R with Type 3 legs
#define cXXFemurLength    57
#define cXXTibiaLength    141
#define cXXTarsLength     85    // 4DOF only...

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength
#define cRRTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength
#define cRFTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength
#define cLRTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength
#define cLFTarsLength	  cXXTarsLength	    //4DOF ONLY


//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1   -0     //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    0    //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -0   //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    0    //Default Coxa setup angle, decimals = 1

#define cRROffsetX      -43     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      82     //Distance Z from center of the body to the Right Rear coxa
#define cRFOffsetX      -43     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -82    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      43      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      82      //Distance Z from center of the body to the Left Rear coxa
#define cLFOffsetX      43      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -82    //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ	 110 
#define CHexInitXZCos60  55        // COS(60) = .5
#define CHexInitXZSin60  95    // sin(60) = .866
#define CHexInitXZ45    78        // Sin and cos(45) .7071
#define CHexInitY	60 //30

#if 1
#define cRRInitPosX     CHexInitXZ45      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZ45

#define cRFInitPosX     CHexInitXZ45      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZ45

#define cLRInitPosX     CHexInitXZ45      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZ45

#define cLFInitPosX     CHexInitXZ45      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZ45


#else
#define cRRInitPosX     cHexInitXZ      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     0

#define cRFInitPosX     cHexInitXZ      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     0

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     0

#define cLFInitPosX     cHexInitXZ      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     0
#endif
//--------------------------------------------------------------------
//[Tars factors used in formula to calc Tarsus angle relative to the ground]
#define cTarsConst	720	//4DOF ONLY
#define cTarsMulti	2	//4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

#endif CFG_HEX_H

