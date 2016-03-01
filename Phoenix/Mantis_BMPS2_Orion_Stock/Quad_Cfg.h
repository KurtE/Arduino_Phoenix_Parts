//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the 
//    Orion Robotics Mantis Quad using their Serial PS2
//
//====================================================================
#ifndef QUAD_CFG_H
#define QUAD_CFG_H

#define QUADMODE            // We are building for quad support...
#define DBGSerial Serial
//==================================================================================================================================
// Define which input classes we will use. If we wish to use more than one we need to define USEMULTI - This will define a forwarder
//    type implementation, that the Inputcontroller will need to call.  There will be some negotion for which one is in contol.
//
//  If this is not defined, The included Controller should simply implement the InputController Class...
//==================================================================================================================================

// Global defines to control which configuration we are using.  Note: Only define one of these...
// 
// Which type of control(s) do you want to compile in
#ifdef USEXBEE    // some options only valid if running with XBEE stuff
#define XBEE_DEBUG_OUTPUT    // use our xbee serial class to do debug stuff
#define DBGSerial XBDSerial
#endif 
//#define DBGSerial         Serial

// Define other optional compnents to be included or not...
#define BALANCE_DELAY 25    // don't add as much as the default here.

//===================================================================
// Debug Options
#ifdef DBGSerial
#define OPT_TERMINAL_MONITOR  
#endif

//#define DEBUG_IOPINS
#ifdef DEBUG_IOPINS
#define DebugToggle(pin)  {digitalWrite(pin, !digitalRead(pin));}
#define DebugWrite(pin, state) {digitalWrite(pin, state);}
#else
#define DebugToggle(pin)  {;}
#define DebugWrite(pin, state) {;}
#endif


// Also define that we are using the AX12 driver

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
//  PhantomX
//==================================================================================================================================
//[SERIAL CONNECTIONS]

//====================================================================
// XBEE on non mega???
#define XBeeSerial Serial
#define XBEE_BAUD        38400
#define DISP_VOLTAGE    // User wants the Battery voltage to be displayed...
#define DISP_VOLTAGE_TIME  1000  // how often to check and report in millis
//--------------------------------------------------------------------
//[Arbotix Pin Numbers]
#define SOUND_PIN    3         // Sound is on The orion shield pin 3
#define PS2_PIN      6         // Their serial PS2 receiver is plugged into pin 6

// Define minimum voltage that we will allow the servos to run
#define cTurnOffVol  650     // 6.5
#define cTurnOnVol   700     // 7v - optional part to say if voltage goes back up, turn it back on...

//====================================================================
#define  DEFAULT_GAIT_SPEED 50  // Default gait speed  - Will depend on what Servos you are using...
#define  DEFAULT_SLOW_GAIT  50  // Had a couple different speeds...

//====================================================================


//--------------------------------------------------------------------
// Define which pins(sevo IDS go with which joint

#define cRRCoxaPin      4   //Rear Right leg Hip Horizontal
#define cRRFemurPin     5   //Rear Right leg Hip Vertical
#define cRRTibiaPin     6   //Rear Right leg Knee

#define cRFCoxaPin      8  //Front Right leg Hip Horizontal
#define cRFFemurPin     9  //Front Right leg Hip Vertical
#define cRFTibiaPin     10   //Front Right leg Knee

#define cLRCoxaPin      16   //Rear Left leg Hip Horizontal
#define cLRFemurPin     17   //Rear Left leg Hip Vertical
#define cLRTibiaPin     18   //Rear Left leg Knee

#define cLFCoxaPin      20   //Front Left leg Hip Horizontal
#define cLFFemurPin     21  //Front Left leg Hip Vertical
#define cLFTibiaPin     22  //Front Left leg Knee


//--------------------------------------------------------------------
//[MIN/MAX ANGLES] - We will let the Orion controller handle Min Max and servo inversions.
#define SERVOS_DO_MINMAX    // the servo controller will take care of this

#define cRRCoxaInv 0
#define cRFCoxaInv 0
#define cLRCoxaInv 0 
#define cLFCoxaInv 0
#define cRRFemurInv 0 
#define cRFFemurInv 0 
#define cLRFemurInv 0 
#define cLFFemurInv 0
#define cRRTibiaInv 1
#define cRFTibiaInv 1
#define cLRTibiaInv 1 
#define cLFTibiaInv 1


//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm

#define cXXCoxaLength     60    // Mantis leg dimensions.
#define cXXFemurLength    113
#define cXXTibiaLength    173

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength

//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1   -450   //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    450    //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -450   //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    450    //Default Coxa setup angle, decimals = 1

#define X_COXA          60      // MM between front and back legs /2
#define Y_COXA          60      // MM between front/back legs /2

#define cRROffsetX      -X_COXA     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      Y_COXA     //Distance Z from center of the body to the Right Rear coxa

#define cRFOffsetX      -X_COXA     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -Y_COXA    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      X_COXA      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      Y_COXA     //Distance Z from center of the body to the Left Rear coxa

#define cLFOffsetX      X_COXA      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -Y_COXA    //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ	 170
#define CHexInitXZCos60  147 //92      // COS(60) = .866
#define CHexInitXZSin60  85 // 92     // sin(60) = .5g
#define CHexInitY	 45       //30

// Lets try some multi leg positions depending on height settings.
#define CNT_HEX_INITS   3
#define MAX_BODY_Y    250
// For Inits we may want to tell system actual angles we are initiing servos to...
// In some cases like some quads may not want legs straight out...
#define cRRInitCoxaAngle1   -300   //Default Coxa setup angle, decimals = 1
#define cRFInitCoxaAngle1    300    //Default Coxa setup angle, decimals = 1
#define cLRInitCoxaAngle1    -300   //Default Coxa setup angle, decimals = 1
#define cLFInitCoxaAngle1    300    //Default Coxa setup angle, decimals = 1


#ifdef DEFINE_HEX_GLOBALS
const byte g_abHexIntXZ[] PROGMEM = {cHexInitXZ, 150, 120};
const byte g_abHexMaxBodyY[] PROGMEM = { 80, 120, MAX_BODY_Y};
#else
extern const byte g_abHexIntXZ[] PROGMEM;
extern const byte g_abHexMaxBodyY[] PROGMEM;
#endif

#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60
//--------------------------------------------------------------------
#endif // HEX_CFG_H
