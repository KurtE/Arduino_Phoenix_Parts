//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver - This version is setup to use AX-12 type servos using the
// Arbotix AX12 and bioloid libraries (which may have been updated)
//====================================================================
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#include <avr\pgmspace.h>
#endif


#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif

#define NUMSERVOS (NUMSERVOSPERLEG*6)
short g_asLegOffsets[NUMSERVOS];       // Offsets per leg
boolean g_fAXSpeedControl;      // flag to know which way we are doing output...
#include "MaestroEx.h"

#ifdef DBGSerial
//#define DEBUG
// Only allow debug stuff to be turned on if we have a debug serial port to output to...
#define DEBUG_SERVOS
#endif

#ifdef DEBUG_SERVOS
#define ServosEnabled   (g_fEnableServos)
#else
#define ServosEnabled  (true)      // always true compiler should remove if...
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
const byte cCoxaPin[] PROGMEM = {
  cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin};
const byte cFemurPin[] PROGMEM = {
  cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin};
const byte cTibiaPin[] PROGMEM = {
  cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin};
#ifdef c4DOF
const byte cTarsPin[] PROGMEM = {
  cRRTarsPin, cRMTarsPin, cRFTarsPin, cLRTarsPin, cLMTarsPin, cLFTarsPin};
#endif

// Not sure yet if I will use the controller class or not, but...
MaestroControllerEx maestro = MaestroControllerEx(cMAESTRO_BAUD);
boolean g_fServosFree;    // Are the servos in a free state?



//--------------------------------------------------------------------
// Helper function to load the servo offsets from the EEPROM
//--------------------------------------------------------------------
void LoadServosConfig(bool reset = false) {

  byte *pb = (byte*)&g_asLegOffsets;
  byte bChkSum = 0;      //
  int i;

  if (EEPROM.read(0) == 6*NUMSERVOSPERLEG) {
    for (i=0; i < sizeof(g_asLegOffsets); i++) {
          *pb = EEPROM.read(i+2);
          bChkSum += *pb++;
    }
    
    // now see if the checksum matches.
    if (bChkSum == EEPROM.read(1) && (reset == false))
      return;    // we have valid data
  }
  
  // got to here, something not right, set up 0's for servo offsets.
  for (i = 0; i < 6*NUMSERVOSPERLEG; i++) {
    g_asLegOffsets[i] = 0;
  }
  
#ifdef DEBUG
  DBGSerial.println("##ERROR: all servo offsets have been set to zero");
#endif
}

void SetServoIds(void) {
#ifdef c3DOF
	maestro.setId(0, cRRCoxaPin);
	maestro.setId(1, cRRFemurPin);
	maestro.setId(2, cRRTibiaPin);
	
	maestro.setId(3, cRMCoxaPin);
	maestro.setId(4, cRMFemurPin);
	maestro.setId(5, cRMTibiaPin);
	
	maestro.setId(6, cRFCoxaPin);
	maestro.setId(7, cRFFemurPin);
	maestro.setId(8, cRFTibiaPin);
	
	maestro.setId(9, cLRCoxaPin);
	maestro.setId(10, cLRFemurPin);
	maestro.setId(11, cLRTibiaPin);
	
	maestro.setId(12, cLMCoxaPin);
	maestro.setId(13, cLMFemurPin);
	maestro.setId(14, cLMTibiaPin);
	
	maestro.setId(15, cLFCoxaPin);
	maestro.setId(16, cLFFemurPin);
	maestro.setId(17, cLFTibiaPin);
#elseif c4DOF
	maestro.setId(0, cRRCoxaPin);
	maestro.setId(1, cRRFemurPin);
	maestro.setId(2, cRRTibiaPin);
	maestro.setId(3, cRRTarsPin);
	
	maestro.setId(4, cRMCoxaPin);
	maestro.setId(5, cRMFemurPin);
	maestro.setId(6, cRMTibiaPin);
	maestro.setId(7, cRMTarsPin);
	
	maestro.setId(8, cRFCoxaPin);
	maestro.setId(9, cRFFemurPin);
	maestro.setId(10, cRFTibiaPin);
	maestro.setId(11, cRFTarsPin);
	
	maestro.setId(12, cLRCoxaPin);
	maestro.setId(13, cLRFemurPin);
	maestro.setId(14, cLRTibiaPin);
	maestro.setId(15, cLRTarsPin);
	
	maestro.setId(16, cLMCoxaPin);
	maestro.setId(17, cLMFemurPin);
	maestro.setId(18, cLMTibiaPin);
	maestro.setId(19, cLMTarsPin);
	
	maestro.setId(20, cLFCoxaPin);
	maestro.setId(21, cLFFemurPin);
	maestro.setId(22, cLFTibiaPin);
	maestro.setId(23, cLFTarsPin);
#endif
	
}

// Some forward references
extern void MakeSureServosAreOn(void);
extern void DoPyPose(byte *psz);
extern void EEPROMReadData(word wStart, uint8_t *pv, byte cnt);
extern void EEPROMWriteData(word wStart, uint8_t *pv, byte cnt);

//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
  SetServoIds();
  LoadServosConfig();
  
  // First lets get the actual servo positions for all of our servos...
  //pinMode(0, OUTPUT);
  g_fServosFree = true;
  
  maestro.poseSize = NUMSERVOS;
  maestro.readPose();
#ifdef cVoltagePin  
  for (byte i=0; i < 8; i++)
    GetBatteryVoltage();  // init the voltage pin
#endif

  g_fAXSpeedControl = false;

#ifdef OPT_GPPLAYER
  _fGPEnabled = true;    // assume we support it.
#endif
}


//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating 
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef cVoltagePin  
word  g_awVoltages[8]={
  0,0,0,0,0,0,0,0};
word  g_wVoltageSum = 0;
byte  g_iVoltages = 0;

word ServoDriver::GetBatteryVoltage(void) {
  g_iVoltages = (++g_iVoltages)&0x7;  // setup index to our array...
  g_wVoltageSum -= g_awVoltages[g_iVoltages];
  g_awVoltages[g_iVoltages] = analogRead(cVoltagePin);
  g_wVoltageSum += g_awVoltages[g_iVoltages];

  return ((long)((long)g_wVoltageSum*125*(CVADR1+CVADR2))/(long)(2048*(long)CVADR2));  

}

#else
#define VOLTAGE_REPEAT_MAX  3
#define VOLTAGE_MAX_TIME_BETWEEN_CALLS 500    // call at least twice a second...

word g_wLastVoltage = 0xffff;    // save the last voltage we retrieved...

unsigned long g_ulTimeLastBatteryVoltage;

word ServoDriver::GetBatteryVoltage(void) {
  return 0;
}
#endif

//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void ServoDriver::BeginServoUpdate(void)    // Start the update 
{
  MakeSureServosAreOn();
  if (ServosEnabled) {
    DebugToggle(A4);
    if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // If we are trying our own Servo control need to save away the new positions...
      for (byte i=0; i < NUMSERVOS; i++) {
        g_awCurAXPos[i] = g_awGoalAXPos[i];
      }
#endif
    }
    else       
      maestro.interpolateStep(true);    // Make sure we call at least once
  }
}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#define cPwmDiv       991  //old 1059;
#define cPFConst      592  //old 650 ; 900*(1000/cPwmDiv)+cPFConst must always be 1500
// A PWM/deg factor of 10,09 give cPwmDiv = 991 and cPFConst = 592
// For a modified 5645 (to 180 deg travel): cPwmDiv = 1500 and cPFConst = 900.
#ifdef c4DOF
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1)
#else
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1)
#endif    
{        
  word    wCoxaSSCV;        // Coxa value in SSC units
  word    wFemurSSCV;       //
  word    wTibiaSSCV;       //
#ifdef c4DOF
  word    wTarsSSCV;        //
#endif

  //Update Right Legs
  g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
  if (LegIndex < 3) {
    wCoxaSSCV = ((long)(-sCoxaAngle1 +900))*1000/cPwmDiv+cPFConst;
    wFemurSSCV = ((long)(-sFemurAngle1+900))*1000/cPwmDiv+cPFConst;
    wTibiaSSCV = ((long)(-sTibiaAngle1+900))*1000/cPwmDiv+cPFConst;
#ifdef c4DOF
    wTarsSSCV = ((long)(-sTarsAngle1+900))*1000/cPwmDiv+cPFConst;
#endif
  } 
  else {
    wCoxaSSCV = ((long)(sCoxaAngle1 +900))*1000/cPwmDiv+cPFConst;
    wFemurSSCV = ((long)((long)(sFemurAngle1+900))*1000/cPwmDiv+cPFConst);
    wTibiaSSCV = ((long)(sTibiaAngle1+900))*1000/cPwmDiv+cPFConst;
#ifdef c4DOF
    wTarsSSCV = ((long)(sTarsAngle1+900))*1000/cPwmDiv+cPFConst;
#endif
  }

  maestro.setNextPose(pgm_read_byte(&cCoxaPin[LegIndex]), wCoxaSSCV + g_asLegOffsets[LegIndex*NUMSERVOSPERLEG + 0]);
  maestro.setNextPose(pgm_read_byte(&cFemurPin[LegIndex]), wFemurSSCV + g_asLegOffsets[LegIndex*NUMSERVOSPERLEG + 1]);
  maestro.setNextPose(pgm_read_byte(&cTibiaPin[LegIndex]), wTibiaSSCV + g_asLegOffsets[LegIndex*NUMSERVOSPERLEG + 2]);
  
//#ifdef DEBUG
//  DBGSerial.print("Leg Index: ");
//  DBGSerial.print(LegIndex);
//  DBGSerial.print(", Pose: ");
//  DBGSerial.print(wCoxaSSCV);
//  DBGSerial.print(", ");
//  DBGSerial.print(wFemurSSCV);
//  DBGSerial.print(", ");
//  DBGSerial.print(wTibiaSSCV);
//#endif

#ifdef c4DOF
  if ((byte)pgm_read_byte(&cTarsLength[LegIndex])) {
    maestro.setNextPose(pgm_read_byte(&cTarsPin[LegIndex]), wTarsSSCV + g_asLegOffsets[LegIndex*NUMSERVOSPERLEG + 3]);
    
//#ifdef DEBUG
//    DBGSerial.print(", ");
//    DBGSerial.print(wTarsSSCV);
//#endif
  }
#endif

//#ifdef DEBUG
//  DBGSerial.println();
//#endif  

  g_InputController.AllowControllerInterrupts(true);    // Ok for hserial again...
}


//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly 
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriver::CommitServoDriver(word wMoveTime)
{
#ifdef cSSC_BINARYMODE
  byte    abOut[3];
#endif

  g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
  if (ServosEnabled) {
    if (g_fAXSpeedControl) {
		// No speed control option
    }
    else {
      maestro.interpolateSetup(wMoveTime);
    }
  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput)
    DBGSerial.println(wMoveTime, DEC);
#endif
  g_InputController.AllowControllerInterrupts(true);    

}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void)
{
  if (ServosEnabled) {
    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    for (byte i = 0; i < NUMSERVOS; i++) {
      //Relax(pgm_read_byte(&cPinTable[i]));
    }
    g_InputController.AllowControllerInterrupts(true);    
    g_fServosFree = true;
  }
}

//--------------------------------------------------------------------
//[MakeSureServosAreOn] Function that is called to handle when you are
//  transistioning from servos all off to being on.  May need to read
//  in the current pose...
//--------------------------------------------------------------------
void MakeSureServosAreOn(void)
{
	// Not sure how to do this...
}

//==============================================================================
// BackgroundProcess - Allows us to have some background processing for those
//    servo drivers that need us to do things like polling...
//==============================================================================
void  ServoDriver::BackgroundProcess(void) 
{
  if (g_fAXSpeedControl) 
    return;  // nothing to do in this mode...

  if (ServosEnabled) {
    DebugToggle(A3);

    maestro.interpolateStep(false);    // Do our background stuff...
  }
}

#ifdef OPT_TERMINAL_MONITOR  
extern void FindServoOffsets(void);
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void ServoDriver::ShowTerminalCommandList(void) 
{
  DBGSerial.println(F("F<frame length> - FL in ms"));    // BUGBUG:: 
  DBGSerial.println(F("W - Wipe EEPROM servo offsets"));    // BUGBUG:: 
#ifdef OPT_FIND_SERVO_OFFSETS
  DBGSerial.println(F("O - Enter Servo offset mode"));
#endif        
}

//==============================================================================
// ProcessTerminalCommand: The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
boolean ServoDriver::ProcessTerminalCommand(byte *psz, byte bLen)
{
#ifdef OPT_FIND_SERVO_OFFSETS
  if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
    FindServoOffsets();
  }
  if ((bLen == 1) && ((*psz == 'w') || (*psz == 'W'))) {
    int data;
	Serial.println("Are you sure you want to set offsets to zero? Y/N: ");
	
	//get user entered data
	while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15)))
	;
	
	if ((data == 'Y') || (data == 'y')) {
		LoadServosConfig(true);
		Serial.println("\nAll offsets set to zero");
	}
	else {
		Serial.println("Loading old configuration");
		void LoadServosConfig();
	}
  }
  if ((bLen >= 1) && ((*psz == 'f') || (*psz == 'F'))) {
    psz++;  // need to get beyond the first character
    while (*psz == ' ') 
      psz++;  // ignore leading blanks...
    byte bFrame = 0;
    while ((*psz >= '0') && (*psz <= '9')) {  // Get the frame count...
      bFrame = bFrame*10 + *psz++ - '0';
    }
    if (bFrame != 0) {
      DBGSerial.print(F("New Servo Cycles per second: "));
      DBGSerial.println(1000/bFrame, DEC);
      extern MaestroControllerEx maestro;
      maestro.frameLength = bFrame;
    }
  } 
#endif

   return false;
}
#endif

//==============================================================================
//	FindServoOffsets - Find the zero points for each of our servos... 
// 		Will use the new servo function to set the actual pwm rate and see
//		how well that works...
//==============================================================================
#ifdef OPT_FIND_SERVO_OFFSETS
void FindServoOffsets()
{
	// not clean but...
	byte abMAESTROServoNum[NUMSERVOSPERLEG*6];           // array of servos...
	
	static char *apszLegs[] = {"RR","RM","RF", "LR", "LM", "LF"};  // Leg Order
	static char *apszLJoints[] = {" Coxa", " Femur", " Tibia", " tArs"}; // which joint on the leg...
	
	int data;
	short sSN = 0; 			// which servo number
	boolean fNew = true;	// is this a new servo to work with?
	boolean fExit = false;	// when to exit
	short sOffset;
	
	if (CheckVoltage()) {
		// Voltage is low... 
		Serial.println("Low Voltage: fix or hit $ to abort");
		while (CheckVoltage()) {
			if (Serial.read() == '$')  return;
		}
	}

	// Fill in array of MAESTRO servo numbers    
	for (sSN=0; sSN < 6; sSN++) {   // Make sure all of our servos initialize to 0 offset from saved.
		abMAESTROServoNum[sSN*NUMSERVOSPERLEG + 0] = pgm_read_byte(&cCoxaPin[sSN]);
		abMAESTROServoNum[sSN*NUMSERVOSPERLEG + 1] = pgm_read_byte(&cFemurPin[sSN]);
		abMAESTROServoNum[sSN*NUMSERVOSPERLEG + 2] = pgm_read_byte(&cTibiaPin[sSN]);
#ifdef c4DOF
		abMAESTROServoNum[sSN*NUMSERVOSPERLEG + 3] = pgm_read_byte(&cTarsPin[sSN]);
#endif
	}
	
	// now lets loop through and get information and set servos to 1500
	for (sSN=0; sSN < 6*NUMSERVOSPERLEG; sSN++ ) {
		maestro.writePos(abMAESTROServoNum[sSN], 1500 + g_asLegOffsets[sSN]);
	}
	
	// OK lets move all of the servos to their zero point.
	Serial.println("Find Servo Zeros.\n$-Exit, +- changes, *-change servo");
	Serial.println("    0-5 Chooses a leg, C-Coxa, F-Femur, T-Tibia");

	//sSN = true;
	sSN = 0;
	while(!fExit) {
		if (fNew) {
			sOffset = g_asLegOffsets[sSN];
			Serial.print("Servo: ");
			Serial.print(apszLegs[sSN/NUMSERVOSPERLEG]);
			Serial.print(apszLJoints[sSN%NUMSERVOSPERLEG]);
			Serial.print("(");
			Serial.print(sOffset, DEC);
			Serial.println(")");
			
			// Now lets wiggle the servo
			//Serial.print("\nWiggling servo number: ");
			//Serial.println(abMAESTROServoNum[sSN]);
			maestro.writePos(abMAESTROServoNum[sSN], (1500 + sOffset + 100));
			delay(250);
			maestro.writePos(abMAESTROServoNum[sSN], (1500 + sOffset - 100));
			delay(500);
			maestro.writePos(abMAESTROServoNum[sSN], (1500 + sOffset));
			delay(250);
			
			fNew = false;
		}

		//get user entered data
		data = Serial.read();
		
		//if data received
		if (data !=-1) 	{
			if (data == '$')
				fExit = true;	// not sure how the keypad will map so give NL, CR, LF... all implies exit
			
			else if ((data == '+') || (data == '-')) {
				if (data == '+')
					sOffset += 5;		// increment by 5us
				else
					sOffset -= 5;		// increment by 5us
				
				Serial.print("    ");
				Serial.println(sOffset, DEC);
				
				g_asLegOffsets[sSN] = sOffset;
				maestro.writePos(abMAESTROServoNum[sSN], (1500 + sOffset));
			}
			else if ((data >= '0') && (data <= '5')) {
				// direct enter of which servo to change
				fNew = true;
				sSN = (sSN % NUMSERVOSPERLEG) + (data - '0')*NUMSERVOSPERLEG;
			}
			else if ((data == 'c') || (data == 'C')) {
				fNew = true;
				sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 0;
			} 
			else if ((data == 'f') || (data == 'F')) {
				fNew = true;
				sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 1;
			}
			else if ((data == 't') || (data == 'T')) {
				fNew = true;
				sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 2;
			}
			else if (data == '*') {
				// direct enter of which servo to change
				fNew = true;
				sSN++;
				if (sSN == 6*NUMSERVOSPERLEG) 
					sSN = 0;	
			}
		}
	}
	
	Serial.print("Find Servo exit ");
	for (sSN=0; sSN < 6*NUMSERVOSPERLEG; sSN++) {
		Serial.print(" ");
		Serial.print(g_asLegOffsets[sSN], DEC);
	}

	Serial.print("\nSave Changes? Y/N: ");

	//get user entered data
	while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15)))
	;
	
	if ((data == 'Y') || (data == 'y')) {
		// Ok they asked for the data to be saved.  We will store the data with a 
		// number of servos (byte)at the start, followed by a byte for a checksum...followed by our offsets array...
		// Currently we store these values starting at EEPROM address 0. May later change...
		
		byte *pb = (byte*)&g_asLegOffsets;
		byte bChkSum = 0;  //
		EEPROM.write(0, 6*NUMSERVOSPERLEG);    // Ok lets write out our count of servos
		
		for (sSN=0; sSN < sizeof(g_asLegOffsets); sSN++) {
			EEPROM.write(sSN+2, *pb);
			bChkSum += *pb++;
		}
		
		// Then write out to address 1 our calculated checksum
		EEPROM.write(1, bChkSum);
		
		Serial.println("\nFinished saving changes");
	}
	else {
		Serial.println("\nChanges discarded. Loading old configuration");
		void LoadServosConfig();
	}
	
	g_ServoDriver.FreeServos();
}
#endif  // OPT_FIND_SERVO_OFFSETS
