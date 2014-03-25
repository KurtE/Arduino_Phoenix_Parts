//#define DEBUG_ORION
//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver: This version is setup to use the main processor to
//    drive the servos, using my hacked library ServoEx, which is based
//    on the Servo class. 
//====================================================================

#include <Orion.h>

//Servo Pin numbers - May be SSC-32 or actual pins on main controller, depending on configuration.
#ifdef QUADMODE
const byte cCoxaPin[] PROGMEM = {
  cRRCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLFCoxaPin};
const byte cFemurPin[] PROGMEM = {
  cRRFemurPin, cRFFemurPin, cLRFemurPin, cLFFemurPin};
const byte cTibiaPin[] PROGMEM = {
  cRRTibiaPin, cRFTibiaPin, cLRTibiaPin, cLFTibiaPin};
#ifdef c4DOF
const byte cTarsPin[] PROGMEM = {
  cRRTarsPin, cRFTarsPin, cLRTarsPin, cLFTarsPin};
#endif
#else
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
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif
boolean g_fServosAttached;
boolean g_fDebugServos = false;

// Any foreword references
extern boolean MakeSureServosAreOn(void);

//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
  byte bServoIndex;
  g_fServosAttached = false;  // remember we are not attached. Could simply ask one of our servos...
  
  Orion.begin();	// Start up the Orion sub-system.
  FreeServos(); 
  // Orion also handles the servo offsets...
//  Orion.SetLipoCutoff(0);	

#if 0  // The Test/Calibrate program sets these up
  // Some of this could/should go external program and may only need to be done once.
  // Note New code is now doing the inversion of servos before calling here...
#define SERVOUNITSPERDEGREE 178
  byte i;
  for (i=0; i < CNT_LEGS; i++) {
	Orion.setServoDegree(pgm_read_byte(&cCoxaPin[i]), SERVOUNITSPERDEGREE);
	//Orion.SetServoDir(pgm_read_byte(&cCoxaPin[i]), i < (CNT_LEGS/2));
	Orion.setServoDegree(pgm_read_byte(&cFemurPin[i]), SERVOUNITSPERDEGREE);
	//Orion.SetServoDir(pgm_read_byte(&cFemurPin[i]), i < (CNT_LEGS/2));
	Orion.setServoDegree(pgm_read_byte(&cTibiaPin[i]), SERVOUNITSPERDEGREE);
	//Orion.SetServoDir(pgm_read_byte(&cTibiaPin[i]), i < (CNT_LEGS/2));
#ifdef c4DOF
	Orion.setServoDegree(pgm_read_byte(&cTarsPin[i]), SERVOUNITSPERDEGREE);
	//Orion.SetServoDir(pgm_read_byte(&cTarsPin[i]), i < (CNT_LEGS/2));
#endif
  }
#endif  

#ifdef OPT_GPPLAYER
  _fGPEnabled = false;  // starts off assuming that it is not enabled...
  _fGPActive = false;
#endif
    
#ifdef cVoltagePin  
  // If we have a voltage pin, we are doing averaging of voltages over
  // 8 reads, so lets prefill that array...
  for (bServoIndex = 0; bServoIndex < 8; bServoIndex++)
    GetBatteryVoltage();

#endif
}
//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating 
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef cTurnOffVol
#ifdef cVoltagePin  
#ifndef CVADR1
#define CVADR1      30  // VD Resistor 1 - reduced as only need ratio... 30K and 10K
#define CVADR2      10  // VD Resistor 2
#endif

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
word ServoDriver::GetBatteryVoltage(void) {
    return Orion.queryVoltage();
}
#endif

#endif
//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
#ifdef OPT_GPPLAYER

//--------------------------------------------------------------------
//[FIsGPSeqDefined]
//--------------------------------------------------------------------
boolean ServoDriver::FIsGPSeqDefined(uint8_t iSeq)
{
  // for now assume that we don't support GP Player
  return false;
}

//--------------------------------------------------------------------
// Setup to start sequence number...
//--------------------------------------------------------------------
void ServoDriver::GPStartSeq(uint8_t iSeq)
{

}

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
void ServoDriver::GPPlayer(void)
{
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
uint8_t ServoDriver::GPNumSteps(void)          // How many steps does the current sequence have
{
  return 0;
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
uint8_t ServoDriver::GPCurStep(void)           // Return which step currently on... 
{
  return 0xff;
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
void ServoDriver::GPSetSpeedMultiplyer(short sm)      // Set the Speed multiplier (100 is default)
{
}


#endif // OPT_GPPLAYER



//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void ServoDriver::BeginServoUpdate(void)    // Start the update 
{
    MakeSureServosAreOn();
}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#ifdef c4DOF
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1)
#else
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1)
#endif    
{   
    if (g_fDebugServos) {
        DBGSerial.print(sCoxaAngle1, DEC);
        DBGSerial.print(",");
        DBGSerial.print(sFemurAngle1, DEC);
        DBGSerial.print(",");
        DBGSerial.print(sTibiaAngle1, DEC);
        DBGSerial.print(" ");
    } else {
        Orion.setAngle(pgm_read_byte(&cCoxaPin[LegIndex]), sCoxaAngle1);
        Orion.setAngle(pgm_read_byte(&cFemurPin[LegIndex]), sFemurAngle1);
        Orion.setAngle(pgm_read_byte(&cTibiaPin[LegIndex]), sTibiaAngle1);
#ifdef c4DOF
        Orion.setAngle(pgm_read_byte(&cTarsPin[LegIndex]), sTarsAngle1);
#endif
    }
}


//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly 
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriver::CommitServoDriver(word wMoveTime)
{
    if (g_fDebugServos) {
        DBGSerial.print(F(" T: "));
        DBGSerial.println(wMoveTime, DEC);
    } else {
        Orion.setTime(wMoveTime);
        Orion.execute();
    }
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void)
{
	for (byte LegIndex=0; LegIndex < 6; LegIndex++) {
		Orion.stopPulse(pgm_read_byte(&cCoxaPin[LegIndex]));
		Orion.stopPulse(pgm_read_byte(&cFemurPin[LegIndex]));
		Orion.stopPulse(pgm_read_byte(&cTibiaPin[LegIndex]));
#ifdef c4DOF
		Orion.stopPulse(pgm_read_byte(&cTarsPin[LegIndex]));
#endif
	}
	g_fServosAttached = false;
}

//--------------------------------------------------------------------
//[MakeSureServosAreOn] Function that is called to handle when you are
//  transistioning from servos all off to being on.  May need to read
//  in the current pose...
//--------------------------------------------------------------------
boolean MakeSureServosAreOn(void)
{
    if (g_fServosAttached)
      return false;    // Servos are alreaddy attached.

    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...

   //Tell all of the servos to go to where they already are...
    Orion.setTime(0);
	for (byte LegIndex=0; LegIndex < 6; LegIndex++) {
		Orion.setAngle(pgm_read_byte(&cCoxaPin[LegIndex]), Orion.queryFBAngle(pgm_read_byte(&cCoxaPin[LegIndex])));
		Orion.setAngle(pgm_read_byte(&cFemurPin[LegIndex]), Orion.queryFBAngle(pgm_read_byte(&cFemurPin[LegIndex])));
		Orion.setAngle(pgm_read_byte(&cTibiaPin[LegIndex]), Orion.queryFBAngle(pgm_read_byte(&cTibiaPin[LegIndex])));
#ifdef c4DOF
		Orion.setAngle(pgm_read_byte(&cTarsPin[LegIndex]), Orion.queryFBAngle(pgm_read_byte(&cTarsPin[LegIndex])));
#endif
	}
    Orion.execute();

    g_InputController.AllowControllerInterrupts(true);    
    g_fServosAttached = true;
    
    return true;
}

//--------------------------------------------------------------------
//Function that gets called from the main loop if the robot is not logically
//     on.  Gives us a chance to play some...
//--------------------------------------------------------------------
void ServoDriver::IdleTime(void)
{
}

#ifdef OPT_TERMINAL_MONITOR  
extern void FindServoOffsets(void);

//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void ServoDriver::ShowTerminalCommandList(void) 
{
    DBGSerial.println(F("S - Servo Debug Toggle"));
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
  if ((bLen == 1) && ((*psz == 's') || (*psz == 'S'))) {
        if (g_fDebugServos) {
            g_fDebugServos = false;
            DBGSerial.println("Servo Debug Off");
        }
        else {
            g_fDebugServos = true;
            DBGSerial.println("Servo Debug On");
        }
    }
#ifdef OPT_FIND_SERVO_OFFSETS
  if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
    FindServoOffsets();
  }
#endif

}


//=================================================================================================================
#ifdef OPT_FIND_SERVO_OFFSETS
void AllLegServos1500() 
{
	for (byte LegIndex=0; LegIndex < 6; LegIndex++) {
		Orion.setPulse(pgm_read_byte(&cCoxaPin[LegIndex]), 0);
		Orion.setPulse(pgm_read_byte(&cFemurPin[LegIndex]), 0);
		Orion.setPulse(pgm_read_byte(&cTibiaPin[LegIndex]), 0);
#ifdef c4DOF
		Orion.setPulse(pgm_read_byte(&cTarsPin[LegIndex]), 0);
#endif
	}
	Orion.execute();
}


//==============================================================================
// WaitForNoServosMoving
//==============================================================================
void WaitForNoServosMoving(void) {
	while (Orion.queryMove())
		delay(1);
}		

//==============================================================================
// MoveServo
//==============================================================================
void MoveServo(byte iServo, short pulse, word time) {
	Orion.setPulse(iServo, pulse);
	Orion.setTime(time);
	Orion.execute();
}


//==============================================================================
//	FindServoOffsets - Find the zero points for each of our servos... 
// 		Will use the new servo function to set the actual pwm rate and see
//		how well that works...
//==============================================================================

void FindServoOffsets()
{
    // not clean but...
#ifdef QUADMODE
    static char *apszLegs[] = {"RR","RF", "LR", "LF"};  // Leg Order
#else
    static char *apszLegs[] = {"RR","RM","RF", "LR", "LM", "LF"};  // Leg Order
#endif	
    static char *apszLJoints[] = {" Coxa", " Femur", " Tibia", " tArs"}; // which joint on the leg...
    int data;
    short sSN = 0; 			// which servo number
	byte iJoint;
	byte iLeg;
	byte iServo;
    boolean fNew = true;	// is this a new servo to work with?
    boolean fExit = false;	// when to exit
    int ich;
	short asOffsets[24];	// orion has 24 pins... Will worry about Arduino pins later...
    short sOffset;
	short sOffsetOrion;		// What Orion thinks the offsets are...
    
    if (CheckVoltage()) {
        // Voltage is low... 
        Serial.println("Low Voltage: fix or hit $ to abort");
        while (CheckVoltage()) {
            if (Serial.read() == '$')  return;
        }
    }
        
        
    // OK lets move all of the servos to their zero point.
    Serial.println("Find Servo Zeros.\n$-Exit, +- changes, *-change servo");
    Serial.println("    0-5 Chooses a leg, C-Coxa, F-Femur, T-Tibia");
    
    // don't continue here until we have a valid voltage to work with.
    AllLegServos1500();
	for (byte ich=0; ich<24; ich++) {
		asOffsets[ich] = 0;	// All of our servo offsets from what is stored start at 0...
	}

    while(!fExit) {
        if (fNew) {
			iLeg = sSN/NUMSERVOSPERLEG;
			iJoint = sSN%NUMSERVOSPERLEG;
			switch (iJoint) {
			case 0:
				iServo = pgm_read_byte(&cCoxaPin[iLeg]);
				break;
			case 1:
				iServo = pgm_read_byte(&cFemurPin[iLeg]);
				break;
			case 2:
				iServo = pgm_read_byte(&cTibiaPin[iLeg]);
				break;
#ifdef c4DOF
			default:
				iServo = pgm_read_byte(&cTarsPin[iLeg]);
				break;
#endif
			}

            sOffset = asOffsets[iServo];
			sOffsetOrion = Orion.queryPOffset(iServo);
			
            Serial.print("Servo: ");
			Serial.print(apszLegs[iLeg]);
			Serial.print(apszLJoints[iJoint]);
			Serial.print("(");
			Serial.print(sOffset+sOffsetOrion, DEC);
			Serial.println(")"); 

	    // Now lets wiggle the servo
            WaitForNoServosMoving();    // wait for any active servos to finish moving...
            MoveServo(iServo, sOffset+1500, 500);

            WaitForNoServosMoving();    // wait for any active servos to finish moving...
            MoveServo(iServo, sOffset-1500, 500);

            WaitForNoServosMoving();    // wait for any active servos to finish moving...
            MoveServo(iServo, sOffset, 500);
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
		Serial.print(sOffset+sOffsetOrion, DEC);
        asOffsets[iServo] = sOffset;
        MoveServo(iServo, sOffset, 500);
	} else if ((data >= '0') && (data < ( CNT_LEGS +'0'))) {
		// direct enter of which servo to change
		fNew = true;
		sSN = (sSN % NUMSERVOSPERLEG) + (data - '0')*NUMSERVOSPERLEG;
	    } else if ((data == 'c') && (data == 'C')) {
		fNew = true;
		sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 0;
	    } else if ((data == 'c') && (data == 'C')) {
		fNew = true;
		sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 1;
	    } else if ((data == 'c') && (data == 'C')) {
		// direct enter of which servo to change
		fNew = true;
		sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 2;
	    } else if (data == '*') {
	        // direct enter of which servo to change
		fNew = true;
		sSN++;
		if (sSN == 6*NUMSERVOSPERLEG) 
		    sSN = 0;	
	    }
	}
    }
    Serial.print("Find Servo exit ");
    for (sSN=0; sSN < 24; sSN++){
        Serial.print(" ");
        Serial.print(asOffsets[sSN], DEC);
    }

    Serial.print("\nSave Changes? Y/N: ");

    //get user entered data
    while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15)))
	; 

    if ((data == 'Y') || (data == 'y')) {

		for (iLeg = 0; iLeg < 6; iLeg++) {
			iServo = pgm_read_byte(&cCoxaPin[iLeg]);
			Orion.setPOffset(iServo, Orion.queryPOffset(iServo) + asOffsets[iServo]);
			iServo = pgm_read_byte(&cFemurPin[iLeg]);
			Orion.setPOffset(iServo, Orion.queryPOffset(iServo) + asOffsets[iServo]);
			iServo = pgm_read_byte(&cTibiaPin[iLeg]);
			Orion.setPOffset(iServo, Orion.queryPOffset(iServo) + asOffsets[iServo]);
#ifdef c4DOF
			iServo = pgm_read_byte(&cTarsPin[iLeg]);
			Orion.setPOffset(iServo, Orion.queryPOffset(iServo) + asOffsets[iServo]);
#endif
		}
		
		// And tell Orion to save these changes...
		Orion.writeRegisters();
    } else {
        void LoadServosConfig();
    }
    
	g_ServoDriver.FreeServos();

}
#endif  // OPT_FIND_SERVO_OFFSETS
#endif  // Terminal monitor