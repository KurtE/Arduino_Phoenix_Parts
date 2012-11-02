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


//=============================================================================
// Global - Local to this file only...
//=============================================================================
#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif
boolean g_fServosAttached;



//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
  byte bServoIndex;
  g_fServosAttached = false;  // remember we are not attached. Could simply ask one of our servos...
  
  Orion.begin();	// Start up the Orion sub-system.
  // Orion also handles the servo offsets...
  Orion.SetLipoCutoff(0);	

  // Some of this could/should go external program and may only need to be done once.
#define SERVOUNITSPERDEGREE 189  
  byte i;
  for (i=0; i < 6; i++) {
	Orion.SetServoDegree(pgm_read_byte(&cCoxaPin[i]), SERVOUNITSPERDEGREE);
	Orion.SetServoDir(pgm_read_byte(&cCoxaPin[i]), i < 3);
	Orion.SetServoDegree(pgm_read_byte(&cFemurPin[i]), SERVOUNITSPERDEGREE);
	Orion.SetServoDir(pgm_read_byte(&cFemurPin[i]), i < 3);
	Orion.SetServoDegree(pgm_read_byte(&cTibiaPin[i]), SERVOUNITSPERDEGREE);
	Orion.SetServoDir(pgm_read_byte(&cTibiaPin[i]), i < 3);
#ifdef c4DOF
	Orion.SetServoDegree(pgm_read_byte(&cTarsPin[i]), SERVOUNITSPERDEGREE);
	Orion.SetServoDir(pgm_read_byte(&cTarsPin[i]), i < 3);
#endif
  }
  

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
}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
// These are from the SSC32 driver...
#define cPwmDiv       1059 //991  //old 1059;
#define cPFConst      650  //592  //old 650 ; 900*(1000/cPwmDiv)+cPFConst must always be 1500
// A PWM/deg factor of 10,09 give cPwmDiv = 991 and cPFConst = 592
// For a modified 5645 (to 180 deg travel): cPwmDiv = 1500 and cPFConst = 900.

// Will modify this to convert to HSERVO units...
#define MAXHSERVO	14800   // Ran into issue when we go too large...
#define AngleToHSERVO(_Angle_) min((((((long)(_Angle_ +900))*1000/cPwmDiv+cPFConst)-1500)*20), MAXHSERVO)

// This is the calculation done with Arc32 code...
//#define AngleToHSERVO(_Angle_) (((long)_Angle_ * StepsPerDegree) / 10)

#define StepsPerDegree 200
#ifdef c4DOF
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1)
#else
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1)
#endif    
{   
	Orion.SetAngle(pgm_read_byte(&cCoxaPin[LegIndex]), sCoxaAngle1);
	Orion.SetAngle(pgm_read_byte(&cFemurPin[LegIndex]), sFemurAngle1);
	Orion.SetAngle(pgm_read_byte(&cTibiaPin[LegIndex]), sTibiaAngle1);
#ifdef c4DOF
	Orion.SetAngle(pgm_read_byte(&cTarsPin[LegIndex]), sTarsAngle1);
#endif
}


//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly 
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriver::CommitServoDriver(word wMoveTime)
{
	Orion.SetTime(wMoveTime);
    Orion.Execute();

}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void)
{
	for (byte LegIndex=0; LegIndex < 6; LegIndex++) {
		Orion.StopPulse(pgm_read_byte(&cCoxaPin[LegIndex]));
		Orion.StopPulse(pgm_read_byte(&cFemurPin[LegIndex]));
		Orion.StopPulse(pgm_read_byte(&cTibiaPin[LegIndex]));
#ifdef c4DOF
		Orion.StopPulse(pgm_read_byte(&cTarsPin[LegIndex]));
#endif
	}
	g_fServosAttached = false;
}

#ifdef OPT_TERMINAL_MONITOR  
extern void FindServoOffsets(void);

//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void ServoDriver::ShowTerminalCommandList(void) 
{
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
#endif

}


//=================================================================================================================
#ifdef OPT_FIND_SERVO_OFFSETS
void AllLegServos1500() 
{
	for (byte LegIndex=0; LegIndex < 6; LegIndex++) {
		Orion.SetPulse(pgm_read_byte(&cCoxaPin[LegIndex]), 0);
		Orion.SetPulse(pgm_read_byte(&cFemurPin[LegIndex]), 0);
		Orion.SetPulse(pgm_read_byte(&cTibiaPin[LegIndex]), 0);
#ifdef c4DOF
		Orion.SetPulse(pgm_read_byte(&cTarsPin[LegIndex]), 0);
#endif
	}
	Orion.Execute();
}


//==============================================================================
// WaitForNoServosMoving
//==============================================================================
void WaitForNoServosMoving(void) {
	while (Orion.QueryMove())
		delay(1);
}		

//==============================================================================
// MOveServo
//==============================================================================
void MoveServo(byte iServo, short pulse, word time) {
	Orion.SetPulse(iServo, pulse);
	Orion.SetTime(time);
	Orion.Execute();
}


//==============================================================================
//	FindServoOffsets - Find the zero points for each of our servos... 
// 		Will use the new servo function to set the actual pwm rate and see
//		how well that works...
//==============================================================================

void FindServoOffsets()
{
    // not clean but...
    static char *apszLegs[] = {"RR","RM","RF", "LR", "LM", "LF"};  // Leg Order
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
			sOffsetOrion = Orion.QueryPOffset(iServo);
			
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
	} else if ((data >= '0') && (data <= '5')) {
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
			Orion.SetPOffset(iServo, Orion.QueryPOffset(iServo) + asOffsets[iServo]);
			iServo = pgm_read_byte(&cFemurPin[iLeg]);
			Orion.SetPOffset(iServo, Orion.QueryPOffset(iServo) + asOffsets[iServo]);
			iServo = pgm_read_byte(&cTibiaPin[iLeg]);
			Orion.SetPOffset(iServo, Orion.QueryPOffset(iServo) + asOffsets[iServo]);
#ifdef c4DOF
			iServo = pgm_read_byte(&cTarsPin[iLeg]);
			Orion.SetPOffset(iServo, Orion.QueryPOffset(iServo) + asOffsets[iServo]);
#endif
		}
		
		// And tell Orion to save these changes...
		Orion.WriteRegisters();
    } else {
        void LoadServosConfig();
    }
    
	g_ServoDriver.FreeServos();

}
#endif  // OPT_FIND_SERVO_OFFSETS
#endif  // Terminal monitor