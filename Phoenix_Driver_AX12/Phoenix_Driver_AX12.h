//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver - This version is setup to use AX-12 type servos using the
// Arbotix AX12 and bioloid libraries (which may have been updated)
//====================================================================
#include <Arduino.h> // Arduino 1.0

#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif

#ifdef cTurretRotPin
#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS +2)
#else
#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS)
#endif

#define cPwmMult      128
#define cPwmDiv       375  
#define cPFConst      512    // half of our 1024 range

// Some defines for Voltage processing
#define VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE 4000  // Min time in us Until we should do next interpolation, as to not interfer.
#define VOLTAGE_MIN_TIME_BETWEEN_CALLS 150      // Max 6+ times per second
#define VOLTAGE_MAX_TIME_BETWEEN_CALLS 1000    // call at least once per second...
#define VOLTAGE_TIME_TO_ERROR          3000    // Error out if no valid item is returned in 3 seconds...


#define USE_BIOLOIDEX            // Use the Bioloid code to control the AX12 servos...
//#define USE_AX12_SPEED_CONTROL   // Experiment to see if the speed control works well enough...
boolean g_fAXSpeedControl;      // flag to know which way we are doing output...
#include "BioloidEx.h"


#ifdef USE_AX12_SPEED_CONTROL
// Current positions in AX coordinates
word      g_awCurAXPos[NUMSERVOS];
word      g_awGoalAXPos[NUMSERVOS];
#endif

#ifdef DBGSerial
//#define DEBUG
// Only allow debug stuff to be turned on if we have a debug serial port to output to...
//#define DEBUG_SERVOS
#endif

#ifdef DEBUG_SERVOS
#define ServosEnabled   (g_fEnableServos)
#else
#define ServosEnabled  (true)      // always true compiler should remove if...
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
#ifdef QUADMODE
static const byte cPinTable[] PROGMEM = {
  cRRCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLFCoxaPin, 
  cRRFemurPin, cRFFemurPin, cLRFemurPin, cLFFemurPin,
  cRRTibiaPin, cRFTibiaPin, cLRTibiaPin, cLFTibiaPin
#ifdef c4DOF
 ,cRRTarsPin,  cRFTarsPin,  cLRTarsPin,  cLFTarsPin
#endif
#ifdef cTurretRotPin
  , cTurretRotPin, cTurretTiltPin
#endif
};
#else
static const byte cPinTable[] PROGMEM = {
  cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin, 
  cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin,
  cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin
#ifdef c4DOF
    ,cRRTarsPin, cRMTarsPin, cRFTarsPin, cLRTarsPin, cLMTarsPin, cLFTarsPin
#endif
#ifdef cTurretRotPin
  , cTurretRotPin, cTurretTiltPin
#endif
};
#endif
#define FIRSTCOXAPIN     0
#define FIRSTFEMURPIN    (CNT_LEGS)
#define FIRSTTIBIAPIN    (CNT_LEGS*2)
#ifdef c4DOF
#define FIRSTTARSPIN     (CNT_LEGS*3)
#define FIRSTTURRETPIN   (CNT_LEGS*4)
#else
#define FIRSTTURRETPIN   (CNT_LEGS*3)
#endif
// Not sure yet if I will use the controller class or not, but...
BioloidControllerEx bioloid = BioloidControllerEx(1000000);
boolean g_fServosFree;    // Are the servos in a free state?


//============================================================================================
// Lets try rolling our own GPSequence code here...
#define GPSEQ_EEPROM_START 0x40       // Reserve the first 64 bytes of EEPROM for other stuff...
#define GPSEQ_EEPROM_START_DATA  0x50 // Reserved room for up to 8 in header...
#define GPSEQ_EEPROM_SIZE 0x800       // I think we have 2K
#define GPSEQ_EEPROM_MAX_SEQ 5        // For now this is probably the the max we can probably hold...


// Not sure if pragma needed or not...
//#pragma pack(1)
typedef struct {
  byte  bSeqNum;       // the sequence number, used to verify
  byte  bCntServos;    // count of servos
  byte  bCntSteps;     // How many steps there are
  byte  bCntPoses;     // How many poses
}
EEPromPoseHeader;

typedef struct {
  byte bPoseNum;        // which pose to use
  word wTime;        // Time to do pose
}
EEPROMPoseSeq;      // This is a sequence entry

// Pose is just an array of words...


// A sequence is stored as:
//<header> <sequences><poses>



// Some forward references
extern void MakeSureServosAreOn(void);
extern void DoPyPose(byte *psz);
extern void EEPROMReadData(word wStart, uint8_t *pv, byte cnt);
extern void EEPROMWriteData(word wStart, uint8_t *pv, byte cnt);
extern void TCSetServoID(byte *psz);
extern void TCTrackServos();
extern void SetRegOnAllServos(uint8_t bReg, uint8_t bVal);


//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
  // First lets get the actual servo positions for all of our servos...
  //  pinMode(0, OUTPUT);
  g_fServosFree = true;
  bioloid.poseSize = NUMSERVOS;
  bioloid.readPose();
#ifdef cVoltagePin  
  for (byte i=0; i < 8; i++)
    GetBatteryVoltage();  // init the voltage pin
#endif

  g_fAXSpeedControl = false;

#ifdef OPT_GPPLAYER
  _fGPEnabled = true;    // assume we support it.
#endif

  // Currently have Turret pins not necessarily same as numerical order so
  // Maybe should do for all pins and then set the positions by index instead
  // of having it do a simple search on each pin...
#ifdef cTurretRotPin
  bioloid.setId(FIRSTTURRETPIN, cTurretRotPin);
  bioloid.setId(FIRSTTURRETPIN+1, cTurretTiltPin);
#endif

  // Added - try to speed things up later if we do a query...
  SetRegOnAllServos(AX_RETURN_DELAY_TIME, 0);  // tell servos to give us back their info as quick as they can...

}


//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating 
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef cVoltagePin  
uint16_t  g_awVoltages[8]={
  0,0,0,0,0,0,0,0};
uint16_t  g_wVoltageSum = 0;
byte  g_iVoltages = 0;

uint16_t ServoDriver::GetBatteryVoltage(void) {
  g_iVoltages = (++g_iVoltages)&0x7;  // setup index to our array...
  g_wVoltageSum -= g_awVoltages[g_iVoltages];
  g_awVoltages[g_iVoltages] = analogRead(cVoltagePin);
  g_wVoltageSum += g_awVoltages[g_iVoltages];

#ifdef CVREF
  return ((long)((long)g_wVoltageSum*CVREF*(CVADR1+CVADR2))/(long)(8192*(long)CVADR2));  
#else
  return ((long)((long)g_wVoltageSum*125*(CVADR1+CVADR2))/(long)(2048*(long)CVADR2));  
#endif
}

#else
uint16_t g_wLastVoltage = 0xffff;    // save the last voltage we retrieved...
byte g_bLegVoltage = 0;		// what leg did we last check?
unsigned long g_ulTimeLastBatteryVoltage;

uint16_t ServoDriver::GetBatteryVoltage(void) {
  // In this case, we have to ask a servo for it's current voltage level, which is a lot more overhead than simply doing
  // one AtoD operation.  So we will limit when we actually do this to maybe a few times per second.  
  // Also if interpolating, the code will try to only call us when it thinks it won't interfer with timing of interpolation.
  unsigned long ulDeltaTime = millis() - g_ulTimeLastBatteryVoltage;
  if (g_wLastVoltage != 0xffff) {
      if ( (ulDeltaTime < VOLTAGE_MIN_TIME_BETWEEN_CALLS) 
            || (bioloid.interpolating &&  (ulDeltaTime < VOLTAGE_MAX_TIME_BETWEEN_CALLS)))
        return g_wLastVoltage;
  }

  // Lets cycle through the Tibia servos asking for voltages as they may be the ones doing the most work...
  register uint16_t wVoltage = ax12GetRegister (pgm_read_byte(&cPinTable[FIRSTTIBIAPIN+g_bLegVoltage]), AX_PRESENT_VOLTAGE, 1);
  if (++g_bLegVoltage >= CNT_LEGS)
	g_bLegVoltage = 0;
  if (wVoltage != 0xffff) {
      g_ulTimeLastBatteryVoltage = millis();
      g_wLastVoltage = wVoltage * 10;
      return g_wLastVoltage;
  }

  // Allow it to error our a few times, but if the time until we get a valid response exceeds some time limit then error out.
  if (ulDeltaTime < VOLTAGE_TIME_TO_ERROR)
    return g_wLastVoltage;
  return 0;

}
#endif

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
#ifdef OPT_GPPLAYER
EEPromPoseHeader g_eepph;  // current header 
byte g_bSeqStepNum;
word g_wSeqHeaderStart;
boolean g_fSeqProgmem;
transition_t *g_ptransCur;    // pointer to our current transisiton...

boolean fRobotUpsideDownGPStart;  // state when we start sequence
#ifdef USE_PYPOSE_HEADER
#define CNT_PYPOSE_SEGS (sizeof(PoseList)/sizeof(PoseList[0]))
#else
#define CNT_PYPOSE_SEGS 0
#endif
//--------------------------------------------------------------------
//[FIsGPSeqDefined]
//--------------------------------------------------------------------
boolean ServoDriver::FIsGPSeqDefined(uint8_t iSeq)
{
#ifdef USE_PYPOSE_HEADER
  if (iSeq < CNT_PYPOSE_SEGS) {
    g_fSeqProgmem = true;
    g_ptransCur = (transition_t *)pgm_read_word(&PoseList[iSeq]);
    // First entry in this table has the count of poses.
    g_eepph.bCntSteps = (byte)pgm_read_word(&(g_ptransCur->time));
    g_eepph.bCntServos = NUMSERVOS;

    return true;  // say that we are valid...
  }
  g_fSeqProgmem = false;
#endif
  iSeq -= CNT_PYPOSE_SEGS;  // update count to subtract off fixed ones.
  if (iSeq >= GPSEQ_EEPROM_MAX_SEQ)
    return false;

  // Now read in the header pointer...
  EEPROMReadData(GPSEQ_EEPROM_START+iSeq*sizeof(word), (uint8_t*)&g_wSeqHeaderStart, sizeof(g_wSeqHeaderStart));  

  if ((g_wSeqHeaderStart < GPSEQ_EEPROM_START_DATA) || (g_wSeqHeaderStart >= GPSEQ_EEPROM_SIZE))
    return false;  // pointer does not look right.

  // Now Read in the actual header
  EEPROMReadData(g_wSeqHeaderStart, (uint8_t*)&g_eepph, sizeof(g_eepph));

  if ((g_eepph.bSeqNum != iSeq) || (g_eepph.bCntServos != NUMSERVOS) ||
    ((g_wSeqHeaderStart + sizeof(g_eepph) + (g_eepph.bCntSteps * sizeof(EEPROMPoseSeq)) + (g_eepph.bCntPoses * sizeof(word) * NUMSERVOS)) >= GPSEQ_EEPROM_SIZE))
    return false;

  return true;  // Looks like it is valid
}


//--------------------------------------------------------------------
// Setup to start sequence number...
//--------------------------------------------------------------------
void ServoDriver::GPStartSeq(uint8_t iSeq)
{
  if ((iSeq == 0xff) && _fGPActive) {
    // Caller is asking us to abort... 
    // I think I can simply clear our active flag and it will cleanup...
    // May need to find a way to clear the interpolating...
    _fGPActive = false;
    return;
  }
  // Use our Validate function to get the initial stuff set up...
  if (!FIsGPSeqDefined(iSeq))
    return;
  _fGPActive = true;
  _iSeq = iSeq;
  g_bSeqStepNum = 0xff;  // Say that we are not in a step yet...
  _sGPSM = 100;          // assume we are running at standard speed
  fRobotUpsideDownGPStart = g_fRobotUpsideDown;
}

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
static const byte cPinIndexTranslateUpsideDownTable[] PROGMEM = {
  0x80+1, 0x80+0, 3, 2, 5, 4, 0x80+7, 0x80+6, 9, 8, 11, 10, 0x80+13, 0x80+12, 15, 14, 17, 16}; 

void ServoDriver::GPPlayer(void)
{
  EEPROMPoseSeq eepps;
  byte bServo;
  word wPosePos;
  byte bServoIndexUpsideDown;

  if (_fGPActive) {
    // See if we are still interpolating the last step
    if ((g_bSeqStepNum != 0xff) && (bioloid.interpolating)) {
      bioloid.interpolateStep(false);
      return;
    }

    if (_sGPSM >= 0) {
      if (++g_bSeqStepNum >= g_eepph.bCntSteps) {
        _fGPActive = false;  // we are done
        return;
      }
    }
    else {
      // Run in reverse
      if ((g_bSeqStepNum == 0xff) || g_bSeqStepNum == 0) {
        _fGPActive = false;  // we are done
        return;
      }
      g_bSeqStepNum--;
    }

#ifdef USE_PYPOSE_HEADER
    if (g_fSeqProgmem) {

      if (_sGPSM >= 0) 
        g_ptransCur++;  // lets point to the next step entry.
      else
        g_ptransCur--;  // lets point to the next step entry.

      word *pwPose = (word*)pgm_read_word(&(g_ptransCur->pose));

      int poseSize = pgm_read_word_near(pwPose); // number of servos in this pose

        if (!fRobotUpsideDownGPStart) {
        for(bServo=0; bServo<poseSize; bServo++) {
          bioloid.setNextPoseByIndex(bServo, pgm_read_word_near(pwPose+1+bServo));  // set a servo value by index for next pose
        }
      }
      else {
        // Upside down 
        for(bServo=0; bServo<poseSize; bServo++) {
          bServoIndexUpsideDown = pgm_read_byte(&cPinIndexTranslateUpsideDownTable[bServo]);
          if (bServoIndexUpsideDown & 0x80)
            bioloid.setNextPoseByIndex(bServoIndexUpsideDown&0x7f, 1023-pgm_read_word_near(pwPose+1+bServo));  
          else
            bioloid.setNextPoseByIndex(bServoIndexUpsideDown, pgm_read_word_near(pwPose+1+bServo));  
        }
      }
      // interpolate - Note: we want to take the Speed multipler into account here. 
      bioloid.interpolateSetup(((long)pgm_read_word(&(g_ptransCur->time))*100)/abs(_sGPSM));
      return;
    }
#endif
    // Lets get the sequence information
    EEPROMReadData(g_wSeqHeaderStart + sizeof(g_eepph) + (g_bSeqStepNum*sizeof(EEPROMPoseSeq)), (uint8_t*)&eepps, sizeof(eepps));

    // Now lets setup to read in the pose information
    word wEEPromPoseLoc = g_wSeqHeaderStart + sizeof(g_eepph) + (g_eepph.bCntPoses*sizeof(EEPROMPoseSeq)) + (eepps.bPoseNum * sizeof(word) * NUMSERVOS); 
    for (bServo=0; bServo < NUMSERVOS; bServo++) {
      EEPROMReadData(wEEPromPoseLoc, (uint8_t*)&wPosePos, sizeof(wPosePos));
      if (!fRobotUpsideDownGPStart) {
        bioloid.setNextPoseByIndex(bServo, wPosePos);  // set a servo value by index for next pose
      }
      else {
        bServoIndexUpsideDown = pgm_read_byte(&cPinIndexTranslateUpsideDownTable[bServo]);
        if (bServoIndexUpsideDown & 0x80)
          bioloid.setNextPoseByIndex(bServoIndexUpsideDown&0x7f, 1023-wPosePos);  
        else
          bioloid.setNextPoseByIndex(bServoIndexUpsideDown, wPosePos);  
      }        
      //      bioloid.setNextPose(bServo+1,wPosePos);
      wEEPromPoseLoc += sizeof(word);
    }

    // interpolate
    bioloid.interpolateSetup((((long)eepps.wTime)*100)/abs(_sGPSM));
  }
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
uint8_t ServoDriver::GPNumSteps(void)          // How many steps does the current sequence have
{
  return _fGPActive ? g_eepph.bCntSteps : 0;
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
uint8_t ServoDriver::GPCurStep(void)           // Return which step currently on... 
{
  return _fGPActive ? g_bSeqStepNum + 1 : 0xff;
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
void ServoDriver::GPSetSpeedMultiplyer(short sm)      // Set the Speed multiplier (100 is default)
{
  _sGPSM = sm;
}



#endif // OPT_GPPLAYER

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
      bioloid.interpolateStep(true);    // Make sure we call at least once

  }
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
  word    wCoxaSDV;        // Coxa value in servo driver units
  word    wFemurSDV;        //
  word    wTibiaSDV;        //
#ifdef c4DOF
  word    wTarsSDV;        //
#endif
  // The Main code now takes care of the inversion before calling.
  wCoxaSDV = (((long)(sCoxaAngle1))* cPwmMult) / cPwmDiv +cPFConst;
  wFemurSDV = (((long)((long)(sFemurAngle1))* cPwmMult) / cPwmDiv +cPFConst);
  wTibiaSDV = (((long)(sTibiaAngle1))* cPwmMult) / cPwmDiv +cPFConst;
#ifdef c4DOF
  wTarsSDV = (((long)(sTarsAngle1))* cPwmMult) / cPwmDiv +cPFConst;
#endif
  if (ServosEnabled) {
    if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // Save away the new positions... 
      g_awGoalAXPos[FIRSTCOXAPIN+LegIndex] = wCoxaSDV;    // What order should we store these values?
      g_awGoalAXPos[FIRSTFEMURPIN+LegIndex] = wFemurSDV;    
      g_awGoalAXPos[FIRSTTIBIAPIN+LegIndex] = wTibiaSDV;    
#ifdef c4DOF
      g_awGoalAXTarsPos[FIRSTTARSPIN+LegIndex] = wTarsSDV;    
#endif
#endif
    }
    else {    
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTCOXAPIN+LegIndex]), wCoxaSDV);
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTFEMURPIN+LegIndex]), wFemurSDV);
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTIBIAPIN+LegIndex]), wTibiaSDV);
#ifdef c4DOF
      if ((byte)pgm_read_byte(&cTarsLength[LegIndex]))   // We allow mix of 3 and 4 DOF legs...
        bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTARSPIN+LegIndex]), wTarsSDV);
#endif
    }
  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput) {
    DBGSerial.print(LegIndex, DEC);
    DBGSerial.print("(");
    DBGSerial.print(sCoxaAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wCoxaSDV, DEC);
    DBGSerial.print("),(");
    DBGSerial.print(sFemurAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wFemurSDV, DEC);
    DBGSerial.print("),(");
    DBGSerial.print("(");
    DBGSerial.print(sTibiaAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wTibiaSDV, DEC);
    DBGSerial.print(") :");
  }
#endif
  g_InputController.AllowControllerInterrupts(true);    // Ok for hserial again...
}


//==============================================================================
// Calculate servo speeds to achieve desired pose timing
// We make the following assumptions:
// AX-12 speed is 59rpm @ 12V which corresponds to 0.170s/60deg
// The AX-12 manual states this as the 'no load speed' at 12V
// The Moving Speed control table entry states that 0x3FF = 114rpm
// and according to Robotis this means 0x212 = 59rpm and anything greater 0x212 is also 59rpm
#ifdef USE_AX12_SPEED_CONTROL
word CalculateAX12MoveSpeed(word wCurPos, word wGoalPos, word wTime)
{
  word wTravel;
  uint32_t factor;
  word wSpeed;
  // find the amount of travel for each servo
  if( wGoalPos > wCurPos) {
    wTravel = wGoalPos - wCurPos;
  } 
  else {
    wTravel = wCurPos - wGoalPos;
  }

  // now we can calculate the desired moving speed
  // for 59pm the factor is 847.46 which we round to 848
  // we need to use a temporary 32bit integer to prevent overflow
  factor = (uint32_t) 848 * wTravel;

  wSpeed = (uint16_t) ( factor / wTime );
  // if the desired speed exceeds the maximum, we need to adjust
  if (wSpeed > 1023) wSpeed = 1023;
  // we also use a minimum speed of 26 (5% of 530 the max value for 59RPM)
  if (wSpeed < 26) wSpeed = 26;

  return wSpeed;
} 
#endif

//------------------------------------------------------------------------------------------
//[OutputServoInfoForTurret] Set up the outputse servos associated with an optional turret
//         the Leg number passed in.  FIRSTTURRETPIN
//------------------------------------------------------------------------------------------
#ifdef cTurretRotPin
void ServoDriver::OutputServoInfoForTurret(short sRotateAngle1, short sTiltAngle1)
{        
  word    wRotateSDV;      
  word    wTiltSDV;        //

  // The Main code now takes care of the inversion before calling.
  wRotateSDV = (((long)(sRotateAngle1))* cPwmMult) / cPwmDiv +cPFConst;
  wTiltSDV = (((long)((long)(sTiltAngle1))* cPwmMult) / cPwmDiv +cPFConst);

  if (ServosEnabled) {
    if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // Save away the new positions... 
      g_awGoalAXPos[FIRSTTURRETPIN] = wRotateSDV;    // What order should we store these values?
      g_awGoalAXPos[FIRSTTURRETPIN+1] = wTiltSDV;    
#endif
    }
    else {    
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTURRETPIN]), wRotateSDV);
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTURRETPIN+1]), wTiltSDV);
    }
  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput) {
    DBGSerial.print("(");
    DBGSerial.print(sRotateAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wRotateSDV, DEC);
    DBGSerial.print("),(");
    DBGSerial.print(sTiltAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wTiltSDV, DEC);
    DBGSerial.print(") :");
  }
#endif
}
#endif
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
#ifdef USE_AX12_SPEED_CONTROL
      // Need to first output the header for the Sync Write
      int length = 4 + (NUMSERVOS * 5);   // 5 = id + pos(2byte) + speed(2 bytes)
      int checksum = 254 + length + AX_SYNC_WRITE + 4 + AX_GOAL_POSITION_L;
      word wSpeed;
      setTXall();
      ax12write(0xFF);
      ax12write(0xFF);
      ax12write(0xFE);
      ax12write(length);
      ax12write(AX_SYNC_WRITE);
      ax12write(AX_GOAL_POSITION_L);
      ax12write(4);    // number of bytes per servo (plus the ID...)
      for (int i = 0; i < NUMSERVOS; i++) {
        wSpeed = CalculateAX12MoveSpeed(g_awCurAXPos[i], g_awGoalAXPos[i], wMoveTime);    // What order should we store these values?
        byte id = pgm_read_byte(&cPinTable[i]);
        checksum += id + (g_awGoalAXPos[i]&0xff) + (g_awGoalAXPos[i]>>8) + (wSpeed>>8) + (wSpeed & 0xff);
        ax12write(id);
        ax12write(g_awGoalAXPos[i]&0xff);
        ax12write(g_awGoalAXPos[i]>>8);
        ax12write(wSpeed&0xff);
        ax12write(wSpeed>>8);

      }
      ax12write(0xff - (checksum % 256));
      setRX(0);

#endif
    }
    else {
      bioloid.interpolateSetup(wMoveTime);
    }

  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput)
    DBGSerial.println(wMoveTime, DEC);
#endif
  g_InputController.AllowControllerInterrupts(true);    

}
//--------------------------------------------------------------------
//[SetRegOnAllServos] Function that is called to set the state of one
//  register in all of the servos, like Torque on...
//--------------------------------------------------------------------
void SetRegOnAllServos(uint8_t bReg, uint8_t bVal)
{
  // Need to first output the header for the Sync Write
  int length = 4 + (NUMSERVOS * 2);   // 2 = id + val
  int checksum = 254 + length + AX_SYNC_WRITE + 1 + bReg;
  setTXall();
  ax12write(0xFF);
  ax12write(0xFF);
  ax12write(0xFE);
  ax12write(length);
  ax12write(AX_SYNC_WRITE);
  ax12write(bReg);
  ax12write(1);    // number of bytes per servo (plus the ID...)
  for (int i = 0; i < NUMSERVOS; i++) {
    byte id = pgm_read_byte(&cPinTable[i]);
    checksum += id + bVal;
    ax12write(id);
    ax12write(bVal);

  }
  ax12write(0xff - (checksum % 256));
  setRX(0);
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void)
{
  if (!g_fServosFree) {
    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    SetRegOnAllServos(AX_TORQUE_ENABLE, 0);  // do this as one statement...
#if 0    
    for (byte i = 0; i < NUMSERVOS; i++) {
      Relax(pgm_read_byte(&cPinTable[i]));
    }
#endif
    g_InputController.AllowControllerInterrupts(true);    
    g_fServosFree = true;
  }
}

//--------------------------------------------------------------------
//Function that gets called from the main loop if the robot is not logically
//     on.  Gives us a chance to play some...
//--------------------------------------------------------------------
static uint8_t g_iIdleServoNum  = (uint8_t)-1;
static uint8_t g_iIdleLedState = 1;  // what state to we wish to set...
void ServoDriver::IdleTime(void)
{
  // Each time we call this set servos LED on or off...
  g_iIdleServoNum++;
  if (g_iIdleServoNum >= NUMSERVOS) {
    g_iIdleServoNum = 0;
    g_iIdleLedState = 1 - g_iIdleLedState;
  }
  ax12SetRegister(pgm_read_byte(&cPinTable[g_iIdleServoNum]), AX_LED, g_iIdleLedState);
  ax12ReadPacket(6);  // get the response...

}

//--------------------------------------------------------------------
//[MakeSureServosAreOn] Function that is called to handle when you are
//  transistioning from servos all off to being on.  May need to read
//  in the current pose...
//--------------------------------------------------------------------
void MakeSureServosAreOn(void)
{
  if (ServosEnabled) {
    if (!g_fServosFree)
      return;    // we are not free

    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    if (g_fAXSpeedControl) {
      for(int i=0;i<NUMSERVOS;i++){
        g_awGoalAXPos[i] = ax12GetRegister(pgm_read_byte(&cPinTable[i]),AX_PRESENT_POSITION_L,2);
        delay(25);   
      }
    }
    else {
      bioloid.readPose();
    }

    SetRegOnAllServos(AX_TORQUE_ENABLE, 1);  // Use sync write to do it.

#if 0
    for (byte i = 0; i < NUMSERVOS; i++) {
      TorqueOn(pgm_read_byte(&cPinTable[i]));
    }
#endif    
    g_InputController.AllowControllerInterrupts(true);    
    g_fServosFree = false;
  }   
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

    int iTimeToNextInterpolate = bioloid.interpolateStep(false);    // Do our background stuff...
    
    // Hack if we are not interpolating, maybe try to get voltage.  This will acutally only do this
    // a few times per second.
#ifdef cTurnOffVol          // only do if we a turn off voltage is defined
#ifndef cVoltagePin         // and we are not doing AtoD type of conversion...
    if (iTimeToNextInterpolate > VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE )      // At least 4ms until next interpolation.  See how this works...
        GetBatteryVoltage();
#endif    
#endif
  }
}


#ifdef OPT_TERMINAL_MONITOR  
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void ServoDriver::ShowTerminalCommandList(void) 
{
  DBGSerial.println(F("V - Voltage"));
  DBGSerial.println(F("M - Toggle Motors on or off"));
  DBGSerial.println(F("F<frame length> - FL in ms"));    // BUGBUG:: 
  DBGSerial.println(F("A - Toggle AX12 speed control"));
  DBGSerial.println(F("T - Test Servos"));
  DBGSerial.println(F("I - Set Id <frm> <to"));
  DBGSerial.println(F("S - Track Servos"));
#ifdef OPT_PYPOSE
  DBGSerial.println(F("P<DL PC> - Pypose"));
#endif
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
  if ((bLen == 1) && ((*psz == 'm') || (*psz == 'M'))) {
    g_fEnableServos = !g_fEnableServos;
    if (g_fEnableServos) 
      DBGSerial.println(F("Motors are on"));
    else
      DBGSerial.println(F("Motors are off"));

    return true;  
  } 
  if ((bLen == 1) && ((*psz == 'v') || (*psz == 'V'))) {
    DBGSerial.print(F("Voltage: "));
    DBGSerial.println(GetBatteryVoltage(), DEC);
#ifdef cVoltagePIN
    DBGSerial.print("Raw Analog: ");
    DBGSerial.println(analogRead(cVoltagePin));
#endif
    DBGSerial.print(F("From Servo 2: "));
    DBGSerial.println(ax12GetRegister (2, AX_PRESENT_VOLTAGE, 1), DEC);    
  }

  if ((bLen == 1) && ((*psz == 't') || (*psz == 'T'))) {
    // Test to see if all servos are responding...
    for(int i=1;i<=NUMSERVOS;i++){
      int iPos;
      iPos = ax12GetRegister(i,AX_PRESENT_POSITION_L,2);
      DBGSerial.print(i,DEC);
      DBGSerial.print(F("="));
      DBGSerial.println(iPos, DEC);
      delay(25);   
    }
  }
  if ((*psz == 'i') || (*psz == 'I')) {
    TCSetServoID(++psz);
  }
  if ((*psz == 's') || (*psz == 'S')) {
    TCTrackServos();
  }

  if ((bLen == 1) && ((*psz == 'a') || (*psz == 'A'))) {
    g_fAXSpeedControl = !g_fAXSpeedControl;
    if (g_fAXSpeedControl) 
      DBGSerial.println(F("AX12 Speed Control"));
    else
      DBGSerial.println(F("Bioloid Speed"));
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
      extern BioloidControllerEx bioloid;
      bioloid.frameLength = bFrame;
    }
  } 

#ifdef OPT_FIND_SERVO_OFFSETS
  else if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
    FindServoOffsets();
  }
#endif
   return false;

}

//==============================================================================
// TCSetServoID - debug function to update servo numbers.
//==============================================================================
void TCSetServoID(byte *psz)
{
  word wFrom = GetCmdLineNum(&psz);
  word wTo = GetCmdLineNum(&psz);

  if (wFrom  && wTo) {
    // Both specified, so lets try
    DBGSerial.print("Change Servo from: ");
    DBGSerial.print(wFrom, DEC);
    DBGSerial.print(" ");
    DBGSerial.print(wTo, DEC);
    ax12SetRegister(wFrom, AX_ID, wTo);
    if (ax12ReadPacket(6)) { // get the response...    
      DBGSerial.print(" Resp: ");
      DBGSerial.println(ax_rx_buffer[4], DEC);
    } 
    else
      DBGSerial.println(" failed");
  }
}

//==============================================================================
// TCTrackServos - Lets set a mode to track servos.  Can use to help figure out
// proper initial positions and min/max values...
//==============================================================================
void TCTrackServos()
{
  // First read through all of the servos to get their position. 
  uint16_t auPos[NUMSERVOS];
  uint16_t  uPos;
  int i;
  boolean fChange;

  // Clear out any pending input characters
  while (DBGSerial.read() != -1)
    ;

  for(i=0;i<NUMSERVOS;i++){
    auPos[i] = ax12GetRegister(pgm_read_byte(&cPinTable[i]),AX_PRESENT_POSITION_L,2);
  }  

  // Now loop until we get some input on the serial
  while (!DBGSerial.available()) {
    fChange = false;
    for(int i=0; i<NUMSERVOS; i++){
      uPos = ax12GetRegister(pgm_read_byte(&cPinTable[i]),AX_PRESENT_POSITION_L,2);
      // Lets put in a littl delta or shows lots
      if (abs(auPos[i] - uPos) > 2) {
        auPos[i] = uPos;
        if (fChange)
          DBGSerial.print(", ");
        else
          fChange = true;  
        DBGSerial.print(pgm_read_byte(&cPinTable[i]), DEC);
        DBGSerial.print(": ");
        DBGSerial.print(uPos, DEC);
        // Convert back to angle. 
        int iAng = (((int)uPos-cPFConst)*cPwmDiv)/cPwmMult;
        DBGSerial.print("(");
        DBGSerial.print(iAng, DEC);
        DBGSerial.print(")");
      }
    }  
    if (fChange)
      DBGSerial.println();
    delay(25);   
  }

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

}
#endif  // OPT_FIND_SERVO_OFFSETS

//==============================================================================
// EEPromReadData - Quick and dirty function to read multiple bytes in from
//  eeprom...
//==============================================================================
void EEPROMReadData(word wStart, uint8_t *pv, byte cnt) {
  while (cnt--) {
    *pv++ = EEPROM.read(wStart++);
  }
}

//==============================================================================
// DoPyPose - This is based off of the Pypose sketch...
// ArbotiX Test Program for use with PyPose 0013
// Copyright (c) 2008-2010 Michael E. Ferguson.  All right reserved.
//
// The code was put onto a memory diet and extended by me...
//==============================================================================
#ifdef OPT_PYPOSE

// Some defines for different modes and commands
#define ARB_SIZE_POSE   7  // also initializes
#define ARB_LOAD_POSE   8
#define ARB_LOAD_SEQ    9
#define ARB_PLAY_SEQ    10
#define ARB_LOOP_SEQ    11
#define ARB_SAVE_EEPROM_SEQ 12
#define ARB_TEST        25

// Global to this function...
byte   g_bPoseSize = NUMSERVOS;  

short g_poses[540];              // enough for 30 steps...
typedef struct{
  byte  pose;    // index of pose to transition to 
  word time;              // time for transition
} 
sp_trans_t;

sp_trans_t g_sequence[30];   // sequence
byte g_bParams[90];  // parameters

// Some forward referencs
extern boolean PyPoseSaveToEEPROM(byte);



void DoPyPose(byte *psz)
{
  int mode = 0;              // where we are in the frame

  byte id = 0;      // id of this frame
  byte length = 0;  // length of this frame
  byte ins = 0;     // instruction of this frame

  byte index = 0;   // index in param buffer
  word wPoseIndex;

  int checksum = 0;              // checksum


    //  pose and sequence storage
  // Put on diet - convert int to short plus convert poses from 2 dimensions to 1 dimension...
  //
  byte seqPos;                // step in current sequence

  // See if the user gave us a string to process.  If so it should be the XBEE DL to talk to. 
  // BUGBUG:: if using our XBEE stuff could default to debug terminal... 
#ifdef USEXBEE

#ifdef DBGSerial
  word wMY;
  wMY = GetXBeeHVal('M', 'Y');

  DBGSerial.print(F("My: "));
  DBGSerial.println(wMY, HEX);  
  wMY = GetXBeeHVal('D', 'L');  // I know reused variable...
  DBGSerial.print(F("PC DL: "));
  DBGSerial.println(wMY, HEX);
  DBGSerial.println(F("Exit Terminal and Start Pypose"));
  DBGSerial.println(F("$TEXTM$"));  // add some support in VB side to convert to text mode...
#endif
  // Now lets reset our xbee...
  APISendXBeeGetCmd('F','R');  // Send a Forced reset...
  delay(2000);
  while(Serial.read() != -1)
    ;

  if (psz) {
    word wDL;
    for (wDL=0; *psz; psz++) {
      if ((*psz >= '0') && (*psz <= '9'))
        wDL = (wDL << 4) + *psz - '0';
      if ((*psz >= 'a') && (*psz <= 'f'))
        wDL = (wDL << 4) + *psz - 'a' + 10;
      if ((*psz >= 'A') && (*psz <= 'F'))
        wDL = (wDL << 4) + *psz - 'A' + 10;
    }
    if (wDL) {

      Serial.print("+++");
      delay (1000);
      while ((checksum=Serial.read()) != -1) {
        //Serial.write((byte)checksum);
      }
      // Now lets set the DL and exit command mode...
      Serial.print("ATDL ");
      Serial.print(wDL, HEX);
      Serial.println(",ATCN");
    }
  }
  delay(10);
  while ((checksum=Serial.read()) != -1) {
    //Serial.write((byte)checksum);
  }

#endif    

  // process messages
  for (;;) {
    while(Serial.available() > 0){
      // We need to 0xFF at start of packet
      if(mode == 0){         // start of new packet
        if(Serial.read() == 0xff){
          mode = 2;
//          digitalWrite(0,HIGH-digitalRead(0));
        }
        //}else if(mode == 1){   // another start byte
        //    if(Serial.read() == 0xff)
        //        mode = 2;
        //    else
        //        mode = 0;
      }
      else if(mode == 2){   // next byte is index of servo
        id = Serial.read();    
        if(id != 0xff)
          mode = 3;
      }
      else if(mode == 3){   // next byte is length
        length = Serial.read();
        checksum = id + length;
        mode = 4;
      }
      else if(mode == 4){   // next byte is instruction
        ins = Serial.read();
        checksum += ins;
        index = 0;
        mode = 5;
      }
      else if(mode == 5){   // read data in 
        g_bParams[index] = Serial.read();
        checksum += (int) g_bParams[index];
        index++;
        if(index + 1 == length){  // we've read params & checksum
          mode = 0;
          if((checksum%256) != 255){ 
            // return a packet: FF FF id Len Err params=None check
            Serial.write((byte)0xff);
            Serial.write((byte)0xff);
            Serial.write((byte)id);
            Serial.write((byte)2);
            Serial.write((byte)64);
            Serial.write((byte)(255-((66+id)%256)));
          }
          else{
            if(id == 253){
              // return a packet: FF FF id Len Err params=None check
              Serial.write((byte)0xff);
              Serial.write((byte)0xff);
              Serial.write((byte)id);
              Serial.write((byte)2);
              Serial.write((byte)0);
              Serial.write((byte)(255-((2+id)%256)));
              // special ArbotiX instructions
              // Pose Size = 7, followed by single param: size of pose
              // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size)
              // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size) 
              // Play Seq = A, no params
              if(ins == ARB_SIZE_POSE){
                g_bPoseSize = bioloid.poseSize = g_bParams[0];
                bioloid.readPose();    
                //Serial.println(bioloid.poseSize);
              }
              else if(ins == ARB_LOAD_POSE){
                int i;    
                //Serial.print("New Pose:");
                wPoseIndex = g_bParams[0] * g_bPoseSize;
                for(i=0; i<bioloid.poseSize; i++){
                  g_poses[wPoseIndex+i] = g_bParams[(2*i)+1]+(g_bParams[(2*i)+2]<<8); 
                  //Serial.print(g_poses[g_bParams[0]][i]);
                  //Serial.print(",");     
                } 
                //Serial.println("");
              }
              else if(ins == ARB_LOAD_SEQ){
                int i;
                for(i=0;i<(length-2)/3;i++){
                  g_sequence[i].pose = g_bParams[(i*3)];
                  g_sequence[i].time = g_bParams[(i*3)+1] + (g_bParams[(i*3)+2]<<8);
                  //Serial.print("New Transition:");
                  //Serial.print((int)g_sequence[i].pose);
                  //Serial.print(" in ");
                  //Serial.println(g_sequence[i].time);      
                }
              }
              else if(ins == ARB_PLAY_SEQ){
                seqPos = 0;
                while(g_sequence[seqPos].pose != 0xff){
                  int i;
                  int p = g_sequence[seqPos].pose;
                  wPoseIndex = p * g_bPoseSize;
                  // are we HALT?
                  if(Serial.read() == 'H') return;
                  // load pose
                  for(i=0; i<bioloid.poseSize; i++){
                    bioloid.setNextPose(i+1,g_poses[wPoseIndex+i]);
                  } 
                  // interpolate
                  bioloid.interpolateSetup(g_sequence[seqPos].time);
                  while(bioloid.interpolating)
                    bioloid.interpolateStep();
                  // next transition
                  seqPos++;
                }
              }
              else if(ins == ARB_LOOP_SEQ){
                while(1){
                  seqPos = 0;
                  while(g_sequence[seqPos].pose != 0xff){
                    int i;
                    int p = g_sequence[seqPos].pose;
                    wPoseIndex = p * g_bPoseSize;
                    // are we HALT?
                    if(Serial.read() == 'H') return;
                    // load pose
                    for(i=0; i<bioloid.poseSize; i++){
                      bioloid.setNextPose(i+1,g_poses[wPoseIndex+i]);
                    } 
                    // interpolate
                    bioloid.interpolateSetup(g_sequence[seqPos].time);
                    while(bioloid.interpolating)
                      bioloid.interpolateStep();
                    // next transition
                    seqPos++;
                  }
                }
              }
              else if(ins == ARB_SAVE_EEPROM_SEQ){
                // g_bParams[0] = which location to save to...
                PyPoseSaveToEEPROM(g_bParams[0]);
              }
              else if(ins == ARB_TEST){
                int i;
                // Test Digital I/O
                for(i=0;i<8;i++){
                  // test digital
                  pinMode(i,OUTPUT);
                  digitalWrite(i,HIGH);  
                  // test analog
                  pinMode(31-i,OUTPUT);
                  digitalWrite(31-i,HIGH);

                  delay(500);
                  digitalWrite(i,LOW);
                  digitalWrite(31-i,LOW);
                }
                // Test Ax-12
                for(i=452;i<552;i+=20){
                  SetPosition(1,i);
                  delay(200);
                }
                delay(1500);
                // Test Analog I/O
                for(i=0;i<8;i++){
                  // test digital
                  pinMode(i,OUTPUT);
                  digitalWrite(i,HIGH);  
                  // test analog
                  pinMode(31-i,OUTPUT);
                  digitalWrite(31-i,HIGH);

                  delay(500);
                  digitalWrite(i,LOW);
                  digitalWrite(31-i,LOW);
                }
              }   
            }
            else{
              int i;
              // pass thru
              if(ins == AX_READ_DATA){
                ax12GetRegister(id, g_bParams[0], g_bParams[1]);
                // return a packet: FF FF id Len Err params check
                if(ax_rx_buffer[3] > 0){
                  for(i=0;i<ax_rx_buffer[3]+4;i++)
                    Serial.write(ax_rx_buffer[i]);
                }
                ax_rx_buffer[3] = 0;
              }
              else if(ins == AX_WRITE_DATA){
                if(length == 4){
                  ax12SetRegister(id, g_bParams[0], g_bParams[1]);
                }
                else{
                  int x = g_bParams[1] + (g_bParams[2]<<8);
                  ax12SetRegister2(id, g_bParams[0], x);
                }
                // return a packet: FF FF id Len Err params check
                Serial.write((byte)0xff);
                Serial.write((byte)0xff);
                Serial.write((byte)id);
                Serial.write((byte)2);
                Serial.write((byte)0);
                Serial.write((byte)(255-((2+id)%256)));
              }
            }
          }
        }
      }
    }

    // update joints
    bioloid.interpolateStep();
  }
}



void EEPROMWriteData(word wStart, uint8_t *pv, byte cnt) {
  while (cnt--) {
    EEPROM.write(wStart++, *pv++);
  }
}

// BUGBUG:: keeping it simple to start.  Will clear any sequence number > than the one we are storing...
boolean PyPoseSaveToEEPROM(byte bSeqNum) {
  //first verify it is in range...
  byte iSeq;
  word w;
  word wHeaderStart;
  EEPromPoseHeader eepph; 
  EEPROMPoseSeq eepps;
  boolean fValidHeaders;

  if (bSeqNum >= GPSEQ_EEPROM_MAX_SEQ)
    return false;

  // Lets first see where the data should start and do some validation... 
  wHeaderStart = GPSEQ_EEPROM_START_DATA;    // here is where I expect the next one to be...
  fValidHeaders = true;
  for (iSeq=0; iSeq < bSeqNum; iSeq++) {
    EEPROMReadData(GPSEQ_EEPROM_START+iSeq*sizeof(word), (uint8_t*)&w, sizeof(w));
    if (w != wHeaderStart) {
      fValidHeaders = false;
      break;
    }
    EEPROMReadData(w, (uint8_t*)&eepph, sizeof(eepph));
    if ((eepph.bSeqNum != iSeq) || (eepph.bCntServos != NUMSERVOS)) {
      fValidHeaders = false;
      break;
    }
    w = wHeaderStart + sizeof(eepph) + (eepph.bCntSteps * sizeof(EEPROMPoseSeq)) + (eepph.bCntPoses * sizeof(word) * NUMSERVOS);
    if (w >= GPSEQ_EEPROM_SIZE) { 
      fValidHeaders = false;
      break;
    }
    wHeaderStart = w;  // save away the last generated one...
  }    

  // Note: if previous data invalid will try to store data after last valid one...
  eepph.bSeqNum = iSeq;    // save the sequence number here.
  eepph.bCntServos = NUMSERVOS;  // say that it has NUMSERVOS steps.
  eepph.bCntPoses = 0;
  for (eepph.bCntSteps = 0; g_sequence[eepph.bCntSteps].pose != 0xff; eepph.bCntSteps++) {
    if (g_sequence[eepph.bCntSteps].pose > eepph.bCntPoses)
      eepph.bCntPoses = g_sequence[eepph.bCntSteps].pose;
  }
  eepph.bCntPoses++;  // Cnt not index...

  // Make sure it will fit!  
  if ((wHeaderStart + sizeof(eepph) + (eepph.bCntSteps * sizeof(EEPROMPoseSeq)) + (eepph.bCntPoses * sizeof(word) * NUMSERVOS)) >= GPSEQ_EEPROM_SIZE)
    return false;

  // Now lets start writing out the data...
  EEPROMWriteData(GPSEQ_EEPROM_START+iSeq*sizeof(word), (uint8_t*)&wHeaderStart, sizeof(wHeaderStart));  

  // Clear out any other headers...
  w = 0;
  while (++iSeq < GPSEQ_EEPROM_MAX_SEQ )
    EEPROMWriteData(GPSEQ_EEPROM_START+iSeq*sizeof(word), (uint8_t*)&w, sizeof(w));  


  // Now write out the sequence header
  EEPROMWriteData(wHeaderStart, (uint8_t*)&eepph, sizeof(eepph));
  wHeaderStart += sizeof(eepph);  // next location to write to.

  // Now lets write out sequence data
  for (eepph.bCntSteps = 0; g_sequence[eepph.bCntSteps].pose != 0xff; eepph.bCntSteps++) {
    eepps.bPoseNum = g_sequence[eepph.bCntSteps].pose;        // which pose to use
    eepps.wTime = g_sequence[eepph.bCntSteps].time;          // Time to do pose
    EEPROMWriteData(wHeaderStart, (uint8_t*)&eepps, sizeof(eepps));
    wHeaderStart += sizeof(eepps);
  }

  // Last lets write out the Pose data...
  EEPROMWriteData(wHeaderStart, (uint8_t*)g_poses, eepph.bCntPoses * NUMSERVOS * sizeof(g_poses[0]));

  return true;

}



#endif  //DOPypose

