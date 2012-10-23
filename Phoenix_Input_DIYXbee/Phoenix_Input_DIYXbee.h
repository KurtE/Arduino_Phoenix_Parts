//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//  The control input subroutine for the phoenix software is placed in this file.
//  Can be used with V2.0 and above
//Configuration version: V1.0
//Hardware setup: DIY XBee
//
//NEW IN V1.0
// - First Release
//
//
//DIY CONTROLS:
// - Left Stick (WALKMODE) Body Height / Rotate
//     (Translate Mode) Body Height / Rotate body Y 
//     (Rotate Mode) Body Height / Rotate body Y
//     (Single leg Mode) Move Tars Y 
//
// - Right Stick (WALKMODE) Walk/Strafe
//      (Translate Mode) Translate body X/Z
//     (Rotate Mode) Rotate body X/Z
//     (Single leg Mode) Move Tars X/Z
//
//  - Left Slider Speed
// - Right Slider Leg Lift Height
//
// - A    Walk Mode
// - B    Translate Mode
// - C    Rotate Mode
// - D    Single Leg Mode
// - E    Balance Mode on/off
//
// - 0    Turn on/off the bot
//
// - 1-8   (Walk mode) Switch gaits
// - 1-6   (Single leg mode) Switch legs
//
//====================================================================
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

#include "diyxbee.h"

//=============================================================================
// Constants
//=============================================================================
//[CONSTANTS]
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4

#define cTravelDeadZone 4  //The deadzone for the analog input from the remote

//=============================================================================
// Global - Local to this file only...
//=============================================================================

DIYPACKET  g_diyp;
DIYPACKET  g_diypPrev;

#ifdef DISP_VOLTAGE
unsigned long ulTimeLastReported = 0;
#endif
extern "C" {
  // Move the Gait Names to program space...
  const char s_sGN1[] PROGMEM = "Ripple 12";
  const char s_sGN2[] PROGMEM = "Tripod 8";
  const char s_sGN3[] PROGMEM = "Tripple 12";
  const char s_sGN4[] PROGMEM = "Tripple 16";
  const char s_sGN5[] PROGMEM = "Wave 24";
  const char s_sGN6[] PROGMEM = "Tripod 6";
  PGM_P s_asGateNames[] PROGMEM = {
    s_sGN1, s_sGN2, s_sGN3, s_sGN4, s_sGN5, s_sGN6        };

  const char s_sLJUDN1[] PROGMEM = "LJOYUD walk";
  const char s_sLJUDN2[] PROGMEM = "LJOYUD trans";
  const char s_sLJUDN3[] PROGMEM = "SetRotOffset";
  PGM_P s_asLJoyUDNames[] PROGMEM = {
    s_sLJUDN1, s_sLJUDN2, s_sLJUDN3        };
  //static const char  *s_asLJoyUDNames[] = {"LJOYUD walk", "LJOYUD trans", "SetRotOffset"};
}
static unsigned        g_BodyYOffset; 
static char            g_BodyYSift; 
static byte            bXBeeControlMode;

byte                  LjoyUDFunction = 0;		// For setting different options/functions in RotateMode
boolean               LockFunction = false;		// If True the actual function are freezed
boolean               LeftJoyLRmode = false;      
boolean               g_fDisplayLiPo;            // Should we display the lipo information?

static short          g_sGPSMController;
static byte           g_bGPCurStepPrev;
#ifdef USEPS2
byte        g_bWhichControl;  // Which input device are we currently using?
#define     WC_UNKNOWN  0      // Not sure yet
#define     WC_PS2   1      // Using PS2 to control robot
#define     WC_XBEE   2      // we are currently using the XBee to control the robot
#endif



#ifdef DEBUG
extern boolean g_fDebugOutput;
#endif

#ifdef USEMULTI
//==============================================================================
//
// Lets define our Sub-class of the InputControllerClass
//
//==============================================================================
class DIYXBeeController : 
public InputController
{
public:
  DIYXBeeController();        // A reall simple constructor...

  virtual void     Init(void);
  virtual void     ControlInput(void);
  virtual void     AllowControllerInterrupts(boolean fAllow);

};


DIYXBeeController g_diyController;


//==============================================================================
// Constructor. See if there is a simple way to have one or more Input
//     controllers. Maybe register at construction time
//==============================================================================
DIYXBeeController::DIYXBeeController()
{
  RegisterInputController(this);
}

#else
#define DIYXBeeController InputController
// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller 

#endif


//==============================================================================
// This is The function that is called by the Main program to initialize
//   the input controller, which in this case is the PS2 controller
//   process any commands.
//==============================================================================
void DIYXBeeController::Init(void)
{

  // Lets try to initialize the XBEE to use
  InitXBee();


#ifdef __AVR__
#if not defined(UBRR1H)
  XBeeSerial.listen();
#endif    
#endif     
  // Clear any stuff left in the buffer
  delay(20);
  ClearXBeeInputBuffer();

#if 0 //def DBGSerial    
  word wMy = GetXBeeMY();
  DBGSerial.print("XBee My: ");
  DBGSerial.println(wMy, HEX);
#endif    
  g_BodyYOffset = 0;
  g_BodyYSift = 0;
  bXBeeControlMode = WALKMODE;

  g_fDisplayLiPo = true;    // assume we display the lipo state
}


//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void DIYXBeeController::AllowControllerInterrupts(boolean fAllow)
{
#ifdef __AVR__
#if not defined(UBRR1H)
  XBeeSerial.listen();
#endif    
#endif    
}



//==============================================================================
// This is code the checks for and processes input from the DIY XBee receiver
//   work
//==============================================================================
void DIYXBeeController::ControlInput(void)
{
  byte iNumButton;

  // Then try to receive a packet of information from the XBEE.
  // It will return true if it has a valid packet
  if (ReceiveXBeePacket(&g_diyp)) {

    if (memcmp((void*)&g_diyp, (void*)&g_diypPrev, sizeof(g_diyp)) != 0) {
#ifdef XBEE_NEW_DATA_ONLY            
      if (g_diystate.fPacketForced)
        SendXbeeNewDataOnlyPacket(1);
#endif                
#ifdef DEBUG
      // setup to output back to our USB port
      if (g_fDebugOutput)  {
        DBGPrintf("%x - %d %d - %d %d - %d %d\n", g_diyp.s.wButtons, g_diyp.s.bRJoyLR, g_diyp.s.bRJoyUD,
        g_diyp.s.bLJoyLR, g_diyp.s.bLJoyUD, g_diyp.s.bRSlider, g_diyp.s.bLSlider);
      }
#endif
    }

    // OK lets try "0" button for Start. 
    if ((g_diyp.s.wButtons & (1<<0)) && ((g_diypPrev.s.wButtons & (1<<0)) == 0)) { //Start Button (0 on keypad) test
      if(g_InControlState.fHexOn)  {
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
        g_BodyYSift = 0;
        g_InControlState.SelectedLeg = 255;

        g_InControlState.fHexOn = 0;
      } 
      else  {
        //Turn on
        g_InControlState.fHexOn = 1;
      }
    } 

    if (g_InControlState.fHexOn) {
      if ((g_diyp.s.wButtons & (1<<0xa)) && ((g_diypPrev.s.wButtons & (1<<0xa)) == 0)) { // A button test 
        MSound(1, 50, 2000);
        XBeePlaySounds(1, 50, 2000);
        bXBeeControlMode = WALKMODE;
        XBeeOutputStringF(F("Walking"));
      }

      if ((g_diyp.s.wButtons & (1<<0xb)) && ((g_diypPrev.s.wButtons & (1<<0xb)) == 0)) { // B button test
        MSound(1, 50, 2000);
        XBeePlaySounds(1, 50, 2000);
        bXBeeControlMode = TRANSLATEMODE;
        XBeeOutputStringF(F("Body Translate"));
      }

      if ((g_diyp.s.wButtons & (1<<0xc)) && ((g_diypPrev.s.wButtons & (1<<0xc)) == 0)) { // C button test
        MSound(1, 50, 2000);
        bXBeeControlMode = ROTATEMODE;
        XBeeOutputStringF(F("Body Rotate"));
      }

      if ((g_diyp.s.wButtons & (1<<0xD)) && ((g_diypPrev.s.wButtons & (1<<0xd)) == 0)) { // D button test - Single Leg
        MSound(1, 50, 2000);
        if (g_InControlState.SelectedLeg==255) // none
          g_InControlState.SelectedLeg=cRF;
        else if (bXBeeControlMode==SINGLELEGMODE) //Double press to turn all legs down
          g_InControlState.SelectedLeg=255;  //none
        bXBeeControlMode=SINGLELEGMODE;        
        XBeeOutputStringF (F("Single Leg"));
      }
      if ((g_diyp.s.wButtons & (1<<0xe)) && ((g_diypPrev.s.wButtons & (1<<0xe)) == 0)) { // E button test - Balance mode
        if (!g_InControlState.BalanceMode) {
          g_InControlState.BalanceMode = 1;
          MSound( 2, 100, 2000, 50, 4000);
          XBeePlaySounds(2, 100, 2000, 50, 4000);
          XBeeOutputStringF(F("Balance On"));
        } 
        else {
          g_InControlState.BalanceMode = 0;
          MSound( 1, 250, 1500);
          XBeePlaySounds(1, 50, 1500);
          XBeeOutputStringF(F("Balance Off"));
        }
      }

#ifdef OPT_GPPLAYER
      if ((g_diyp.s.wButtons & (1<<0xf)) && ((g_diypPrev.s.wButtons & (1<<0xf)) == 0)) { // F button test - GP Player
        if (g_ServoDriver.FIsGPEnabled()) {   //F Button GP Player Mode Mode on/off -- SSC supports this mode
          XBeeOutputStringF(F("Run Sequence"));
          MSound(1, 50, 2000);

          g_InControlState.BodyPos.x = 0;
          g_InControlState.BodyPos.z = 0;
          g_InControlState.BodyRot1.x = 0;
          g_InControlState.BodyRot1.y = 0;
          g_InControlState.BodyRot1.z = 0;
          g_InControlState.TravelLength.x = 0;
          g_InControlState.TravelLength.z = 0;
          g_InControlState.TravelLength.y = 0;

          g_InControlState.SelectedLeg=255;  //none
          g_InControlState.fSLHold=0;

          bXBeeControlMode = GPPLAYERMODE;
        } 
        else {
          XBeeOutputStringF(F("Seq Disabled"));
          MSound(1, 50, 2000);
        }
      }
#endif   
      //Hack there are several places that use the 1-N buttons to select a number as an index
      // so lets convert our bitmap of which key may be pressed to a number...
      // BUGBUG:: There is probably a cleaner way to convert... Will extract buttons 1-9
      iNumButton = 0;  // assume no button
      if ((g_diyp.s.wButtons & 0x3fe) && ((g_diypPrev.s.wButtons & 0x3fe) == 0)) { // buttons 1-9
        word w = g_diyp.s.wButtons & 0x3fe;

        while ((w & 0x1) == 0)     {
          w >>= 1;
          iNumButton++;
        } 
      }

      // BUGBUG:: we are using all keys now, may want to reserve some...  
      //Switch gait
      // We will do slightly different here than the RC version as we have a bit per button
      if ((bXBeeControlMode==WALKMODE) && iNumButton  && (iNumButton <= NUM_GAITS)) { //1-8 Button Gait select   
        if ( abs(g_InControlState.TravelLength.x)<cTravelDeadZone &&  abs(g_InControlState.TravelLength.z)<cTravelDeadZone &&  
          abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone)  {
          //Switch Gait type
          MSound( 1, 50, 2000);   //Sound P9, [50\4000]
          g_InControlState.GaitType = iNumButton-1;
#ifdef DEBUG
          DBGPrintf("New Gate: %d\n\r", g_InControlState.GaitType);
#endif
          GaitSelect();
#ifdef DEBUG
          DBGPrintf("Output Gate Named\n\r");
#endif
          XBeeOutputStringF((const __FlashStringHelper *)pgm_read_word(&s_asGateNames[g_InControlState.GaitType]));
        }
      }

      //Switch single leg
      if (bXBeeControlMode==SINGLELEGMODE) {
        if (iNumButton>=1 && iNumButton<=6) {
          MSound( 1, 50, 2000);   //Sound P9, [50\4000]
          g_InControlState.SelectedLeg = iNumButton-1;
          g_InControlState.fSLHold=0;
        }

        if (iNumButton == 9) {  //Switch Directcontrol
          MSound( 1, 50, 2000);   //Sound P9, [50\4000]
          g_InControlState.fSLHold ^= 1;    //Toggle g_InControlState.fSLHold
        }

      } 
      else if (bXBeeControlMode==WALKMODE)  {
        g_InControlState.SelectedLeg=255; // none
        g_InControlState.fSLHold=0;
      }

      //Body Height - Control depends on how big our packet was (ie which controller we are using) 
      if (g_diystate.cbPacketSize > PKT_MSLIDER)
        g_InControlState.BodyPos.y = SmoothControl((g_diyp.s.bMSlider / 2), g_InControlState.BodyPos.y, SmDiv);
      else if (g_diystate.cbPacketSize > PKT_LPOT)
        g_InControlState.BodyPos.y =  SmoothControl((g_diyp.s.bLPot*2/3), g_InControlState.BodyPos.y, SmDiv);//Zenta test

      else
        g_InControlState.BodyPos.y =  SmoothControl((g_diyp.s.bLJoyUD / 2), g_InControlState.BodyPos.y, SmDiv);


      //Leg lift height - Right slider has value 0-255 translate to 30-93
      g_InControlState.LegLiftHeight = 30 + g_diyp.s.bRSlider/3;//Zenta trying 3 instead of 4

      //---------------------------------------------------------------------------------------------------
      //Walk mode   
      //---------------------------------------------------------------------------------------------------
      if (bXBeeControlMode==WALKMODE) {  // Kurt's Arduino version
        if (g_diystate.cbPacketSize > PKT_MSLIDER) {
          g_InControlState.TravelLength.x = -(g_diyp.s.bLJoyLR - 128);
          g_InControlState.TravelLength.z = -(g_diyp.s.bLJoyUD - 128) ;
          g_InControlState.TravelLength.y = -(g_diyp.s.bRJoyLR - 128)/3;
          //g_InControlState.BodyRot1.x = SmoothControl(((bPacket(PKT_RJOYUD)-128)*2), g_InControlState.BodyRot1.x, 2);
          //g_InControlState.BodyRot1.z = SmoothControl((-(bPacket(PKT_RPOT)-128)*2), g_InControlState.BodyRot1.z, 2);
          g_InControlState.InputTimeDelay = 128 -  max( max( abs(g_diyp.s.bLJoyLR-128),  abs(g_diyp.s.bLJoyUD-128)),  abs(g_diyp.s.bRJoyLR-128)) + (128 -(g_diyp.s.bLSlider)/2);
        }  
        else if (g_diystate.cbPacketSize > PKT_LPOT) {  // Case for original DIY XBee with extra pots
          g_InControlState.TravelLength.x = -(g_diyp.s.bLJoyLR - 128);
          g_InControlState.TravelLength.z = -(g_diyp.s.bLJoyUD - 128) ;
          g_InControlState.TravelLength.y = -(g_diyp.s.bRJoyLR - 128)/3;
          g_InControlState.BodyRot1.z = SmoothControl((-(g_diyp.s.bRPot-128)*2), g_InControlState.BodyRot1.z, SmDiv);
          g_InControlState.InputTimeDelay = 128 -  max( max( abs(g_diyp.s.bLJoyLR-128),  abs(g_diyp.s.bLJoyUD-128)),  abs(g_diyp.s.bRJoyLR-128)) + (128 -(g_diyp.s.bLSlider)/2);
        }
        else { // original DIY XBee
          g_InControlState.TravelLength.x = -(g_diyp.s.bRJoyLR - 128);
          g_InControlState.TravelLength.z = -(g_diyp.s.bRJoyUD - 128) ;
          g_InControlState.TravelLength.y = -(g_diyp.s.bLJoyLR - 128)/3;
          g_InControlState.InputTimeDelay = 128 -  max( max( abs(g_diyp.s.bRJoyLR-128),  abs(g_diyp.s.bRJoyUD-128)),  abs(g_diyp.s.bLJoyLR-128)) + (128 -(g_diyp.s.bLSlider)/2);
        }
      }
      //---------------------------------------------------------------------------------------------------
      //Body move 
      //---------------------------------------------------------------------------------------------------
      if (bXBeeControlMode==TRANSLATEMODE)  {
        if (g_diystate.cbPacketSize > PKT_LPOT) {
          g_InControlState.BodyPos.x =  SmoothControl(((g_diyp.s.bRJoyLR-128)*2/3), g_InControlState.BodyPos.x, SmDiv);
          g_InControlState.BodyPos.z =  SmoothControl(((g_diyp.s.bRJoyUD-128)*2/3), g_InControlState.BodyPos.z, SmDiv);
          g_InControlState.BodyRot1.y = SmoothControl(((g_diyp.s.bLJoyLR-128)*2), g_InControlState.BodyRot1.y, SmDiv);
        }
        else {
          g_InControlState.BodyPos.x =  SmoothControl(((g_diyp.s.bRJoyLR-128)*2/3), g_InControlState.BodyPos.x, SmDiv);
          g_InControlState.BodyPos.z =  SmoothControl(((g_diyp.s.bRJoyUD-128)*2/3), g_InControlState.BodyPos.z, SmDiv);
          g_InControlState.BodyRot1.y = SmoothControl(((g_diyp.s.bLJoyLR-128)*2), g_InControlState.BodyRot1.y, SmDiv);

        }
        g_InControlState.InputTimeDelay = 128 - abs(g_diyp.s.bLJoyUD-128) + (128 -(g_diyp.s.bLSlider)/2);
      }  

      //---------------------------------------------------------------------------------------------------
      //Body rotate 
      //---------------------------------------------------------------------------------------------------
      if (bXBeeControlMode==ROTATEMODE) {
        if (iNumButton &&(iNumButton <=3)) {
          LjoyUDFunction = iNumButton -1;
          MSound( 1, 20, 2000); 
          XBeeOutputStringF((const __FlashStringHelper *)pgm_read_word(&s_asLJoyUDNames[LjoyUDFunction]));
        }
        if (iNumButton == 4) {  // Toogle Left Joystick left/Right function
          LeftJoyLRmode = !LeftJoyLRmode;
          if (LeftJoyLRmode) {
            XBeeOutputStringF(F("LJOYLR trans"));
            MSound( 1, 20, 1000); 
          } 
          else {
            XBeeOutputStringF(F("LJOYLR rotate"));
            MSound( 1, 20, 2000); 
          }
        }
#ifdef DISP_VOLTAGE
        if (iNumButton == 8) {  // Toogle g_fDisplayLiPo
          g_fDisplayLiPo = !g_fDisplayLiPo;
          MSound( 1, 20, 1500+500*g_fDisplayLiPo); 
        }
#endif 
        if (iNumButton == 9) {  // Toogle LockFunction
          LockFunction = !LockFunction;
          if (LockFunction) {
            XBeeOutputStringF(F("Lock ON"));
            MSound( 1, 20, 1500); 
          } 
          else {
            XBeeOutputStringF(F("Lock OFF"));
            MSound( 1, 20, 2500); 
          }
        }


        // BranchLJoyUDFunction in basic
        switch (LjoyUDFunction) {
        case 0:
          g_InControlState.TravelLength.z = -(g_diyp.s.bLJoyUD-128);  // dito need to update
          break;
        case 1:
          g_InControlState.BodyPos.z = SmoothControl(((g_diyp.s.bLJoyUD-128)*2/3), g_InControlState.BodyPos.z, SmDiv);
          break;
        default:
          if (!LockFunction) {
            g_InControlState.BodyRotOffset.z = (g_diyp.s.bLJoyUD - 128);
            g_InControlState.BodyRotOffset.y = (g_diyp.s.bRPot - 128);
          }
        }

        if (LeftJoyLRmode) {//;Do X translation:
          g_InControlState.BodyPos.x =  SmoothControl(((g_diyp.s.bLJoyLR-128)*2/3), g_InControlState.BodyPos.x, SmDiv);
        } 
        else {
          g_InControlState.BodyRot1.y = SmoothControl (((g_diyp.s.bLJoyLR-128)*2), g_InControlState.BodyRot1.y, SmDiv); 
        }
        g_InControlState.BodyRot1.x = SmoothControl(((g_diyp.s.bRJoyUD-128)*2), g_InControlState.BodyRot1.x, SmDiv);
        g_InControlState.BodyRot1.z  = SmoothControl((-(g_diyp.s.bRJoyLR-128)*2), g_InControlState.BodyRot1.z , SmDiv);

        g_InControlState.InputTimeDelay = 128 - abs(g_diyp.s.bLJoyUD-128) + (128 -(g_diyp.s.bLSlider)/2);

      }

      //---------------------------------------------------------------------------------------------------
      //Single Leg Mode
      //---------------------------------------------------------------------------------------------------
      if (bXBeeControlMode == SINGLELEGMODE)  {
        g_InControlState.SLLeg.x = SmoothControl(((g_diyp.s.bRJoyLR-128)), g_InControlState.SLLeg.x , SmDiv);//;
        g_InControlState.SLLeg.z = SmoothControl((-(g_diyp.s.bRJoyUD-128)), g_InControlState.SLLeg.z , SmDiv);//;
        g_InControlState.SLLeg.y = SmoothControl((-(g_diyp.s.bLJoyUD-128)), g_InControlState.SLLeg.y , SmDiv);//Need to check packetsize..
        g_InControlState.InputTimeDelay = 128 -  max( max( abs(g_diyp.s.bRJoyLR-128),  abs(g_diyp.s.bRJoyUD-128)),  abs(g_diyp.s.bLJoyLR-128)) + (128 -(g_diyp.s.bLSlider)/2);
      }

      //---------------------------------------------------------------------------------------------------
      // Sequence General Player Mode
      //---------------------------------------------------------------------------------------------------
#ifdef OPT_GPPLAYER
      if (bXBeeControlMode == GPPLAYERMODE) { 
        // Lets try some speed control... Map all values if we have mapped some before
        // or start mapping if we exceed some minimum delta from center
        // Have to keep reminding myself that commander library already subtracted 128...
        if (g_ServoDriver.FIsGPSeqActive() ) {
          if ((g_sGPSMController != 32767)  
            || (g_diyp.s.bRJoyUD > (128+16)) || (g_diyp.s.bRJoyUD < (128-16)))
          {
            // We are in speed modify mode...
            short sNewGPSM = map(g_diyp.s.bRJoyUD, 0, 255, -200, 200);
            if (sNewGPSM != g_sGPSMController) {
              g_sGPSMController = sNewGPSM;
              g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
            }
          }
          
          // See what step we are on, if it changed then output the step to the user
          byte bCurStep = g_ServoDriver.GPCurStep();
          if (bCurStep != g_bGPCurStepPrev) {
            g_bGPCurStepPrev = bCurStep;
            
            // Lets build a quick and dirty string to output
            char szTemp[20];
            sprintf(szTemp, "cs: %d SM: %d", bCurStep, (g_sGPSMController == 32767) ? 100 : g_sGPSMController);
            XBeeOutputString(szTemp);
          }
          
        }

        if (iNumButton>=1 && iNumButton<=9) { //1-9 Button Play GP Seq
          word wGPSeqPtr;
          if (!g_ServoDriver.FIsGPSeqActive() ) {
            uint8_t GPSeq = iNumButton-1;

            if ( g_ServoDriver.FIsGPSeqDefined(GPSeq)) {
              XBeeOutputStringF(F("Start Sequence"));  // Tell user sequence started.
              g_ServoDriver.GPStartSeq(GPSeq);
              g_sGPSMController = 32767;    // Say we are not in modifiy speed modifier mode yet...
              g_bGPCurStepPrev = 0xff;
            }  
            else {
              XBeeOutputStringF(F("Seq Not defined"));  // that sequence was not defined...
            }
          }
          else {
            // Cancel the current one
            g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
            MSound (2, 50, 2000, 50, 2000);
          }
        }
      }
#endif            

      //Calculate walking time delay
    }
    g_diypPrev = g_diyp; // remember the last packet
  }  
  else  {
    // Not a valid packet - we should go to a turned off state as to not walk into things!
    if (g_InControlState.fHexOn && (g_diystate.fPacketForced ))  {
      // Turn off
      //   MSound(4, 100,2500, 80, 2250, 100, 2500, 60, 20000); // play it a little different...

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
      g_BodyYSift = 0;
      g_InControlState.SelectedLeg = 255;
      g_InControlState.fHexOn = 0;
    }
  }

#ifdef DISP_VOLTAGE
  // Check every so often for voltage and if changed report back to user...
  if (g_fDisplayLiPo && ((millis() - ulTimeLastReported) > DISP_VOLTAGE_TIME)) {
    word wVoltage = g_ServoDriver.GetBatteryVoltage();
    XBeeOutputVal(wVoltage);
    ulTimeLastReported = millis();
  }

#endif
#ifdef DEBUG_ENTERLEAVE
  DBGSerout ("Exit: Control Input\n\r");
#endif 
}


#include "diyxbee_code.h"




