//=============================================================================
// DIYXBee.h XBee Support for the DIY Remote control
// [Packets sent from Remote to Robot]
//=============================================================================
#ifndef _DIYXBEE_H_
#define _DIYXBEE_H_

#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

#define XBEEDATAVERSION      1                                         // Data version...

#ifndef CXBEEPACKETTIMEOUTMS
#define CXBEEPACKETTIMEOUTMS 250					// how long to wait for packet after we send request
#endif

#ifndef CXBEEFORCEREQMS	
#define CXBEEFORCEREQMS		1000					// if nothing in 1 second force a request...
#endif

#ifndef CXBEETIMEOUTRECVMS
#define CXBEETIMEOUTRECVMS	2000					// 2 seconds if we receive nothing
#endif


#define XBEE_TRANS_READY			0x01	// Transmitter is ready for requests.
// Optional Word to use in ATDL command
#define XBEE_TRANS_NOTREADY		        0x02	// Transmitter is exiting transmitting on the sent DL
// No Extra bytes.
#define XBEE_TRANS_DATA				0x03	// Data Packet from Transmitter to Robot*
// Packet data described below.  Will only be sent when the robot sends
// the remote a XBEE_RECV_REQ_DATA packet and we must return sequence number
#define XBEE_TRANS_NEW				0x04	// New Data Available
// No extra data.  Will only be sent when we are in NEW only mode
#define XBEE_ENTER_SSC_MODE			0x05	// The controller is letting robot know to enter SSC echo mode
// while in this mode the robot will try to be a pass through to the robot. This code assumes
// cSSC_IN/cSSC_OUT.  The robot should probalby send a XBEE_SSC_MODE_EXITED when it decides
// to leave this mode...	
// When packet is received, fPacketEnterSSCMode will be set to TRUE.  Handlers should probalby
// get robot to a good state and then call XBeeHandleSSCMode, which will return when some exit
// condition is reached.  Start of with $$<CR> command as to signal exit
#define XBEE_REQ_SN_NI				0x06	// Request the serial number and NI string

#define XBEE_TRANS_CHANGED_DATA                 0x07    // We transmite a bit mask with which fields changed plus the bytes that changes

#define XBEE_TRANS_NOTHIN_CHANGED               0x08    // 
#define XBEE_TRANS_DATA_VERSION                 0x09    //  What format of data this transmitter supports. 
// 1- New format supports changed data packets...
#define XBEE_DEBUG_ATTACH                       0x0A     // Debug Attach - used to say send debug info to display
#define XBEE_DEBUG_DETACH                       0x0B     // End debug output messages...
#define XBEE_DEBUG_STRING                       0x0C     // Debug Text sent from Debug App...

//[Packets sent from Robot to remote]
#define XBEE_RECV_REQ_DATA			0x80	// Request Data Packet*
// Old Format No extra bytes: expect to receive XBEE_TRANS_DATA_PACKET
// New Format 1 extra byte to signal - Will Return 1 of 3 messages...
#define XBEE_RECV_REQ_NEW			0x81	// Request Only New data
// No Extra bytes goes into New only mode and we will typically 
// wait until Remote says it has new data before asking for data.
// In new mode, the remote may choose to choose a threshold of how big a change
// needs to be before sending the XBEE_TRANS_NEW value.
#define XBEE_RECV_REQ_NEW_OFF		        0x82	// We will request data when we want it
#define XBEE_RECV_NEW_THRESH	 	        0x83	// Set new Data thresholds
// currently not implemented
#define XBEE_RECV_DISP_VAL			0x84	// Display a value on line 2
// If <cbExtra> is  0 then we will display the number contained in <SerialNumber> 
// If not zero, then it is a count of bytes in a string to display.
#define XBEE_RECV_DISP_STR			0x85	// Display a string value on line 2
#define XBEE_PLAY_SOUND				0x86	// Will make sounds on the remote...
//	<cbExtra> - 2 bytes per sound: Duration <0-255>, Sound: <Freq/25> to make fit in byte...
#define XBEE_SSC_MODE_EXITED		        0x87	// a message sent back to the controller when
// it has left SSC-mode.
#define XBEE_SEND_SN_NI_DATA		        0x88	// Response for REQ_SN_NI - will return
// 4 bytes - SNH
// 4 bytes - SNL
// up to 20 bytes(probably 14) for NI
#define XBEE_RECV_DISP_VAL0		        0x89	// Display a 2nd value on line 2 - Col 0 on mine
#define XBEE_RECV_DISP_VAL1		        0x8A	// Display a 2nd value on line 2
#define XBEE_RECV_DISP_VAL2		        0x8B	// Display a value on line 2  - Cal 

#define XBEE_RECV_REQ_DATA2                     0x90    // New format... 

//[XBEE_TRANS_DATA] - has 8 extra bytes
//	0 - Buttons High
//	1 - Buttons Low
// 	2 - Right Joystick L/R
//	3 - Right Joystick U/D
//	4 - Left Joystick L/R
//	5 - Left Joystick U/D
// 	6 - Right Slider
//	7 - Left Slider

// OK Lets define some structures...

// Main data packet						
// Main data packet						
typedef struct _diypacketOrig {
  uint16_t	wButtons;					// the 16 buttons
  uint8_t	bRJoyLR;					// right joystick X (LR)
  uint8_t	bRJoyUD;					//				..Y (UD)
  uint8_t	bLJoyLR;					// Left Joystick  X (LR)
  uint8_t	bLJoyUD;					// 				  Y (UD)
  uint8_t	bRSlider;					// Right Slider
  uint8_t	bLSlider;					// Left Slider
} 
DIYPACKETORIG;
// Main data packet						
typedef union {
  uint8_t    ab[12];
  struct  {    
    uint16_t wButtons;					// the 16 buttons
    uint8_t	bRJoyLR;					// right joystick X (LR)
    uint8_t	bRJoyUD;					//				..Y (UD)
    uint8_t	bLJoyLR;					// Left Joystick  X (LR)
    uint8_t	bLJoyUD;					// 				  Y (UD)
    uint8_t	bRSlider;					// Right Slider
    uint8_t	bLSlider;					// Left Slider
    // Added values since first version                    
    uint8_t    bRPot;                                          // top Pot on right joystick
    uint8_t    bLPot;                                          // ... Left joystick
    uint8_t	bMSlider;					// Middle Slider
    uint8_t    bButtons2;                                      // Extra buttons on new DIY...
  } 
  s;
} 
DIYPACKET;

typedef DIYPACKET *PDIYPACKET;


enum {
  PKT_BTNLOW=0,     // Low Buttons 0-7
  PKT_BTNHI,         // High buttons 8-F
  PKT_RJOYLR,        // Right Joystick Up/Down
  PKT_RJOYUD,        // Right joystick left/Right
  PKT_LJOYLR,	// Left joystick Left/Right
  PKT_LJOYUD,	// Left joystick Up/Down
  PKT_RSLIDER,	// right slider
  PKT_LSLIDER,	// Left slider
  PKT_RPOT,          // Right Pot
  PKT_LPOT,          // Left Pot
  PKT_MSLIDER,       // Middle Slider
  PKT_BTNS2          // Extra buttons like on top of joystick
};


typedef unsigned long ulong;

// Now define some static state stuff
typedef struct _diystate
{
  // state information
  boolean 	fTransReadyRecvd;
  boolean 	fPacketValid; 
  boolean 	fPacketTimeOut;
  boolean 	fPacketForced;
  uint8_t         cbPacketSize;    // what was the size of the last data packet received.
  // More internal to function...
  boolean 	fReqDataPacketSent;
  boolean 	fReqDataForced;
  boolean 	fSendOnlyNewMode;
  uint8_t	        bPacketNum;
  uint8_t	        bAPIPacket[33];				// Api packet
  uint16_t	wAPIDL;					// current destination.
  uint16_t        wDBGDL;                                // Debug Destination.
  uint8_t         bTransDataVersion;                      // What version of data 

  // Other information, could make static to file...
  ulong	ulLastPacket;
  ulong 	ulLastRequest;

} 
DIYSTATE;

#ifdef XBEE_DEBUG_OUTPUT
// Merge in the Debug stuff here...
#define XBEE_DEBUG_MAX_OUTPUT 80
#define XBEE_DEBUG_MAX_INPUT 24
class XBeeDebugSerial : 
public Stream
{
private:
  uint8_t _abOut[XBEE_DEBUG_MAX_OUTPUT];
  uint8_t _cbOut;
  uint8_t _abIn[XBEE_DEBUG_MAX_INPUT];
  uint8_t _cbIn;  //  simple only one message processed
  uint8_t _iIn;  // index to next character to return

public:
    // public methods
  XBeeDebugSerial();
  ~XBeeDebugSerial();
  void begin(long speed);

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual int peek();
  virtual void flush();

  using Print::write;

  // Function for our XBee code to call to put the data into the input buffer
  void SetInputText(uint8_t *pb, uint8_t cb);

};

extern XBeeDebugSerial XBDSerial;
#endif //XBEE_DEBUG_OUTPUT



extern DIYSTATE g_diystate;

// Forward references some may be moved to external files later
extern void InitXBee();    // assume hard coded for now to UART2... 
extern void SendXBeePacket(uint16_t wDL, uint8_t bPHType, uint8_t cbExtra, uint8_t *pbExtra);
extern void SendXbeeNewDataOnlyPacket(boolean fNewOnly);
extern void XBeeOutputVal(uint16_t w);
extern boolean ReceiveXBeePacket(PDIYPACKET pdiyp);

#define XBeeOutputString(pString) 	{SendXBeePacket(g_diystate.wAPIDL, XBEE_RECV_DISP_STR, strlen(pString), (uint8_t*)pString);}
extern void XBeeOutputStringF(const __FlashStringHelper *pString);
#define XBEE_MAX_NOTES     5
extern void XBeePlaySounds(uint8_t cNotes, ...);

//extern void XBeeOutputVal(uint8_t bXbeeVal);
//extern void XBeeOutputString(char *pString);
//extern void XBeePlaySounds(uint8_t *pb, uint8_t cb);
extern void APISendXBeeGetCmd(char c1, char c2);  // use this to send other commands as well
extern uint16_t GetXBeeHVal (char c1, char c2);
//extern uint16_t GetXBeeMY();
//extern uint16_t GetXBeeDL();
#define GetXBeeMY()			GetXBeeHVal ('M', 'Y')
#define GetXBeeDL()			GetXBeeHVal ('D', 'L')
extern void SetXBeeHexVal(char c1, char c2, unsigned long _lval);

// These functions are the ones to actually talk to the hardware.  Should try to make sure
// all of the other functions talk through these...
extern uint8_t ReadFromXBee(uint8_t *pb, uint8_t cb, ulong Timeout, uint16_t wEOL);
extern void ClearXBeeInputBuffer(void);
extern boolean XBeeCheckForQueuedBytes(void);
extern void WaitForXBeeTransmitComplete(void);

extern void ClearInputBuffer(void);

#if not defined(UBRR1H)
extern SoftwareSerial XBeeSerial;
#endif


#endif // _DIYXBEE_H_


