/****************************************************************************
 * - DIY remote control XBee support file
 *
 ****************************************************************************/
#ifdef DBGSerial
#define DEBUG
//#define DEBUG_XBEE
//#define DEBUG_OUT
//#define DEBUG_VERBOSE
#endif

#ifdef USEXBEE
#include "diyxbee.h"

#ifdef __AVR__
// Add support for running on non-mega Arduino boards as well.
#if defined(cXBEE_IN)
SoftwareSerial XBeeSoftSerial(cXBEE_IN, cXBEE_OUT);
#else
#if not defined(XBeeSerial)
#define XBeeSerial Serial2
#endif

#endif

#else
// Non-AVR.
#ifndef XBeeSerial
#define XBeeSerial Serial3
#endif
#endif


DIYSTATE g_diystate;
#define XBEE_API_PH_SIZE    1            // Changed Packet Header to just 1 byte - Type - we don't use sequence number anyway...
//#define XBEE_NEW_DATA_ONLY 1
// Some forward definitions
#ifdef DEBUG
extern void DebugMemoryDump(const byte* , int, int);
static boolean s_fDisplayedTimeout = false;
#endif

#ifdef XBEE_DEBUG_OUTPUT
XBeeDebugSerial XBDSerial;
#endif

#ifndef CXBEETIMEOUTRECVMS
#define CXBEETIMEOUTRECVMS 250
#endif
//=============================================================================
// Xbee stuff
//=============================================================================
void InitXBee(void)
{
#ifdef OPT_DEBUGPINS
#ifndef DEBUG_PINS_FIRST
#define DEBUG_PINS_FIRST 2
#endif
  pinMode(DEBUG_PINS_FIRST, OUTPUT);
  pinMode(DEBUG_PINS_FIRST+1, OUTPUT);
  pinMode(DEBUG_PINS_FIRST+2, OUTPUT);
  pinMode(DEBUG_PINS_FIRST+3, OUTPUT);
  pinMode(DEBUG_PINS_FIRST+4, OUTPUT);
#endif

  char abT[10];  
#define XBEE_POWERUP_HACK  
  #ifdef XBEE_POWERUP_HACK  
  DEBUGTOGGLE(DEBUG_PINS_FIRST);
  XBeeSerial.begin(9600);   
  XBeeSerial.println("ATCN");
  XBeeSerial.flush();
  XBeeSerial.end();
  delay(100);
#endif
XBeeSerial.begin(XBEE_BAUD);    // BAUD rate is defined in Hex_CFG.h file...
  // Ok lets set the XBEE into API mode...


  DEBUGTOGGLE(DEBUG_PINS_FIRST);
  delay(1000);
  XBeeSerial.write("+++");
  XBeeSerial.flush();

  // Lets try to get an OK 
  XBeeSerial.setTimeout(100);
  if (XBeeSerial.readBytesUntil(13, abT, sizeof(abT)) < 2) {
    // probably failed, maybe we have not set everything up yet...
    delay(2000);
    ClearXBeeInputBuffer();      
    XBeeSerial.print("+++");
    XBeeSerial.flush();
    XBeeSerial.setTimeout(2000);

    XBeeSerial.readBytesUntil(13, abT, sizeof(abT));
    XBeeSerial.println("ATGT 3");
  }

  DEBUGTOGGLE(DEBUG_PINS_FIRST);
  delay(20);
  XBeeSerial.write("ATAP 1\rATCN\r");
  delay(100);
  ClearXBeeInputBuffer();      

  // for Xbee with no flow control, maybe nothing to do here yet...
  g_diystate.fPacketValid = false; 
  g_diystate.fDataPacketsReceived = false;
  g_diystate.bPacketNum = 0;
  g_diystate.wDBGDL= 0xffff;          // Debug 

    }



//==============================================================================
// [SendXBeePacket] - Simple helper function to send the 4 byte packet header
//     plus the extra data if any
//==============================================================================
void SendXBeePacket(uint16_t wDL, uint8_t bPHType, uint8_t cbExtra, uint8_t *pbExtra)
{
  // Tell system to now output to the xbee
  uint8_t abPH[9];
  uint8_t *pbT;
  uint8_t bChkSum;
  int i;

  // We need to setup the xbee Packet
  abPH[0]=0x7e;                        // Command prefix
  abPH[1]=0;                            // msb of size
  abPH[2]=cbExtra+XBEE_API_PH_SIZE + 5;    // size LSB
  abPH[3]=1;                             // Send to 16 bit address.

  g_diystate.bPacketNum = g_diystate.bPacketNum + 1;
  if (g_diystate.bPacketNum == 0)
    g_diystate.bPacketNum = 1;        // Don't pass 1 as this says no ack
  abPH[4]=g_diystate.bPacketNum;        // frame number
  abPH[5]=wDL >> 8;        // Our current destination MSB/LSB
  abPH[6]=wDL & 0xff;
  abPH[7]=0;                            // No Options

  abPH[8]=bPHType;

  // Now compute the initial part of the checksum
  bChkSum = 0;
  for (i=3;i <= 8; i++)
    bChkSum += abPH[i];

  // loop through the extra bytes in the exta to build the checksum;
  pbT = pbExtra;
  for (i=0; i < cbExtra; i++)
    bChkSum += *pbT++;                // add each byte to the checksum

  // Ok lets output the fixed part
  XBeeSerial.write(abPH,9);

  // Ok lets write the extra bytes if any to the xbee
  if (cbExtra)
    XBeeSerial.write(pbExtra,cbExtra);

  // Last write out the checksum
  bChkSum = 0xff - bChkSum;
  XBeeSerial.write(bChkSum);

#ifdef DEBUG_OUT
  // We moved dump before the serout as hserout will cause lots of interrupts which will screw up our serial output...
  // Moved after as we want the other side to get it as quick as possible...
  if (g_fDebugOutput) {
    DBGSerial.print("SDP: ");
    DBGSerial.print(bPHType, HEX);
    DBGSerial.print(" ");
    DBGSerial.println(cbExtra, HEX);

#ifdef DEBUG_VERBOSE        // Only ouput whole thing if verbose...
    if (cbExtra)
      DebugMemoryDump(pbExtra, 0, cbExtra);
#endif    
    DBGSerial.println();
  }

#endif

}


//==============================================================================
// [XBeeOutputVal] - Simple wrapper function to pass a word value back to
//            remote control to display
//==============================================================================
void XBeeOutputVal(uint16_t w)
{
  uint8_t ab[2];
  ab[0] = w >> 8;  // First byte is MSB
  ab[1] = w & 0xff;  // 2nd byte is LSB
  SendXBeePacket(g_diystate.wAPIDL, XBEE_RECV_DISP_VAL, sizeof(ab), (uint8_t*)ab);
#ifdef DBGSerial
  DBGSerial.print("XBee Display Val: ");
  DBGSerial.print(w, DEC);
  DBGSerial.print(" to: ");
  DBGSerial.println(g_diystate.wAPIDL, HEX);  
#endif  
}

//==============================================================================
// [XBeeOutputStringF] - Output string held in flash memory to remote to display
//            remote control to display
//==============================================================================
void XBeeOutputStringF(const __FlashStringHelper *pString)
{
  // Could replicate SendXBeePacket function to handle program memory.
  // But for first shot, simply copy string to local and then use it...
  char szT[20];  // This should be large enough to hold our maximum string.
  strcpy_P(szT, (const char PROGMEM *)pString);
  SendXBeePacket(g_diystate.wAPIDL, XBEE_RECV_DISP_STR, strlen(szT), (uint8_t*)szT);
}
//==============================================================================
// [XBeePlaySounds] - Simple wrapper to take the inline notes and package them
//            up and semd them...
//            remote control to display
//==============================================================================
void XBeePlaySounds(uint8_t cNotes, ...)
{
  va_list ap;
  uint8_t abNotes[XBEE_MAX_NOTES*2];        // Should not hard code...
  uint8_t cb = 0;

  va_start(ap, cNotes);

  if (cNotes > XBEE_MAX_NOTES)    // don't overrun our buffer...
    cNotes = XBEE_MAX_NOTES;

  while (cNotes > 0) {
    abNotes[cb++] = (uint8_t)va_arg(ap, unsigned int);
    abNotes[cb++] = (uint8_t)(va_arg(ap, unsigned int) / 25);
    cNotes--;
  }
  va_end(ap);
  SendXBeePacket(g_diystate.wAPIDL, XBEE_PLAY_SOUND, cb, abNotes);
}


//////////////////////////////////////////////////////////////////////////////
//==============================================================================
// [APIRecvPacket - try to receive a packet from the XBee. 
//        - Will return The packet length if it receives something, else 0
//        - Pass in buffer to receive packet.  Assumed it is big enough...
//        - pass in timeout if zero will return if no data...
//        
//==============================================================================
byte APIRecvPacket(ulong Timeout)
{
  uint8_t cbRead;
  char abT[3];
  uint8_t bChksum;
  int i;

  short wPacketLen;
  //  First see if the user wants us to wait for input or not
  //    hserstat HSERSTAT_INPUT_EMPTY, _TP_Timeout            // if no input available quickly jump out.
  if (Timeout == 0) 
  {
    if (!XBeeSerial.available())
      return 0;        // nothing waiting for us...
    Timeout = 100;    // .1 second?

  }

  XBeeSerial.setTimeout(Timeout);
  DEBUGTOGGLE(DEBUG_PINS_FIRST+3);

  // Now lets try to read in the data from the xbee...
  // first read in the delimter and packet length
  // We now do this in two steps.  The first to try to resync if the first character
  // is not the proper delimiter...

  do {    
    cbRead = XBeeSerial.readBytes(abT, 1);
    if (cbRead == 0)
      return 0;
  } 
  while (abT[0] != 0x7e);

  cbRead = XBeeSerial.readBytes(abT, 2);
  if (cbRead != 2)
    return 0;                // did not read in full header or the header was not correct.

  wPacketLen = (abT[0] << 8) + abT[1];

  if (wPacketLen >= sizeof(g_diystate.bAPIPacket)) {
#ifdef DBGSerial      
    DBGSerial.print("Packet Length error: ");
    DBGSerial.println(wPacketLen, DEC);
#endif        
    ClearXBeeInputBuffer();  // maybe clear everything else out.
    return 0;  // Packet bigger than expected maybe bytes lost...
  }

  // Now lets try to read in the packet
  cbRead = XBeeSerial.readBytes((char*)g_diystate.bAPIPacket, wPacketLen+1);
  if (cbRead != wPacketLen+1)
    return 0;       // did not read in full packet.

  // Now lets verify the checksum.
  bChksum = 0;
  for (i = 0; i < wPacketLen; i++)
    bChksum = bChksum + g_diystate.bAPIPacket[i];             // Add that byte to the buffer...


  if (g_diystate.bAPIPacket[wPacketLen] != (0xff - bChksum)) {
#ifdef DBGSerial      
    DBGSerial.print("Packet Checksum error: ");
    DBGSerial.print(wPacketLen, DEC);
    DBGSerial.print(" ");
    DBGSerial.print(g_diystate.bAPIPacket[wPacketLen], HEX);
    DBGSerial.print("!=");
    DBGSerial.println(0xff - bChksum, HEX);
#endif        
    return 0;                // checksum was off
  }
  DEBUGTOGGLE(DEBUG_PINS_FIRST+4);
  return wPacketLen;    // return the packet length as the caller may need to know this...
}



//==============================================================================
// [SetXBeeHexVal] - Set one of the XBee Hex value registers.
//==============================================================================

void SetXBeeHexVal(char c1, char c2, unsigned long _lval)
{
  uint8_t abT[12];

  // Build a command buffer to output
  abT[0] = 0x7e;                    // command start
  abT[1] = 0;                        // Msb of packet size
  abT[2] = 8;                        // Packet size
  abT[3] = 8;                        // CMD=8 which is AT command

  g_diystate.bPacketNum = g_diystate.bPacketNum + 1;
  if (g_diystate.bPacketNum == 0)
    g_diystate.bPacketNum = 1;        // Don't pass 1 as this says no ack

  abT[4] = g_diystate.bPacketNum;    // Frame id
  abT[5] = c1;                    // Command name
  abT[6] = c2;

  abT[7] = _lval >> 24;            // Now output the 4 bytes for the new value
  abT[8] = (_lval >> 16) & 0xFF;
  abT[9] = (_lval >> 8) & 0xFF;
  abT[10] = _lval & 0xFF;

  // last but not least output the checksum
  abT[11] = 0xff - 
    ( ( 8+g_diystate.bPacketNum + c1 + c2 + (_lval >> 24) + ((_lval >> 16) & 0xFF) +
    ((_lval >> 8) & 0xFF) + (_lval & 0xFF) ) & 0xff);

  XBeeSerial.write(abT, sizeof(abT));

}


//==============================================================================
// [SetXbeeDL] - Set the XBee DL to the specified word that is passed
//         simple wrapper call to hex val
//==============================================================================
void SetXBeeDL (unsigned short wNewDL)
{
  SetXBeeHexVal('D','L', wNewDL);
  g_diystate.wAPIDL = wNewDL;        // remember what DL we are talking to.
}


//==============================================================================
// [APISendXBeeGetCmd] - Output the command packet to retrieve a hex or string value
//==============================================================================

void APISendXBeeGetCmd(char c1, char c2)
{
  uint8_t abT[8];

  // just output the bytes that we need...
  abT[0] = 0x7e;                    // command start
  abT[1] = 0;                        // Msb of packet size
  abT[2] = 4;                        // Packet size
  abT[3] = 8;                        // CMD=8 which is AT command

  g_diystate.bPacketNum = g_diystate.bPacketNum + 1;
  if (g_diystate.bPacketNum == 0)
    g_diystate.bPacketNum = 1;        // Don't pass 1 as this says no ack

  abT[4] = g_diystate.bPacketNum;    // Frame id
  abT[5] = c1;                    // Command name
  abT[6] = c2;

  // last but not least output the checksum
  abT[7] = 0xff - ((8 + g_diystate.bPacketNum + c1 + c2) & 0xff);
#ifdef DEBUG_OUTPUT
  if (g_fDebugOutput) {
    DBGSerial.print("ASGC -");
    DBGSerial.write(c1);
    DBGSerial.write(c2);
    DBGSerial.println(g_diystate.bPacketNum, HEX);
  }
#endif
  XBeeSerial.write(abT, sizeof(abT));
}



//==============================================================================
// [GetXBeeHVal] - Set the XBee DL or MY or??? Simply pass the two characters
//             that were passed in to the XBEE
//==============================================================================
uint16_t GetXBeeHVal (char c1, char c2)
{
  word wPacketLen;
  uint16_t wRet;
  byte b;
  // Output the request command
  APISendXBeeGetCmd(c1, c2);
  XBeeSerial.flush();

  // Now lets loop reading responses 
  for (;;)
  {

    if ((wPacketLen = APIRecvPacket(10))==0)
      break;
#ifdef DEBUG_VERBOSE                
    if (g_fDebugOutput) {
      DBGSerial.print("GHV - Recv Packet");
      DBGSerial.print(g_diystate.bAPIPacket[0], HEX);
      DBGSerial.print(" ");
      DBGSerial.println(g_diystate.bAPIPacket[1], HEX);
    }
#endif
    // Only process the cmd return that is equal to our packet number we sent and has a valid return state
    if ((g_diystate.bAPIPacket[0] == 0x88) && (g_diystate.bAPIPacket[1] == g_diystate.bPacketNum) &&
      (g_diystate.bAPIPacket[4] == 0))
    {
      wRet = 0;
      for (b=5; b < wPacketLen; b++) {
        wRet = (wRet << 8) + g_diystate.bAPIPacket[b];
      }
      return wRet;
      // BUGBUG: Why am I using the high 2 bytes if I am only processing words?
      //return     (g_diystate.bAPIPacket[5] << 8) + g_diystate.bAPIPacket[6];
    }
  }
  return 0xffff;                // Did not receive the data properly.
}





/////////////////////////////////////////////////////////////////////////////


//==============================================================================
// [ClearXBeeInputBuffer] - This simple helper function will clear out the input
//                        buffer from the XBEE
//==============================================================================
void ClearXBeeInputBuffer(void)
{
  uint8_t b[1];

#ifdef DEBUG
  boolean fBefore = g_fDebugOutput;
  g_fDebugOutput = false;
#endif    
  //    XBeeSerial.flush();    // clear out anything that was queued up...        
  while (XBeeSerial.read() != -1)
    ;    // just loop as long as we receive something...
#ifdef DEBUG
  g_fDebugOutput = fBefore;
#endif    
}



//==============================================================================
// [DebugMemoryDump] - striped down version of rprintfMemoryDump
//==============================================================================
#ifdef DEBUG
void DebugMemoryDump(const uint8_t* data, int off, int len)
{
  int x;
  int c;
  int line;
  const uint8_t * b = data;

  for(line = 0; line < ((len % 16 != 0) ? (len / 16) + 1 : (len / 16)); line++)  {
    int line16 = line * 16;
    DBGSerial.print(line16, HEX);
    DBGSerial.print("|");
    ;

    // put hex values
    for(x = 0; x < 16; x++) {
      if(x + line16 < len) {
        c = b[off + x + line16];
        DBGSerial.print(c, HEX);
        DBGSerial.print(" ");
      }
      else
        DBGSerial.write("   ");
    }
    DBGSerial.write("| ");

    // put ascii values
    for(x = 0; x < 16; x++) {
      if(x + line16 < len) {
        c = b[off + x + line16];
        DBGSerial.write( ((c > 0x1f) && (c < 0x7f))? c : '.');
      }
      else
        DBGSerial.write(" ");
    }
    DBGSerial.println();
  }
}

#endif


//==============================================================================
// [ReceiveXBeePacket] - This function will try to receive a packet of information
//         from the remote control over XBee.
//
// the data in a standard packet is arranged in the following byte order:
//    0 - Buttons High
//    1 - Buttons Low
//     2 - Right Joystick L/R
//    3 - Right Joystick U/D
//    4 - Left Joystick L/R
//    5 - Left Joystick U/D
//     6 - Right Slider
//    7 - Left Slider
//==============================================================================
boolean ReceiveXBeePacket(PDIYPACKET pdiyp)
{
  uint8_t cbRead;
  uint8_t bDataOffset;
  uint16_t wNewDL;
  ulong ulCurrentTime;
  ulong ulTimeDiffMS;
  boolean _fPacketValidPrev = g_diystate.fPacketValid;        // Save away the previous state as this is the state if no new data...
  boolean _fNewPacketAvail = false;

  g_diystate.fPacketValid = false;
  g_diystate.fNewPacket = false;      // Did we receive a new packet?

  //    We will first see if we have a packet header waiting for us.
  //  BUGBUG:: Question should I loop after I process a package and only get out of the
  //             loop when I have no more data, or only process one possible message?
  //            Maybe depends on message?
  DEBUGTOGGLE(DEBUG_PINS_FIRST+1);
  for (;;)
  {
    if (!XBeeSerial.available())
      break;        // no input available, break from this loop

    // The XBEE has sent us some data so try to get a packet header
    cbRead = APIRecvPacket(10);        // Lets read in a complete packet.
    if (!cbRead)
      break;                            // Again nothing read?

#ifdef DEBUG
    s_fDisplayedTimeout = false;        // say that we got something so we can display empty again...
#endif
    if (g_diystate.bAPIPacket[0] == 0x81)
      bDataOffset = 5;                // Received packet with 16 bit addressing
    else if (g_diystate.bAPIPacket[0] == 0x80)
      bDataOffset = 11;                // Received packet with 64 bit addressing
    else if (g_diystate.bAPIPacket[0] == 0x89) {
      if (g_diystate.bAPIPacket[2] != g_diystate.bTxStatusLast) {
        g_diystate.bTxStatusLast = g_diystate.bAPIPacket[2];
#ifdef DEBUG
        DBGSerial.print("CTDB TStat: ");
        DBGSerial.println(g_diystate.bAPIPacket[2], HEX);
#endif
      }
      continue;
    }
    else
      break;                            // Invalid packet lets bail from this loop.

    // Change CB into the number of extra bytes...
    cbRead -= (bDataOffset + 1);        // Ph is only 1 byte long now... 

#ifdef DEBUG_XBEE
    if (g_fDebugOutput) {
        DebugMemoryDump(&g_diystate.bAPIPacket[bDataOffset + 0], 0, cbRead);
    }
#endif

    //-----------------------------------------------------------------------------
    // [XBEE_TRANS_DATA]
    //-----------------------------------------------------------------------------
    if (g_diystate.bAPIPacket[bDataOffset + 0] == XBEE_TRANS_DATA) {
      if (cbRead >= sizeof(DIYPACKETORIG)) {
        pdiyp->s.wButtons = g_diystate.bAPIPacket[bDataOffset + 1] + (g_diystate.bAPIPacket[bDataOffset + 2] << 8);

        // Simple copy the memory down
        g_diystate.cbPacketSize = (cbRead <= sizeof(DIYPACKET))? cbRead : sizeof(DIYPACKET);

        memcpy(&pdiyp->ab[2], &g_diystate.bAPIPacket[bDataOffset + 3], g_diystate.cbPacketSize);

        // process first as higher number of these come in...
        g_diystate.fPacketValid = true;    // data is valid
        g_diystate.fNewPacket = true;      // Did we receive a new packet?
        g_diystate.fDataPacketsReceived = true;
        // BUGBUG:: Assuming 16 bit addressing
        g_diystate.wAPIDL = (uint16_t)(g_diystate.bAPIPacket[1] << 8) + g_diystate.bAPIPacket[2];
        g_diystate.ulLastPacket = millis();
        DEBUGTOGGLE(DEBUG_PINS_FIRST+2);

        return true;    //          // get out quick!
      } 
      else {
#ifdef DEBUG
        DBGSerial.print("XBEE_TRANS_DATA: ");
        DBGSerial.print(sizeof(DIYPACKET), DEC);
        DBGSerial.print(" ");
        DBGSerial.println(cbRead, DEC);
#endif            
      }
    }

    //-----------------------------------------------------------------------------
    // [XBEE_DEBUG_ATTACH]
    //-----------------------------------------------------------------------------
#ifdef XBEE_DEBUG_OUTPUT
    else if ((g_diystate.bAPIPacket[bDataOffset + 0] == XBEE_DEBUG_ATTACH) && (cbRead == 2)) {
      word w = (g_diystate.bAPIPacket[bDataOffset + 1] << 8) + g_diystate.bAPIPacket[bDataOffset + 2];    // take care of warning, probably not needed
      if (w != g_diystate.wDBGDL) {
        g_diystate.wDBGDL = w;    // take care of warning, probably not needed
        //g_fDebugOutput = true;  // Let user control this...
        MSound (2, 50, 1500, 50, 2500);
      }
    }    

    //-----------------------------------------------------------------------------
    // [XBEE_DEBUG_DETACH]
    //-----------------------------------------------------------------------------
    else if (g_diystate.bAPIPacket[bDataOffset + 0] == XBEE_DEBUG_DETACH) {
      g_diystate.wDBGDL = 0xffff;
      g_fDebugOutput = false;  // turn off debug output
      MSound (2, 50, 2500, 50, 1500);
    }    

    //-----------------------------------------------------------------------------
    // [XBEE_DEBUG_STRING]
    //-----------------------------------------------------------------------------
    else if (g_diystate.bAPIPacket[bDataOffset + 0] == XBEE_DEBUG_STRING) {
      XBDSerial.SetInputText(&g_diystate.bAPIPacket[bDataOffset + 1], cbRead); // need to check byte order
      // Ok lets take this and append it onto our debug input text buffer...
    }    
#endif
    //-----------------------------------------------------------------------------
    // [UNKNOWN PACKET]
    //-----------------------------------------------------------------------------
    else {
#ifdef DEBUG
      if (g_fDebugOutput) {
        DBGSerial.print("Unknown Packet: ");
        DBGSerial.print(g_diystate.bAPIPacket[bDataOffset + 0], HEX);
        DBGSerial.print(" ");
        DBGSerial.println(cbRead, DEC);
      }
#endif //DEBUG
    }
  }

  //-----------------------------------------------------------------------------
  // Exited above loop now See if we we are in some form of timeout condition.
  // We should receive a new data message about 30 times per second.  If we
  // have gone for a reasonably long time without receiving a message, assume
  // transmitter went away!
  //-----------------------------------------------------------------------------
  // Only send when we know the transmitter is ready.  Also if we are in the New data only mode don't ask for data unless we have been 
  //    old there
  // is new data. We relax this a little and be sure to ask for data every so often as to make sure the remote is still working...
  // 
  if (g_diystate.fDataPacketsReceived) {
    ulCurrentTime = millis();
    ulTimeDiffMS = ulCurrentTime - g_diystate.ulLastPacket;

    // See if we exceeded a global timeout.  If so let caller know so they can stop themself if necessary...
    if (ulTimeDiffMS > CXBEETIMEOUTRECVMS) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        if (!s_fDisplayedTimeout) {
          DBGSerial.println("XBEE Timeout"); 
          s_fDisplayedTimeout = true;
        }
      }                
#endif //DEBUG
      g_diystate.fPacketValid = 0;
      return false;
    }

    g_diystate.fPacketValid = _fPacketValidPrev;    // Say the data is in the same state as the previous call...
  }

  return g_diystate.fPacketValid;
}



//==============================================================================
// XBee Debug... 
//==============================================================================
#ifdef XBEE_DEBUG_OUTPUT

XBeeDebugSerial::XBeeDebugSerial() {
  _cbOut = 0;
  _cbIn = 0;
  _iIn = 0;
}

XBeeDebugSerial::~XBeeDebugSerial() {
}

void XBeeDebugSerial::begin(long speed) {
}

size_t XBeeDebugSerial::write(uint8_t b) {
  // going to be a pretty stupid system to start...
  // If a debug monitor has registered with us, we will 
  // simply dump stuff into our debug buffer until we get CR or max length
  // then send remote display string command...
  if (g_diystate.wDBGDL != 0xffff) {
    if (b != 0x0a)  { // ignore lf
      if (b == 0x0d) {  // have a CR so lets output the string to the other side...
        _abOut[_cbOut] = 0;  // Make sure it is null terminated...
        SendXBeePacket(g_diystate.wDBGDL, XBEE_RECV_DISP_STR, _cbOut, (uint8_t*)_abOut);
        _cbOut = 0;
      } 
      else {
        _abOut[_cbOut++] = b;
        if( _cbOut >= XBEE_DEBUG_MAX_OUTPUT) {
          SendXBeePacket(g_diystate.wDBGDL, XBEE_RECV_DISP_STR, _cbOut, (uint8_t*)_abOut);
          _cbOut = 0;
        }
      }
    }
    return 1;
  }
  return 0;
}

int XBeeDebugSerial::read() {
  // if Any characters are in the queue that we have not processed return it...
  if (_iIn < _cbIn)
    return _abIn[_iIn++];

  return -1;
}

int XBeeDebugSerial::peek() {
  if (_iIn < _cbIn)
    return _abIn[_iIn];

  return -1;
}

int XBeeDebugSerial::available() {
  return _cbIn - _iIn;
}

void XBeeDebugSerial::flush() {
  XBeeSerial.flush();    // Try waiting for XBee to complete all outputs...
}

void XBeeDebugSerial::SetInputText(uint8_t *pb, uint8_t cb) {
  // Not thread safe, but what the heck... 
  _cbIn = cb;
  _iIn = 0;

  for (int i = 0; i < cb; i++)
    _abIn[i] = *pb++;
}

#endif //XBEE_DEBUG_OUTPUT
#endif //USE_XBEE ?? Should we remove?





