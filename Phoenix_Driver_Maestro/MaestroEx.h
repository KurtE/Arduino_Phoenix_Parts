/*
  BioloidController.h - ArbotiX Library for Bioloid Pose Engine
  Copyright (c) 2008-2012 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef MaestroEx_h
#define MaestroEx_h

/* poses:
 *  PROGMEM prog_uint16_t name[ ] = {4,512,512,482,542}; // first number is # of servos
 * sequences:
 *  PROGMEM transition_t name[] = {{NULL,count},{pose_name,1000},...} 
 */

#include <inttypes.h>

/* pose engine runs at 30Hz (33ms between frames) 
   recommended values for interpolateSetup are of the form X*BIOLOID_FRAME_LENGTH - 1 */
#define MAESTRO_FRAME_LENGTH      20
#define MAX_NUM_SERVOS 24



/** a structure to hold transitions **/
typedef struct{
  unsigned int * pose;    // addr of pose to transition to 
  int time;               // time for transition
} transition_t; 

/** Bioloid Controller Class for mega324p/644p clients. **/
class MaestroControllerEx
{
  public:
    MaestroControllerEx(HardwareSerial & port, long baud) : port_ (port), baud_ (baud) {
      begin();
      port_.begin(baud_);
    }
    MaestroControllerEx(HardwareSerial & port, long baud, int rxPin, int txPin) : port_(port), baud_(baud), rxPin_(rxPin), txPin_(txPin) {
      begin();
      port_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    }
    
    void begin();
    void setup(int servo_cnt);

    /* Pose Manipulation */
    void  loadPose( const unsigned int * addr );  // load a named pose from FLASH  
    void  readPose();                             // read a pose in from the servos  
    int   readPos(int id);
	  void  writePose();                            // write a pose out to the servos
    void  writePos(int id, int pos);
	  void  writeGroup(int startId, int numServos, unsigned int positionArray[]);
	  int   getCurPose(int id);                     // get a servo value in the current pose
    int   getNextPose(int id);                    // get a servo value in the next pose
    void  setNextPose(int id, int pos);           // set a servo value in the next pose
    void  setNextPoseByIndex(int index, int pos); // set a servo value by index for next pose
    void  setId(int index, int id);               // set the id of a particular storage index
    int   getId(int index);                       // get the id of a particular storage index
    
    /* Pose Engine */
    void interpolateSetup(int time);              // calculate speeds for smooth transition
    void interpolateStep(bool fWait=true);        // move forward one step in current interpolation  
    unsigned char interpolating;                  // are we in an interpolation? 0=No, 1=Yes
    unsigned char runningSeq;                     // are we running a sequence? 0=No, 1=Yes 
    int poseSize;                                 // how many servos are in this pose, used by Sync()
    
    // Kurt's Hacks
    uint8_t frameLength;                          // Allow variable frame lengths, to test...

    /* to interpolate:
     *  bioloid.loadPose(myPose);
     *  bioloid.interpolateSetup(67);
     *  while(bioloid.interpolating > 0){
     *      bioloid.interpolateStep();
     *      delay(1);
     *  }
     */

    /* Sequence Engine */
    void playSeq( const transition_t * addr );    // load a sequence and play it from FLASH
    void play();                                  // keep moving forward in time
    unsigned char playing;                        // are we playing a sequence? 0=No, 1=Yes

    /* to run the sequence engine:
     *  bioloid.playSeq(walk);
     *  while(bioloid.playing){
     *      bioloid.play();
     *  }
     */
    
  private:  
    unsigned int * pose_;                         // the current pose, updated by Step(), set out by Sync()
    unsigned int * nextpose_;                     // the destination pose, where we put on load
    int * speed_;                                 // speeds for interpolation 
    unsigned char * id_;                          // servo id for this index

    unsigned long nextframe_;                     //    
    transition_t * sequence;                      // sequence we are running
    int transitions;                              // how many transitions we have left to load
    
    HardwareSerial & port_;
    long baud_;
    int rxPin_;
    int txPin_;
   
};
#endif
