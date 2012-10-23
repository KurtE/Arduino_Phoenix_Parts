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

#ifndef BioloidEx_h
#define BioloidEx_h

/* poses:
 *  PROGMEM prog_uint16_t name[ ] = {4,512,512,482,542}; // first number is # of servos
 * sequences:
 *  PROGMEM transition_t name[] = {{NULL,count},{pose_name,1000},...} 
 */

#include <ax12.h>

/* pose engine runs at 30Hz (33ms between frames) 
   recommended values for interpolateSetup are of the form X*BIOLOID_FRAME_LENGTH - 1 */
#define BIOLOID_FRAME_LENGTH      20
/* we need some extra resolution, use 13 bits, rather than 10, during interpolation */
#define BIOLOID_SHIFT             3

/** a structure to hold transitions **/
typedef struct{
    unsigned int * pose;    // addr of pose to transition to 
    int time;               // time for transition
} transition_t; 

/** Bioloid Controller Class for mega324p/644p clients. **/
class BioloidControllerEx
{
  public:
    /* For compatibility with legacy code */
    BioloidControllerEx(long baud);               // baud usually 1000000
    
    /* New-style constructor/setup */ 
    BioloidControllerEx() {};
    void setup(int servo_cnt);

    /* Pose Manipulation */
    void loadPose( const unsigned int * addr ); // load a named pose from FLASH  
    void readPose();                            // read a pose in from the servos  
    void writePose();                           // write a pose out to the servos
    int getCurPose(int id);                     // get a servo value in the current pose
    int getNextPose(int id);                    // get a servo value in the next pose
    void setNextPose(int id, int pos);          // set a servo value in the next pose
    void setNextPoseByIndex(int index, int pos);  // set a servo value by index for next pose
    void setId(int index, int id);              // set the id of a particular storage index
    int getId(int index);                       // get the id of a particular storage index
    
    /* Pose Engine */
    void interpolateSetup(int time);            // calculate speeds for smooth transition
    void interpolateStep(boolean fWait=true);                     // move forward one step in current interpolation  
    unsigned char interpolating;                // are we in an interpolation? 0=No, 1=Yes
    unsigned char runningSeq;                   // are we running a sequence? 0=No, 1=Yes 
    int poseSize;                               // how many servos are in this pose, used by Sync()
    
    // Kurt's Hacks
    uint8_t frameLength;                        // Allow variable frame lengths, to test...

    /* to interpolate:
     *  bioloid.loadPose(myPose);
     *  bioloid.interpolateSetup(67);
     *  while(bioloid.interpolating > 0){
     *      bioloid.interpolateStep();
     *      delay(1);
     *  }
     */

    /* Sequence Engine */
    void playSeq( const transition_t * addr );  // load a sequence and play it from FLASH
    void play();                                // keep moving forward in time
    unsigned char playing;                      // are we playing a sequence? 0=No, 1=Yes

    /* to run the sequence engine:
     *  bioloid.playSeq(walk);
     *  while(bioloid.playing){
     *      bioloid.play();
     *  }
     */
    
  private:  
    unsigned int * pose_;                       // the current pose, updated by Step(), set out by Sync()
    unsigned int * nextpose_;                   // the destination pose, where we put on load
    int * speed_;                               // speeds for interpolation 
    unsigned char * id_;                        // servo id for this index

//    unsigned long lastframe_;                   // time last frame was sent out  
    unsigned long nextframe_;                   //    
    transition_t * sequence;                    // sequence we are running
    int transitions;                            // how many transitions we have left to load
   
};
#endif

