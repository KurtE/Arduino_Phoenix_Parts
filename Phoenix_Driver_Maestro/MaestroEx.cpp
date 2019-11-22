/*
  BioloidController.cpp - ArbotiX Library for Bioloid Pose Engine
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
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
 
#include "MaestroEx.h"
//#include "Hex_Cfg.h"
#ifdef ESP32
#include <pgmspace.h>
#else
#include <avr\pgmspace.h>
#endif


#if cMAESTRO_IN == 0
#define MAESTROSerial Serial1
#else
SoftwareSerial MAESTROSerial(cMAESTRO_IN, cMAESTRO_OUT);
#endif

#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif


/* initializes serial1 transmit at baud, 8-N-1 */
MaestroControllerEx::MaestroControllerEx(long baud){
  id_ = (unsigned char *) malloc(MAX_NUM_SERVOS * sizeof(unsigned char));
  pose_ = (unsigned int *) malloc(MAX_NUM_SERVOS * sizeof(unsigned int));
  nextpose_ = (unsigned int *) malloc(MAX_NUM_SERVOS * sizeof(unsigned int));
  speed_ = (int *) malloc(MAX_NUM_SERVOS * sizeof(int));
  
  // initialize
  poseSize = 6*NUMSERVOSPERLEG;
  for(int i=0 ; i < MAX_NUM_SERVOS ; i++){
    id_[i] = i;
    pose_[i] = 1500;
    nextpose_[i] = 1500;
  }  
  
  frameLength = MAESTRO_FRAME_LENGTH;
  interpolating = 0;
  playing = 0;
  nextframe_ = millis();
  
  MAESTROSerial.begin(baud);
}

void MaestroControllerEx::setId(int index, int id){
  id_[index] = id;
}

int MaestroControllerEx::getId(int index){
  return id_[index];
}

/* load a named pose from FLASH into nextpose. */
//void MaestroControllerEx::loadPose( const unsigned int * addr ){
//  int i;
//  poseSize = pgm_read_word_near(addr); // number of servos in this pose
//  for(i=0; i<poseSize; i++)
//    nextpose_[i] = pgm_read_word_near(addr+1+i) << BIOLOID_SHIFT;
//}

/* read in current servo positions to the pose. */
void MaestroControllerEx::readPose(){
//  for(int i=0;i<poseSize;i++){
//    pose_[i] = ax12GetRegister(id_[i],AX_PRESENT_POSITION_L,2)<<BIOLOID_SHIFT;
//    delay(25);   
//  }
}

int readPos(int id) {
	byte sendBytes[2];
	byte replyBytes[2];
	int responsePosition;
	
	sendBytes[0] = 0x90; // Command byte: Set Target.
	sendBytes[1] = id; // First data byte holds channel number.
	
	MAESTROSerial.print(sendBytes[0]);
	MAESTROSerial.print(sendBytes[1]);
	
	replyBytes[0] = MAESTROSerial.read();
	replyBytes[1] = MAESTROSerial.read();
	
	responsePosition = replyBytes[1];          //send x_high to rightmost 8 bits
	responsePosition = responsePosition << 8;  //shift x_high over to leftmost 8 bits
	responsePosition |= replyBytes[0];         //logical OR keeps x_high intact in combined and fills in rightmost 8 bits
	
	return responsePosition;
}

/* write pose out to servos using sync write. */
void MaestroControllerEx::writePose(){
	//writeGroup(id_[0], poseSize, pose_);
	int startId = 0;
	int numIds = 0;
	unsigned int poseSubset[MAX_NUM_SERVOS];
	
	//Need to write each servo position in current pose_
	for(int i = 0; i < poseSize; i++) {
		// loop through ID array and write block of servo positions 
		// when we have a break in pin numbers
		if (id_[i] == id_[i]+1) {
			poseSubset[numIds] = pose_[i];
			numIds++;
		}
		else {
			poseSubset[numIds] = pose_[i];
			numIds++;
			
			writeGroup(startId, numIds, poseSubset);
			numIds = 0;
			startId = i+1;
		}
	}
	
//	for(int i=0; i < poseSize; i++) {
//		writePos(id_[i], pose_[i]);
//	}
}

/**
 * Write single servo position
 */
void MaestroControllerEx::writePos(int pin, int target){
	byte serialBytes[4];
	target = target*4;
	
	serialBytes[0] = 0x84; // Command byte: Set Target.
	serialBytes[1] = pin; // First data byte holds pin/channel number.
	serialBytes[2] = target & 0x7F; // Second byte holds the lower 7 bits of target.
	serialBytes[3] = (target >> 7) & 0x7F;   // Third data byte holds the bits 7-13 of target.

	MAESTROSerial.write(serialBytes, 4);
}

/**
 * Writes to continuous block of servos from startPin
 * id series needs to match the positionArray formatting
 */
void MaestroControllerEx::writeGroup(int startPin, int numServos, unsigned int positionArray[]) {
	int numBytes = 3+2*numServos;
	byte serialBytes[numBytes];
	
	serialBytes[0] = 0x9F;
	serialBytes[1] = numServos;
	serialBytes[2] = startPin;
	
	int j = 3;
	for(int i = 0 ; i < numServos ; i++) {
		int target = positionArray[i]*4;
		
		serialBytes[j] = target & 0x7F;
		j++;
		serialBytes[j] = (target >> 7) & 0x7F;
		j++;
	}
	
	MAESTROSerial.write(serialBytes, numBytes);
}

/**
 * Set up for an interpolation from pose to nextpose over TIME 
 * milliseconds by setting servo speeds. 
 */
void MaestroControllerEx::interpolateSetup(int time){
  int frames = (time/frameLength) + 1;
  nextframe_ = millis() + frameLength;
  
  // set speed each servo...
  for(int i=0;i<poseSize;i++) {
    if(nextpose_[i] > pose_[i]) {
      speed_[i] = (nextpose_[i] - pose_[i])/frames + 1;
    }
    else {
      speed_[i] = (pose_[i] - nextpose_[i])/frames + 1;
    }
  }
  
  interpolating = 1;
}

/**
 * interpolate our pose, this should be called at about 30Hz.
 */
#define WAIT_SLOP_FACTOR 10  
void MaestroControllerEx::interpolateStep(bool fWait){
  if(interpolating == 0) return;
  
  //int i;
  int complete = poseSize;
  
  if (!fWait) {
    if (millis() < (nextframe_ - WAIT_SLOP_FACTOR)) {
      return;    // We still have some time to do something... 
    }
  }

  while(millis() < nextframe_) ;
  nextframe_ = millis() + frameLength;

  // update each servo
  for(int i=0;i<poseSize;i++){
    int diff = nextpose_[i] - pose_[i];
    if(diff == 0){
      complete--;
    }
    else{
      if(diff > 0){
        if(diff < speed_[i]){
          pose_[i] = nextpose_[i];
          complete--;
        }
        else
          pose_[i] += speed_[i];
      }
      else{
        if((-diff) < speed_[i]){
          pose_[i] = nextpose_[i];
          complete--;
        }
        else
          pose_[i] -= speed_[i];                
      }       
    }
  }
  
  if(complete <= 0) 
	  interpolating = 0;
  
  writePose();
}

///* get a servo value in the current pose */
//int MaestroControllerEx::getCurPose(int id){
//  for(int i=0; i<poseSize; i++){
//    if( id_[i] == id )
//      return ((pose_[i]) >> BIOLOID_SHIFT);
//  }
//  return -1;
//}
///* get a servo value in the next pose */
//int MaestroControllerEx::getNextPose(int id){
//  for(int i=0; i<poseSize; i++){
//    if( id_[i] == id )
//      return ((nextpose_[i]) >> BIOLOID_SHIFT);
//  }
//  return -1;
//}

/** 
 * set a servo value in the next pose
 * id - the pin number of the servo we want to set 
 */
void MaestroControllerEx::setNextPose(int id, int pos){
  for(int i=0; i<poseSize; i++){
    if( id_[i] == id ){
      nextpose_[i] = pos;
      return;
    }
  }
}

/* Added by Kurt */
//void MaestroControllerEx::setNextPoseByIndex(int index, int pos) {  // set a servo value by index for next pose
//  if (index < poseSize) {
//    nextpose_[index] = (pos << BIOLOID_SHIFT);
//  }
//}
/* play a sequence. */
//void MaestroControllerEx::playSeq( const transition_t  * addr ){
//  sequence = (transition_t *) addr;
//  // number of transitions left to load
//  transitions = pgm_read_word_near(&sequence->time);
//  sequence++;    
//  // load a transition
//  loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
//  interpolateSetup(pgm_read_word_near(&sequence->time));
//  transitions--;
//  playing = 1;
//}
/* keep playing our sequence */
//void MaestroControllerEx::play(){
//  if(playing == 0) return;
//  if(interpolating > 0){
//    interpolateStep();
//  }
//  else{  // move onto next pose
//    sequence++;   
//    if(transitions > 0){
//      loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
//      interpolateSetup(pgm_read_word_near(&sequence->time));
//      transitions--;
//    }
//    else{
//      playing = 0;
//    }
//  }
//}
