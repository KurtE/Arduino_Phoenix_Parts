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

#include "BioloidEx.h"
#include <avr/pgmspace.h>

/* initializes serial1 transmit at baud, 8-N-1 */
BioloidControllerEx::BioloidControllerEx(long baud){
  int i;
  // setup storage
  id_ = (unsigned char *) malloc(AX12_MAX_SERVOS * sizeof(unsigned char));
  pose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
  nextpose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
  speed_ = (int *) malloc(AX12_MAX_SERVOS * sizeof(int));
  // initialize
  for(i=0;i<AX12_MAX_SERVOS;i++){
    id_[i] = i+1;
    pose_[i] = 512;
    nextpose_[i] = 512;
  }
  frameLength = BIOLOID_FRAME_LENGTH;
  interpolating = 0;
  playing = 0;
  nextframe_ = millis();
  ax12Init(baud);  
}

/* new-style setup */
void BioloidControllerEx::setup(int servo_cnt){
  int i;
  // setup storage
  id_ = (unsigned char *) malloc(servo_cnt * sizeof(unsigned char));
  pose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
  nextpose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
  speed_ = (int *) malloc(servo_cnt * sizeof(int));
  // initialize
  poseSize = servo_cnt;
  for(i=0;i<poseSize;i++){
    id_[i] = i+1;
    pose_[i] = 512;
    nextpose_[i] = 512;
  }
  interpolating = 0;
  playing = 0;
  nextframe_ = millis();
}
void BioloidControllerEx::setId(int index, int id){
  id_[index] = id;
}
int BioloidControllerEx::getId(int index){
  return id_[index];
}

/* load a named pose from FLASH into nextpose. */
void BioloidControllerEx::loadPose( const unsigned int * addr ){
  int i;
  poseSize = pgm_read_word_near(addr); // number of servos in this pose
  for(i=0; i<poseSize; i++)
    nextpose_[i] = pgm_read_word_near(addr+1+i) << BIOLOID_SHIFT;
}
/* read in current servo positions to the pose. */
void BioloidControllerEx::readPose(){
  for(int i=0;i<poseSize;i++){
    pose_[i] = ax12GetRegister(id_[i],AX_PRESENT_POSITION_L,2)<<BIOLOID_SHIFT;
    delay(25);   
  }
}
/* write pose out to servos using sync write. */
void BioloidControllerEx::writePose(){
  int temp;
  int length = 4 + (poseSize * 3);   // 3 = id + pos(2byte)
  int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION_L;
  setTXall();
  ax12write(0xFF);
  ax12write(0xFF);
  ax12write(0xFE);
  ax12write(length);
  ax12write(AX_SYNC_WRITE);
  ax12write(AX_GOAL_POSITION_L);
  ax12write(2);
  for(int i=0; i<poseSize; i++)
  {
    temp = pose_[i] >> BIOLOID_SHIFT;
    checksum += (temp&0xff) + (temp>>8) + id_[i];
    ax12write(id_[i]);
    ax12write(temp&0xff);
    ax12write(temp>>8);
  } 
  ax12write(0xff - (checksum % 256));
  setRX(0);
}

/* set up for an interpolation from pose to nextpose over TIME 
 milliseconds by setting servo speeds. */
void BioloidControllerEx::interpolateSetup(int time){
  int i;
  int frames = (time/frameLength) + 1;
  nextframe_ = millis() + frameLength;
  // set speed each servo...
  for(i=0;i<poseSize;i++){
    if(nextpose_[i] > pose_[i]){
      speed_[i] = (nextpose_[i] - pose_[i])/frames + 1;
    }
    else{
      speed_[i] = (pose_[i]-nextpose_[i])/frames + 1;
    }
  }
  interpolating = 1;
}
/* interpolate our pose, this should be called at about 30Hz. */
#define WAIT_SLOP_FACTOR 10  
void BioloidControllerEx::interpolateStep(boolean fWait){
  if(interpolating == 0) return;
  int i;
  int complete = poseSize;
  if (!fWait) {
    if (millis() < (nextframe_ - WAIT_SLOP_FACTOR)) {
      return;    // We still have some time to do something... 
    }
  }
  while(millis() < nextframe_) ;
  nextframe_ = millis() + frameLength;
  // update each servo
  for(i=0;i<poseSize;i++){
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
  if(complete <= 0) interpolating = 0;
  writePose();      
}

/* get a servo value in the current pose */
int BioloidControllerEx::getCurPose(int id){
  for(int i=0; i<poseSize; i++){
    if( id_[i] == id )
      return ((pose_[i]) >> BIOLOID_SHIFT);
  }
  return -1;
}
/* get a servo value in the next pose */
int BioloidControllerEx::getNextPose(int id){
  for(int i=0; i<poseSize; i++){
    if( id_[i] == id )
      return ((nextpose_[i]) >> BIOLOID_SHIFT);
  }
  return -1;
}
/* set a servo value in the next pose */
void BioloidControllerEx::setNextPose(int id, int pos){
  for(int i=0; i<poseSize; i++){
    if( id_[i] == id ){
      nextpose_[i] = (pos << BIOLOID_SHIFT);
      return;
    }
  }
}

/* Added by Kurt */
void BioloidControllerEx::setNextPoseByIndex(int index, int pos) {  // set a servo value by index for next pose
  if (index < poseSize) {
    nextpose_[index] = (pos << BIOLOID_SHIFT);
  }
}
/* play a sequence. */
void BioloidControllerEx::playSeq( const transition_t  * addr ){
  sequence = (transition_t *) addr;
  // number of transitions left to load
  transitions = pgm_read_word_near(&sequence->time);
  sequence++;    
  // load a transition
  loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
  interpolateSetup(pgm_read_word_near(&sequence->time));
  transitions--;
  playing = 1;
}
/* keep playing our sequence */
void BioloidControllerEx::play(){
  if(playing == 0) return;
  if(interpolating > 0){
    interpolateStep();
  }
  else{  // move onto next pose
    sequence++;   
    if(transitions > 0){
      loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
      interpolateSetup(pgm_read_word_near(&sequence->time));
      transitions--;
    }
    else{
      playing = 0;
    }
  }
}



