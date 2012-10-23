// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.
// Modified by Kurt to use the ServoEx library


#include <ServoEx.h> 
 
ServoEx myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(13);  // attaches the servo on pin 13 to the servo object 
} 
 
 
void loop() 
{ 
  if (myservo.moving())
    return;  // last move is still active
  pos = (pos == 0)? 180 : 0;  
  myservo.move(pos, 2500);
} 
