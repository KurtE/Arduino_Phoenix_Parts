/*
  ServoEx.h - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
  Copyright (c) 2009 Michael Margolis.  All right reserved.
  
  This version was extended by Kurt Eckhardt to allow for timed moves, including a group move where
  one or more servos are told to go a new end point and that they should take a specific time to
  get there.

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

/* 
  
  A servo is activated by creating an instance of the ServoEx class passing the desired pin to the attach() method.
  The Servos are pulsed in the background using the value most recently written using the write() method

  Note that analogWrite of PWM on pins associated with the timer are disabled when the first servo is attached.
  Timers are seized as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.
  The sequence used to sieze timers is defined in timers.h

  The methods are:

   ServoEx - Class for manipulating servo motors connected to Arduino pins.

   attach(pin )  - Attaches a servo motor to an i/o pin.
   attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
   default min is 544, max is 2400  
 
   write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
   writeMicroseconds() - Sets the servo pulse width in microseconds 
   read()      - Gets the last written servo pulse width as an angle between 0 and 180. 
   readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
   attached()  - Returns true if there is a servo attached. 
   detach()    - Stops an attached servos from pulsing its i/o pin. 
   
   New methods:
	moving  	- Returns true if the servo is still moving to it's new location.
    move	 	- Move the one servo to a new location...
	
	New Class cServoGroupMove - used to start a new group move.  There is one instance of this class
		defined ServoGroupMove. 
		
	The methods are:	

    start 		- Starts a group move.  The servos are not moved until the commit call
    commit		- Commits the moves(write, writeMicroseconds) that happened since start

    moving		- Returns a bitmask of the servos that are still moving.  The bits are in the order
				  the servos were created.
    wait		- Waits for all of the servos defined in the mask are to their end points.
 
 */

#ifndef _Servo_Group_Move_h_
#define _Servo_Group_Move_h_

#include <inttypes.h>

/* 
 * Defines for 16 bit timers used with  ServoEx library 
 *
 * If _useTimerX is defined then TimerX is a 16 bit timer on the curent board
 * timer16_Sequence_t enumerates the sequence that the timers should be allocated
 * _Nbr_16timers indicates how many 16 bit timers are available.
 *
 */

// Say which 16 bit timers can be used and in what order
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define _useTimer5
#define _useTimer1 
#define _useTimer3
#define _useTimer4 
typedef enum { _timer5, _timer1, _timer3, _timer4, _Nbr_16timers } timer16_Sequence_t ;

#elif defined(__AVR_ATmega32U4__)  
#define _useTimer1 
typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t ;

#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#define _useTimer3
#define _useTimer1
typedef enum { _timer3, _timer1, _Nbr_16timers } timer16_Sequence_t ;

#elif defined(__AVR_ATmega128__) ||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
#define _useTimer3
#define _useTimer1
typedef enum { _timer3, _timer1, _Nbr_16timers } timer16_Sequence_t ;

#else  // everything else
#define _useTimer1
typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t ;                  
#endif

#define Servo_VERSION           2      // software version of this library

#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define REFRESH_INTERVAL    20000     // minumim time to refresh servos in microseconds 

#define SERVOS_PER_TIMER       12     // the maximum number of servos controlled by one timer 
#define MAX_SERVOS   (_Nbr_16timers  * SERVOS_PER_TIMER)

#define INVALID_SERVO         255     // flag indicating an invalid servo index

typedef struct  {
  uint8_t nbr        :6 ;             // a pin number from 0 to 63
  uint8_t isActive   :1 ;             // true if this channel is enabled, pin not pulsed if false 
} ServoPin_t   ;  

typedef struct {
  ServoPin_t Pin;
  unsigned int ticks;				  // Current Tick count 
  unsigned int ticksNew;			  // New end point tick count
  int	ticksDelta;					  // How much to change per servo cycle
  unsigned int ticksPending; 		  // New pending value that commit will use.	
} servo_t;

class ServoEx
{
public:
  ServoEx();
  uint8_t attach(int pin);           // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(int pin, int min, int max); // as above but also sets min and max values for writes. 
  void detach();
  void write(int value);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
  void writeMicroseconds(int value); // Write pulse width in microseconds 
  int read();                        // returns current pulse width as an angle between 0 and 180 degrees
  int readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached();                   // return true if this servo is attached, otherwise false 
  bool moving();					 // return true if the servo is still moving  
  void move(int value, unsigned int MoveTime); // A one servo group move...
private:
#if defined(__arm__)
   uint16_t min_ticks;
   uint16_t max_ticks;
#else
   int8_t min;                       // minimum is this value times 4 added to MIN_PULSE_WIDTH    
   int8_t max;                       // maximum is this value times 4 added to MAX_PULSE_WIDTH   
#endif
   uint8_t servoIndex;               // index into the channel data for this servo
};


class cServoGroupMove {
  public:
    // Constructor. Set up variables.
    // Warning: currently this is limited to 32 servos...
    //ServoGroupMove();

    void     start(void);                            // Start a group move
    void     commit(unsigned int wMoveTime);         // how long in milliseconds the move should take
	void	 abort(void);							 // Abort any moves that may be active

    uint32_t moving(void);			    // returns bit mask for which servos are active.
    void     wait(uint32_t ulSGMMask);

};

extern cServoGroupMove ServoGroupMove;

#endif