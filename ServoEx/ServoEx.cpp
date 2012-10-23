/*
 ServoEx.cpp - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
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

#include <avr/interrupt.h>
#include <Arduino.h> 

#include "ServoEx.h"

#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds


cServoGroupMove ServoGroupMove;


#define TRIM_DURATION       2                               // compensation ticks to trim adjust for digitalWrite delays // 12 August 2009

//#define NBR_TIMERS        (MAX_SERVOS / SERVOS_PER_TIMER)

static servo_t servos[MAX_SERVOS];                          // static array of servo structures
static volatile int8_t Channel[_Nbr_16timers ];             // counter for the servo being pulsed for each timer (or -1 if refresh interval)

uint8_t ServoCount = 0;                                     // the total number of attached servos

// Group move variables
uint8_t GroupMoveActiveCnt = 0;								// Do we have a group move active at this time?

// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER)) // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)       // returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])            // macro to access servo class by timer and channel

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo 

/************ static functions common to all instances ***********************/

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)
{
  register servo_t *pservo;
  if( Channel[timer] < 0 )
    *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer 
  else{
	pservo = &SERVO(timer,Channel[timer]);
    if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && pservo->Pin.isActive == true )  {
      digitalWrite( pservo->Pin.nbr,LOW); // pulse this channel low if activated   
	  // See if we are in a timed move, if so update the move for the next time through...
	  if (pservo->ticksDelta > 0) {
	    pservo->ticks +=pservo->ticksDelta;
		if (pservo->ticks >= pservo->ticksNew) {
			pservo->ticks = pservo->ticksNew;
			pservo->ticksDelta = 0;
		}
	  }
	  else if (pservo->ticksDelta < 0) {
	    pservo->ticks +=pservo->ticksDelta;
		if (pservo->ticks <= pservo->ticksNew) {
			pservo->ticks = pservo->ticksNew;
			pservo->ticksDelta = 0;
		}
	  }
	} 
  }

  Channel[timer]++;    // increment to the next channel
  if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
	pservo = &SERVO(timer,Channel[timer]);
    *OCRnA = *TCNTn + pservo->ticks;
    if(pservo->Pin.isActive == true)     // check if activated
      digitalWrite( pservo->Pin.nbr,HIGH); // its an active channel so pulse it high  

  }  
  else { 
    // finished all channels so wait for the refresh period to expire before starting over 
    if( ((unsigned)*TCNTn) + 4 < usToTicks(REFRESH_INTERVAL) )  // allow a few ticks to ensure the next OCR1A not missed
      *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);  
    else 
      *OCRnA = *TCNTn + 4;  // at least REFRESH_INTERVAL has elapsed
    Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel
  }
}

#ifndef WIRING // Wiring pre-defines signal handlers so don't define any if compiling for the Wiring platform
// Interrupt handlers for Arduino 
#if defined(_useTimer1)
SIGNAL (TIMER1_COMPA_vect) 
{ 
  handle_interrupts(_timer1, &TCNT1, &OCR1A); 
}
#endif

#if defined(_useTimer3)
SIGNAL (TIMER3_COMPA_vect) 
{ 
  handle_interrupts(_timer3, &TCNT3, &OCR3A); 
}
#endif

#if defined(_useTimer4)
SIGNAL (TIMER4_COMPA_vect) 
{
  handle_interrupts(_timer4, &TCNT4, &OCR4A); 
}
#endif

#if defined(_useTimer5)
SIGNAL (TIMER5_COMPA_vect) 
{
  handle_interrupts(_timer5, &TCNT5, &OCR5A); 
}
#endif

#elif defined WIRING
// Interrupt handlers for Wiring 
#if defined(_useTimer1)
void Timer1Service() 
{ 
  handle_interrupts(_timer1, &TCNT1, &OCR1A); 
}
#endif
#if defined(_useTimer3)
void Timer3Service() 
{ 
  handle_interrupts(_timer3, &TCNT3, &OCR3A); 
}
#endif
#endif


static void initISR(timer16_Sequence_t timer)
{  
#if defined (_useTimer1)
  if(timer == _timer1) {
    TCCR1A = 0;             // normal counting mode 
    TCCR1B = _BV(CS11);     // set prescaler of 8 
    TCNT1 = 0;              // clear the timer count 
#if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF1A);      // clear any pending interrupts; 
    TIMSK |=  _BV(OCIE1A) ;  // enable the output compare interrupt  
#else
    // here if not ATmega8 or ATmega128
    TIFR1 |= _BV(OCF1A);     // clear any pending interrupts; 
    TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt 
#endif    
#if defined(WIRING)       
    timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service); 
#endif	
  } 
#endif  

#if defined (_useTimer3)
  if(timer == _timer3) {
    TCCR3A = 0;             // normal counting mode 
    TCCR3B = _BV(CS31);     // set prescaler of 8  
    TCNT3 = 0;              // clear the timer count 
#if defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF3A);     // clear any pending interrupts;   
	ETIMSK |= _BV(OCIE3A);  // enable the output compare interrupt     
#else  
    TIFR3 = _BV(OCF3A);     // clear any pending interrupts; 
    TIMSK3 =  _BV(OCIE3A) ; // enable the output compare interrupt      
#endif
#if defined(WIRING)    
    timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service);  // for Wiring platform only	
#endif  
  }
#endif

#if defined (_useTimer4)
  if(timer == _timer4) {
    TCCR4A = 0;             // normal counting mode 
    TCCR4B = _BV(CS41);     // set prescaler of 8  
    TCNT4 = 0;              // clear the timer count 
    TIFR4 = _BV(OCF4A);     // clear any pending interrupts; 
    TIMSK4 =  _BV(OCIE4A) ; // enable the output compare interrupt
  }    
#endif

#if defined (_useTimer5)
  if(timer == _timer5) {
    TCCR5A = 0;             // normal counting mode 
    TCCR5B = _BV(CS51);     // set prescaler of 8  
    TCNT5 = 0;              // clear the timer count 
    TIFR5 = _BV(OCF5A);     // clear any pending interrupts; 
    TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt      
  }
#endif
} 

static void finISR(timer16_Sequence_t timer)
{
    //disable use of the given timer
#if defined WIRING   // Wiring
  if(timer == _timer1) {
    #if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    TIMSK1 &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt
    #else 
    TIMSK &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt   
    #endif
    timerDetach(TIMER1OUTCOMPAREA_INT); 
  }
  else if(timer == _timer3) {     
    #if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    TIMSK3 &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
    #else
    ETIMSK &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
    #endif
    timerDetach(TIMER3OUTCOMPAREA_INT);
  }
#else
    //For arduino - in future: call here to a currently undefined function to reset the timer
#endif
}

static boolean isTimerActive(timer16_Sequence_t timer)
{
  // returns true if any servo is active on this timer
  for(uint8_t channel=0; channel < SERVOS_PER_TIMER; channel++) {
    if(SERVO(timer,channel).Pin.isActive == true)
      return true;
  }
  return false;
}


/****************** end of static functions ******************************/

ServoEx::ServoEx()
{
  if( ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    // assign a servo index to this instance
	servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values  - 12 Aug 2009
  }
  else
    this->servoIndex = INVALID_SERVO ;  // too many servos 
}

uint8_t ServoEx::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t ServoEx::attach(int pin, int min, int max)
{
  if(this->servoIndex < MAX_SERVOS ) {
    pinMode( pin, OUTPUT) ;                                   // set servo pin to output
    servos[this->servoIndex].Pin.nbr = pin;  
    // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128 
    this->min  = (MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    this->max  = (MAX_PULSE_WIDTH - max)/4; 
    // initialize the timer if it has not already been initialized 
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if(isTimerActive(timer) == false)
      initISR(timer);    
    servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
  } 
  return this->servoIndex ;
}

void ServoEx::detach()  
{
  servos[this->servoIndex].Pin.isActive = false;  
  timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if(isTimerActive(timer) == false) {
    finISR(timer);
  }
}

void ServoEx::write(int value)
{  
  if(value < MIN_PULSE_WIDTH)
  {  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    if(value < 0) value = 0;
    if(value > 180) value = 180;
    value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());      
  }
  this->writeMicroseconds(value);
}

void ServoEx::writeMicroseconds(int value)
{
  // calculate and store the values for the given channel
  byte channel = this->servoIndex;
  if( (channel < MAX_SERVOS) )   // ensure channel is valid
  {  
    if( value < SERVO_MIN() )          // ensure pulse width is valid
      value = SERVO_MIN();
    else if( value > SERVO_MAX() )
      value = SERVO_MAX();   
    
  	value = value - TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

	// Changes to handle multiple servo group move
	if (GroupMoveActiveCnt)
      servos[channel].ticksPending = value;  
	else {
      uint8_t oldSREG = SREG;
      cli();
      servos[channel].ticks = value;  
      SREG = oldSREG;   
	}
  } 
}

int ServoEx::read() // return the value as degrees
{
  return  map( this->readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);     
}

int ServoEx::readMicroseconds()
{
  unsigned int pulsewidth;
  if( this->servoIndex != INVALID_SERVO )
    pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION ;   // 12 aug 2009
  else 
    pulsewidth  = 0;

  return pulsewidth;   
}

bool ServoEx::attached()
{
  return servos[this->servoIndex].Pin.isActive ;
}

bool ServoEx::moving()
{
  return (servos[this->servoIndex].ticksDelta != 0) ;
}

void ServoEx::move(int value, unsigned int MoveTime)
{
	// For now just do shorthand of start, write and commit
	ServoGroupMove.start();
	write(value);
	ServoGroupMove.commit(MoveTime);
}


//====================================================================================

void cServoGroupMove::start(void) 
{
	// 
	if (!GroupMoveActiveCnt) {
		uint8_t i;
		for (i=0; i < ServoCount; i++)
			servos[i].ticksPending = (unsigned int)-1;	// special value to say not part of group move.
	}
	GroupMoveActiveCnt++;	// Increment counter to say we are in a group move.
}

void cServoGroupMove::commit(unsigned int wMoveTime)
{
    uint8_t oldSREG = SREG;
	uint8_t 
	i;
	if (GroupMoveActiveCnt) {
		if ((--GroupMoveActiveCnt) == 0) {
			// Ok we are back to zero.  Need to convert the move time to number of cycles...
			// Lets convert the move time into number of Servo Intervals
			// NOte Refresh_interval is in microseconds and our time was in milliseconds
			wMoveTime = (wMoveTime + (REFRESH_INTERVAL/2000))/(REFRESH_INTERVAL/1000);
			if (wMoveTime) {
				// At least one clock tick so now 
				for (i=0; i < ServoCount; i++) {
					if ((servos[i].ticksPending != (unsigned int)-1) && (servos[i].Pin.isActive) && 
							(servos[i].ticks != servos[i].ticksPending)) {
						cli();
						servos[i].ticksNew = servos[i].ticksPending;  
						servos[i].ticksDelta = (int)((int)servos[i].ticksNew - (int)servos[i].ticks)/ (int)wMoveTime;
						if (!servos[i].ticksDelta) 
							servos[i].ticksDelta = (servos[i].ticksNew > servos[i].ticks)? 1 : -1;
						SREG = oldSREG;   
					}
				}
			}
			else {
				// less than one clock tick lets just set all of the active ones to their new values...
				for (i=0; i < ServoCount; i++) {
					if ((servos[i].ticksPending != (unsigned int)-1) && (servos[i].Pin.isActive)) {
						cli();
						servos[i].ticks = servos[i].ticksPending;  
						SREG = oldSREG;   
					}
					servos[i].ticksDelta = 0;	// make sure they are all cleared out.
				}
			}
		}	
	}	
}

void cServoGroupMove::abort()
{
	GroupMoveActiveCnt = 0;	// clear out the counts...
}


uint32_t cServoGroupMove::moving(void)
{
	uint8_t i;
	uint32_t	ulRet = 0;
	uint32_t	ulMask = 1;
	for (i=0; i < ServoCount; i++) {
		if (servos[i].ticksDelta != 0) 
			ulRet |= ulMask;
		ulMask <<= 1;	// setup for next servo...
	}
	return ulRet;
}

void cServoGroupMove::wait(uint32_t ulSGMMask)
{
	uint8_t i;
	for (i=0; (i < ServoCount) && ulSGMMask; i++) {
		if (ulSGMMask & 0x1) { 
			// We are interested in this servo...
			while (servos[i].ticksDelta != 0) 
				delay(1);
		}
		ulSGMMask >>= 1;
	}
}
