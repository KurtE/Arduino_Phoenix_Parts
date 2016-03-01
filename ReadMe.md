Warning
=======

This is a Work In Progress!  There are no warrantees or Guarantees of any type that this code is useable for anything.  

But I hope it is.

This set of directories are my latest attempt of breaking up the Phoenix code base for the Arduino, 
into libraries that you can use to build different builds for many different hexapods as well as a few quads

Quad SUPPORT
============

WARNING: This branch now has some and maybe even some support for Octopods.  

Note: This includes functionality of the changes made by Jeroen (Xan) for the code up on the Lynxmotion 
gethub Quadcode... However I changed the implementation such that you can choose which servos need to be reversed in the 
hex config file, also not specific to quads. This change required updates to all of the servo Driver files

General
=======

Most of the code in these libraries are in header files, which allows them to be compiled specifically for each project.   


Installation
============

To use this code you need to copy each of the included directories into your Arduino Library directory.  

I personally use the User library directory, which is located some place like: 

c:\users\kurt\My Documents\Arduino\libraries.

In the Phoenix library there are several examples of configurations.  

Once these directories are installed, you can simply go to the file menu, Choose the Examples menu item, then
in this sub-menu there should be a menu item Phoenix, which when chosen, should give you several configurations…

Once you load a configuration that is either your configuration or the one closest to it, you can then save this
sketch into your own sketchbook and make any modifications that are necessary or desired to make your robot
work correctly.

Library Descriptions
====================

Main Phoenix Code
-----------------

Phoenix - Contains the main header file with data descriptions as well as the main code base.  In addition 
it contains sub-folders with examples

Servo Drivers
-------------

Note: You only need to copy the libraries you need to your Arduino libraries folder.  For example most Lynxmotion
robots will only need the SSC-32 version.  Most Trossen ones will only need the AX12 version.

Phoenix_Driver_SSC32 - This is the main driver for robots that use the Lynxmotion SSC-32 to drive the servos.  This
includes all of the Lynxmotion robots.

Phoenix_Driver_AX12 - This is the servo driver used for Hexapods that use the Robotis AX-12 or AX-18 servos 
such as PhantomX

Phoenix_Driver_Orion - Test code that allows me to use the Orion Robotics Orion Shield to drive standard servos

Phoenix_Driver_ServoEx - This is a version that uses the ServoEx library which I will mention below. This will only
work on Arduino Mega class Arduinos.

ServoEx - This is my own enhanced version of the Arduino Servo library that adds the ability for timed moves.


Input Classes 
-------------

Note: Again you only need to install the input libraries you wish to try out.


Phoenix_Input_Commander - Support for the Arbotix Commander.  This is used with most of the Trossen Robots

Phoenix_Input_DIYXbee - Support for our DIY XBee remote control.  Only a few of us have this.

Phoenix_Input_PS2 - Support for the Lynxmotion PS2 (as well as several others).  Used with most of the Lynxmotion
robots.  Note: You also need to install the Bill Porter Arduino PS2 library. You can get this from the Lynxmotion
github or several other places

Phoenix_Input_RC - A first pass at RC input. I believe this one was for my Hitec 6...

Phoenix_Input_Serial - This one talks to a serial port and uses the old Lynxmotion Powerpod serial protocol.  So you
can download that program from Lynxmotion and use their Test program to try it out.

Some Notes about Capabilities and Options
========================================

There are features and options that are part of the code base that should probably be documented.  While the below is
not complete, I hope it might help out some.  Sorry that the information here is a bit rough.

\#define Options
--------------

Many of the different Input drivers as well as the servo drivers have options that can be defined.  Many of the options
are in place to either include or exclude code from the executable.  Some are specific to each of the drivers, but some 
are more generic.  Here are a few of the ones that I picked out from the others.

First there is simple defines for selecting something major like four degrees for freedom and Quad mode (versus the default
of hexapod)

\#define c4DOF

\#define QUADMODE

Allow the actual program to do something at startup.  Example with Orion driver I may want to detect if a button is pressed
and hang there in a loop as to allow me to update the firmware of the Orion.
\#define OPT_SKETCHSETUP // If defined will call   SketchSetup();

Some Servo drivers needs the main code to call it whenever it can as to do some background work.  Example AX12 to do the
default interpolation. 
\#define OPT_BACKGROUND_PROCESS

The Original Phoenix code base has the ability to run General Purpose Sequences that were stored on the SSC-32 servo driver.  For some
of the other servo drivers I have done code to emulate this.  On AX-12 I allow you to import Pypose sequences.  This type of code is
enabled by:
\#define OPT_GPPLAYER

For some robots, I am still experimenting with the ability to adjust the leg positions (angles between legs) as well as the distance of
the leg from the center.  Currently only supported using the Commander.  
#define ADJUSTABLE_LEG_ANGLES

For those robots whose controller boards who have a Serial port that you can use to talk to something like the Arduino Serial Monitor, you
can define DBGSerial, to the actual Serial port.  On other boards, example Arbotix, where Serial is used by the XBee and Serial1 is used for
talking to the AX servos, you typically cannot define this.   
\#define DBGSerial

If DBGSerial is defined, then you can also enable other features.  In particular a simple Terminal monitor, that allows you to type in simple
commands.  If defined, there are a few very basic commands that are part of the main code (Toggle Debug on and Off), and each of the Servo
drivers can add additional commands.  In addition, with this latest delta you can specify that an Input Controller can add commands as well.
Note: The terminal monitor is mainly only called when the Robot is logically Off.  With the Commander, this is when the commander is turned off.
With PS2, the robot is toggled on and off by the Start button.
          
\#define OPT_TERMINAL_MONITOR  
\#define OPT_TERMINAL_MONITOR_IC  // Allow Input controller to define stuff as well

Within the Terminal monitor there is also support to include or not include some of the larger commands.  Example some of the Servo drivers (SSC-32) has
a command that allows you to adjust the zero position of the servos and then store that data away.
\#define OPT_FIND_SERVO_OFFSETS

Likewise there is code in the SSC-32 driver to try to forward stuff that is received on the terminal Serial port to the SSC-32 and likewise data from the SSC-32
back to the Terminal Serial port, in attempt to allow you to use programs on a PC to talk directly to the SSC-32.
#define OPT_SSC_FORWARDER

With some of the more recent builds, the gaits have been defined as a structure.  With this you now have the ability to have your specific robot 
completely replace the set of gaits used or add gaits to the default list.  An example of this is defined in the Phantom Phoenix Quad example under Phoenix. 
\#define OVERWRITE_GAITS
\#define ADD_GAITS

With the use of Lipo or LifePo4 types of batteries, it is good idea to try to shut off the robot before the battery is drained too far.  (100ths of volt)  
When the power goes below this point, if possible it will shut off all of the servos. And if sound is enabled, it will make some noise.  You should still
shut the system off as the processor will still be using up the battery.
\#define cTurnOffVol 1000

If cTurnOffVol is defined you can also define a voltage that allows the robot to turn back on.  This was needed as sometimes you start a robot on USB, 
before power switch is turned on.  So if you then turn on power it would be ok to again allow the robot to turn on.
#\define cTurnOnVol 1100

With Some servo drivers like the AX12 driver, the servo driver code can get the voltage from the servos.  But if your processor board has the ability to get the 
voltage, you can instead define which pin to do the analogRead from.
#define cVoltagePin 

Since by default the voltage to the IO pins cannot exceed the system voltage, often these pins are connected up through a resistor divider circuit (2 resistors).  
Example Lynxmotion BotBoarduino has a 10K and 30K resistor.  So you need to define this somewhere.  Actually you just need the ratios
\#define CVADR1  40
\#define CVADR2  10

Up till now all of the boards were 5V, but now with Teensy and the like some or now 3.3v so needed somewhere to define.  Defaults to 5V
\#define CVREF   500

Terminal Monitor
----------------

Again this is cut and paste from posts, so real rough...


D<CR> - Toggles debug on and off. Allows me to have code in place that either outputs or does not. (The command simply toggles: g_fDebugOutput)

E - EEPROM dumping functions. There is some quick and dirty code here. If the next character is a h, it will dump in hex, 
if the next character is a w it will dump words. The next parameter is the start address. If not given starts where the previous one left off. 
Next optional one is the number of bytes to dump  (Not on all boards)

(SSC32 mainly)
O - Enter Servo Offset mode. sometimes I find, that I screw up and plug the servos into the wrong SSC-32 pin. That is one of the good things about the 
servo offset mode that is part of the Terminal monitor. When you enter the O command (If the appropriate option is configured to include it), it enters 
the mode that allows you to use the keyboard to play with the servos. As you select different servos, the code wiggles the servo to let you know 
which one it is, plus it prints out a logical name for the pin like LF Coxa. If this is not the servo you think it is (or nothing moves) then you know 
there is an issue. Using the + and - keys moves the servos a little. (Note with Arduino Monitor more of a pain, as nothing is sent until CR, but you 
can enter multiple +s or -s. You can use the * key to change to the next servo. Also I allow you to enter 0-5 to select a leg (in your case 0-3), 
plus C for Coxa, F for Femur, T for Tibia and not in your case A for Tars.
To exit this mode enter $. It will then prompt if you wish to save your changes, which if so the offsets on the SSC-32 will be updated.

S - Enter SSC-32 forwarder mode.  Exits when it receives a line $<cr>.  Warning if you processor tends to reboot when a terminal is connected (DTR...), this 
can be problematic.

(AX-12 - Again only when you can actually have a debug terminal) 
V - Prints out current Voltage
M - Toggle Motors on or off for debugging
F<frame length> - FL in ms - Allows you to experiment with how long between frames
T - Test Servos - Prints out the current position of each of the servos.  Helps to detect if a servo resets it's id to 1
S - Set id <frm> <to - Allows you to fix the servos if one resets.

Major Contributors
==================

Jeroen Janssen [aka Xan] -  The original Phoenix code was written by to run on the Lynxmotion Phoenix 
(http://www.lynxmotion.com/c-117-phoenix.aspx). It was originally written in Basic for the Basic Atom Pro 28
processor by Basic Micro.  

Kare Halvorsen (aka Zenta) -  The Lynxmotion Phoenix was based on the original Phoenix that was developed by
him.  In addition a lot of the software was based off of his earlier Excel spreadsheet (PEP).  More details up on his 
Project page (http://www.lynxmotion.com/images/html/proj098.htm).

Me - I later ported the code to C/C++ and the Arduino environment and with the help of Kåre and Jeroen hopefully 
reduced the number of bugs I introduced as part of this port.   

Michael E. Ferguson (lnxfergy up on Trossen) - Arbotix Commander, Ax12.

Bill Porter - PS2 library for Arduino.


Again Warning
=============

This is a WIP - No promises or guarantees!
