Warning
=======

This is a Work In Progress!  There are no warrantees or Guarantees of any type that this code is useable for anything.  

But I hope it is.

This set of directories are my latest attempt of breaking up the Phoenix code base for the Arduino, 
into libraries that you can use to build different builds for many different hexapods (and hopefully Quads as well)

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

Major Contributors
==================

Jeroen Janssen [aka Xan] -  The original Phoenix code was written by to run on the Lynxmotion Phoenix 
(http://www.lynxmotion.com/c-117-phoenix.aspx). It was originally written in Basic for the Basic Atom Pro 28
processor by Basic Micro.  

Kåre Halvorsen (aka Zenta) -  The Lynxmotion Phoenix was based on the original Phoenix that was developed by
Him,  and a lot of the software was based off of his earlier Excel spreadsheet (PEP).  More details up on his 
Project page (http://www.lynxmotion.com/images/html/proj098.htm).

Me - I later ported the code to C/C++ and the Arduino environment and with the help of Kåre and Jeroen hopefully 
reduced the number of bugs I introduced as part of this port.   

Michael E. Ferguson (lnxfergy up on Trossen) - Arbotix Commander, Ax12...

Bill Porter - PS2 library for Arduino.


Again Warning
=============

This is a WIP - No promises or guarantees!
