This project - is my current WIP for supporting all of my current existing   Lynxmotion based Hex robots.

First the disclaimer: 
 This project is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.


Currently I am in the process of trying to merge in all of the functionality of my different code bases into one 
project.  Once this is  working I will try to reorganize the code such that it is easy to create a simple project that
is specific to one specific robot and yet have a way such that I can make fixes or add functionality in one place 
and have it apply to all of the configurations that it should

The original Phoenix code was written by Jeroen Janssen [aka Xan] to run on the Lynxmotion Phoenix 
(http://www.lynxmotion.com/c-117-phoenix.aspx). It was originally written in Basic for the Basic Atom Pro 28
processor by Basic Micro.  The Lynxmotion Phoenix was based on the original Phoenix that 
was developed by Kåre Halvorsen (aka Zenta) and a lot of the software was based off of his earlier Excel 
spreadsheet (PEP).  More details up on his Project page (http://www.lynxmotion.com/images/html/proj098.htm).

I later ported the code to C/C++ and the Arduino environment and with the help of Kåre and Jeroen hopefully 
reduced the number of bugs I introduced as part of this port.   

I am also making use of the work of others including: 

Michael E. Ferguson (lnxfergy up on Trossen) - Arbotix Commander, Ax12...

Bill Porter - PS2 library for Arduino.

...

Currently the way I choose which robot I am building for is a very large kludge.  This is currently the main Hex_Cfg.h file
 is simply a place holder where you define which one you want and I have separate header files for each of my configurations. Looks like: 
    #ifndef HEX_CFG_H
    #define HEX_CFG_H
    // Only include one of these configurations!
    #include "Hex_Cfg_Chr3.h"
    //#include "Hex_Cfg_Thr4.h"
    //#include "Hex_Cfg_THex3.h"
    #endif

I then have each of the Input Controllers, with #ifdef around if it is being used or not: Like: #ifdef USEPS2 or #ifdef USESERIAL ... 

Currently the code base has the Input controller code for: 
    PS2 - using the library from Bill Porter
    Serial -  using the old Powerpod Serial test interface.
    Arbotix Commander 2 (Real simple but limited XBee Controller). 

Next will be to merge in my DIY XBee stuff, probably also including the ability to send debug stuff to the PC...

Currently this code base only has the SSC-32 servo controller support, but soon will add in some additional ones including:
    Arduino Mega  - running servos directly.
   Chipkit Max32/Uno32 running servos
  AX12... 

The CHR-3 cfg file has been updated to allow multiple different processors, in particular it has Botboarduino, Arduino Mega using my Shield 
and Chipkit Max32 using my Shield. Currently I am testing it with the Max32 with SSC-32.  If you are curious, more information about the
shield is up on the thread: http://www.lynxmotion.net/viewtopic.php?f=26&t=8021&start=47
  



