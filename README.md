# SharcSim

 * This is the Payload Status & Sensor Message Simulator
  
 It is meant to run on a Pololu A-Star 32U4 Prime Board
 (The A-Star Prime is an Arduino Leonardo compatible board
 that includes a microSD card reader, a buzzer, and some
 built in buttons).
  
 If this board is not available, a standard Leonardo
 (or Uno) with a SD shield could be used (with some minor
 code modifications).
  
 USAGE:
 ---------------------------------------------------------
 * Copy the message file to the SD card and name it "test.txt"
  Edit this file and add the following to be the first line in 
  the file (if it is not already the first line):
          $INIT1,6700672,6905874,
          
 * This tells it how many milliseconds into the file to start
 the simulator (in this case about 6700 seconds -- there was
 a lot of waiting around before the rocket was launched, and
 we want to skip that waiting period).  And the second number
 is the end time; in this case about 200 seconds after the start.
  
 * When the simulator starts, it will beep twice if it can
 read the data file.  It will beep 6 times if no SD card,
 and 4 times if it can't read the "test.txt" file.
  
 * The program then waits for the "B" button to be pressed.
  
 * After the button is pressed, it beeps once and starts
 sending status messages over the Serial port in real
 time.  It will also beep every time the state changes.
 It will beep twice when the simulation is completed.
 Note that for a large data file, it may take several
 seconds to seek to the start of the simulation.
  
 * Also, status messages and other debugging info is echoed
 to the USB serial port.
  
 * Serial data is sent at 115200 bps!
  
 * See below for the message format description
  


== microSD card considerations ==

You will need to install a jumper between GND and CS to enable
the microSD card.

(You will also need to have a version of the A-Star 32U4 with a
microSD card socket and you will need to insert a formatted
microSD card into the socket with the message file on it.)

= Buzzer =

You will need to install a jumper on JUMPER-6, to enable
the buzzer.



 MESSAGE FORMAT
---------------------------------------------------------
 Mission State values:
   - STATE_LOADED  1
   - STATE_READY   2
   - STATE_LAUNCH  3
   - STATE_COAST   4
   - STATE_APOGEE  5
   - STATE_DEPLOY  6
   - STATE_EJECT_A 7
   - STATE_EJECT_B 8
   - STATE_COMPLETE 9

-----------------------------------------------------

  * rel time: The (relative) time since launch
  * time: The (absolute) time since start of recording
  * cos angle: The cosine of the angle above horizontal multiplied by 100
           Only calculated in STATE_LOAED -- below 60 (about 53 degrees 
           from horizontal -- horizontal is a cos angle of 99)
  * accel raw: The value from the sensor (x,y,z) squared (no units) 
           1g is about 2000
  * accel count: The number of times in a row that accel is 
           over the launch threshold
  * min alt msl: The altitude on the ground, above mean sea level, 
           not corrected for current atmospheric conditions
  * cur alt agl: The current altitude above ground level (min alt msl)
  * max alt agl: The maximum altitude

Example Data:
-----------------------------------------------------

MSG_TYP, state, rel time (ms), time (ms), cos ang (x100), accel_raw^2, accel cnt, temp (F), min alt msl (ft), cur alt agl (ft), max alt agl (ft)
  * $INIT1, 6700672,
  * $SYSTA,     1,             0,     12191,             99,        2013,         0,       87,             4376,                0,              13,
  * $SYSTA,     2,             0,   6701679,             59,        1815,         0,       94,             4502,               25,              45,
  * $SYSTA,     3,           436,   6705619,             59,       39936,         9,       94,             4502,               25,              45,
  * $SYSTA,     4,         13431,   6718614,             59,         101,         0,       92,             4502,             4231,            4231,
  * $SYSTA,     5,         29057,   6734240,             59,         529,         0,       91,             4502,             6571,            6631,
  * $SYSTA,     6,         32199,   6737382,             59,         402,         0,       91,             4502,             6341,            6631,
  * $SYSTA,     7,         41413,   6746596,             59,        1496,         0,       92,             4502,             5168,            6631,
  * $SYSTA,     8,        157947,   6863130,             59,        1874,         0,       89,             4502,             1017,            6631,
  * $SYSTA,     9,        429187,   7134370,             59,        1976,         0,      103,             4502,              -15,            6631,

