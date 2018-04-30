/*
 * This is the Payload Status & Sensor Message Simulator
 * 
 * It is meant to run on a Pololu A-Star 32U4 Prime Board
 * (The A-Star Prime is an Arduino Leonardo compatible board
 * that includes a microSD card reader, a buzzer, and some
 * built in buttons).
 * 
 * If this board is not available, a standard Leonardo
 * (or Uno) with a SD shield could be used (with some minor
 * code modifications).
 *
 * Required library: Pololu AStar32U4 library
 * https://github.com/pololu/a-star-32u4-arduino-library
 * 
 * Usage:
 * Copy the message file to the SD card and name it "test.txt"
 * Edit this file and add the following to be the first line in 
 * the file (if it is not already the first line):
 *         $INIT1,6700672,6905874,
 *         
 * This tells it how many milliseconds into the file to start
 * the simulator (in this case about 6700 seconds -- there was
 * a lot of waiting around before the rocket was launched, and
 * we want to skip that waiting period).  And the second number
 * is the end time; in this case about 200 seconds after the start.
 * 
 * When the simulator starts, it will beep twice if it can
 * read the data file.  It will beep 6 times if no SD card,
 * and 4 times if it can't read the "test.txt" file.
 * 
 * The program then waits for the "B" button to be pressed.
 * 
 * After the button is pressed, it beeps once and starts
 * sending status messages over the Serial port in real
 * time.  It will also beep every time the state changes.
 * It will beep twice when the simulation is completed.
 * Note that for a large data file, it may take several
 * seconds to seek to the start of the simulation.
 * 
 * Also, status messages and other debugging info is echoed
 * to the USB serial port.
 * 
 * Serial data is sent at 115200 bps!
 * 
 * See below for the message format description
 * 


== microSD card considerations ==

You will need to install a jumper between GND and CS to enable
the microSD card.

(You will also need to have a version of the A-Star 32U4 with a
microSD card socket and you will need to insert a formatted
microSD card into the socket with the message file on it.)

= Buzzer =

You will need to install a jumper on JUMPER-6, to enable
the buzzer.

*/


// MESSAGE FORMAT
//---------------------------------------------------------
// Mission State values:
//   STATE_LOADED  1
//   STATE_READY   2
//   STATE_LAUNCH  3
//   STATE_COAST   4
//   STATE_APOGEE  5
//   STATE_DEPLOY  6
//   STATE_EJECT_A 7
//   STATE_EJECT_B 8
//   STATE_COMPLETE 9

// - rel time: The (relative) time since launch
// - time: The (absolute) time since start of recording
// - cos angle: The cosine of the angle above horizontal multiplied by 100
//           Only calculated in STATE_LOAED -- below 60 (about 53 degrees 
//           from horizontal -- horizontal is a cos angle of 99)
// - accel raw: The value from the sensor (x,y,z) squared (no units) 
//           1g is about 2000
// - accel count: The number of times in a row that accel is 
//           over the launch threshold
// - min alt msl: The altitude on the ground, above mean sea level, 
//           not corrected for current atmospheric conditions
// - cur alt agl: The current altitude above ground level (min alt msl)
// - max alt agl: The maximum altitude

// example data:
// MSG_TYP, state, rel time (ms), time (ms), cos ang (x100), accel_raw^2, accel cnt, temp (F), min alt msl (ft), cur alt agl (ft), max alt agl (ft)
//  $INIT1, 6700672,
//  $SYSTA,     1,             0,     12191,             99,        2013,         0,       87,             4376,                0,              13,
//  $SYSTA,     2,             0,   6701679,             59,        1815,         0,       94,             4502,               25,              45,
//  $SYSTA,     3,           436,   6705619,             59,       39936,         9,       94,             4502,               25,              45,
//  $SYSTA,     4,         13431,   6718614,             59,         101,         0,       92,             4502,             4231,            4231,
//  $SYSTA,     5,         29057,   6734240,             59,         529,         0,       91,             4502,             6571,            6631,
//  $SYSTA,     6,         32199,   6737382,             59,         402,         0,       91,             4502,             6341,            6631,
//  $SYSTA,     7,         41413,   6746596,             59,        1496,         0,       92,             4502,             5168,            6631,
//  $SYSTA,     8,        157947,   6863130,             59,        1874,         0,       89,             4502,             1017,            6631,
//  $SYSTA,     9,        429187,   7134370,             59,        1976,         0,      103,             4502,              -15,            6631,


#include <AStar32U4.h>
#include <SPI.h>
#include <SD.h>

#define SERIAL_BPS 115200
#define DATA_FNAME "test.txt"

const uint16_t LINE_SZ = 256;
const uint8_t CS = 4;    // Chip Select pin

boolean is_init = false;
boolean is_to_start = false;
uint32_t start_time_msec = 0;
uint32_t end_time_msec = 0;
uint32_t begin_msec = 0;
uint32_t sim_msec = 0;
uint32_t cur_msec = 0;
uint32_t check_msec = 0;
int last_state = 0;


AStar32U4Buzzer buzzer;
AStar32U4ButtonB buttonB;

void setup() {
  // Initialize the SD card.
  Serial.begin(SERIAL_BPS);
  Serial1.begin(SERIAL_BPS);
  
  //while (!Serial);  // wait for usb
  delay(2000);
  
  Serial.println("SHARC Payload Status & Sensor Data Simulator (ver 1.0, Apr 2018)");
  
  init_SD();

  runSim(); // Simulator is run here!
}

void loop() {
  // Do nothing, simulation is complete!
}

// Buzz the given note, cnt times
void buzz(int note, int cnt) {
  for (int i=0; i<cnt; i++) {
    buzzer.playNote(note, 100, 15);      
    delay(200);
  }
}

void init_SD() {
  if (!SD.begin(CS)) {
    Serial.println("Card failed, or not present");
    buzz(NOTE_C(3), 6);
    while(1){}  // done
  }  
}

File openFile(char *fname) {
  File file = SD.open(fname);
  if (!file) {
    Serial.println("error opening test.txt");
    buzz(NOTE_C(3), 4);
    while(1){}  // done
  }
  return file; 
}

void runSim() {
  is_init = false;
  is_to_start = false;
  start_time_msec = 0;
  end_time_msec = 0;
  begin_msec = 0;
  cur_msec = 0;
  check_msec = 0;
  sim_msec = 0;
  last_state = 0;
 
  File file = openFile(DATA_FNAME);

  Serial.println("Press 'B' button to start");
  buzz(NOTE_C(5), 2);
  while (!buttonB.isPressed());
  buzz(NOTE_C(5), 1);
  
  readFile(file);   // All the action is in here!
  file.close();
  
  buzz(NOTE_C(5), 2);
  Serial.println();
  Serial.println();
}


// Read the file and process the lines in it.
void readFile(File file) {  
  char line[LINE_SZ];
  uint32_t lineNumber = 1;
  uint32_t i = 0;

  Serial.println("Loading simulation data file...");

  while (file.available()) {
    char x = file.read();
    
    if (x == '\n' || x == '\r') { // Look for the end of line
      line[i] = 0;            // Terminate the line with a null
      if (i > 0) {
        if (!is_init) {
          processInit(line);  // Read the initialization data
        } else {
          processLine(line);  // Process the line
          lineNumber++;
        }
        i = 0;                // Reset the line position
      }
    } else {
      if (!isprint(x)) x = ' ';  // blank out non-print chars
      line[i++] = x;             // add the char to the line
    }
  }
  Serial.print("Simulation complete, processed ");
  Serial.print(lineNumber); Serial.println(" lines");
}

// Process a system status message
void processLine(char *line) {
  char mt[16];      // Message Type
  int st;           // Mission State
  uint32_t t_tm;    // Time since takeoff
  uint32_t a_tm;    // Time since startup
  int c_ang;        // Cosine angle
  uint32_t acc;     // Absolute raw acceleration (squared)
  int a_cnt;        // Acceleration threshold count
  int temp;         // Temperature (F)
  int min_a;        // Minimum altitude
  int cur_a;        // Current altitude
  int max_a;        // Maximum altitude
  int t;            // test value -- not used
  int p = sscanf(line, "$%5s,%d,%ld,%ld,%d,%ld,%d,%d,%d,%d,%d,%d,", mt, &st, &t_tm, &a_tm, &c_ang, &acc, &a_cnt, &temp, &min_a, &cur_a, &max_a, &t);
  
  if (p == 11 && strcmp(mt, "SYSTA") == 0) {
    if (!is_to_start && a_tm >= start_time_msec) {
      Serial.println("Starting simulation");
      is_to_start = true;
    }
    if (is_to_start && a_tm <= end_time_msec) {
      sim_msec = a_tm;
      if (last_state != 0 && last_state != st) {
         buzz(NOTE_C(6), 1);
      }
      last_state = st;
      
      if (begin_msec == 0) {
        begin_msec =  millis();
      }
      while (check_msec < sim_msec) {
        delay(1);
        check_msec = start_time_msec + (millis()-begin_msec);
      }            
      Serial1.println(line);
      Serial.println(line);        
    } 
  }
}

void processInit(char *line) {
  char mt[16];      // Message Type
  uint32_t st;      // start time
  uint32_t et;      // end time
  int t;            // test value -- not used
  int p = sscanf(line, "$%5s,%ld,%ld,%d,", mt, &st, &et, &t);
  
  if (p == 3 && strcmp(mt, "INIT1") == 0) {
    is_init = true;
    start_time_msec = st;
    end_time_msec = et;
    Serial.print("Found INIT1 message: ");
    Serial.print(start_time_msec);
    Serial.print(" to ");
    Serial.println(end_time_msec);
    Serial.println("Searching for start of simulation...");
  }
}

