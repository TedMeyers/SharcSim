//
// SHARC Future Heavy Payload Control Program 2017
//
// Required library: Pololu AStar32U4 Library
// https://github.com/pololu/a-star-32u4-arduino-library
//
// Enable various components ('NO_' in front means disabled)
#define NO_SERIAL_WAIT
#define ENABLE_BUZZER
#define ENABLE_SERVO
#define ENABLE_GPS
#define NO_ENABLE_GYRO
#define ENABLE_IMU
#define ENABLE_ALT
#define ENABLE_SD
#define ENABLE_SERIAL_OUT
#define LOG_STATUS_CHANGE
#define LOG_ALL_SERIAL_OUT false
#define SERIAL_STAT_SKIP 3

#ifdef ENABLE_SERIAL_OUT
  #define NO_USB_SERIAL_OUT
  #ifdef USB_SERIAL_OUT
    #define SERIAL_OUT Serial
  #else
    #define SERIAL_OUT Serial1
  #endif
#endif

#ifdef ENABLE_SERVO
  #define SERVO_CLOSED 28
  #define SERVO_OPEN 100
#endif

#ifdef ENABLE_SD
  #define CHIP_SELECT 4
  #define LOG_FILE "log_4.txt"
  #define SD_LOG_STATUS  
  #ifdef ENABLE_IMU
    #define SD_LOG_ACC
    #define SD_LOG_CMP
  #endif
  #ifdef ENABLE_ALT
    #define SD_LOG_ALT
    #define SD_READ_TEMP
  #endif
  #ifdef ENABLE_GYRO
    #define X_SD_LOG_GYRO
  #endif
#endif

#ifdef ENABLE_GPS
  #define SERIAL_GPS Serial1
  #define MAX_BUF 128
#endif

// ---------------------------------
// TUNING PARAMETERS
// SHould lower apogee alt thresh to maybe 5?
#define ACCEL_CNT_THRESH 4
#define LAUNCH_ACCEL_THRESH 4000
#define FREE_FALL_THRESH 200
#define DOWN_CNT_THRESH 4
#define LOAD_COUNT_THRESH 20
#define FREE_FALL_CNT_THRESH 4
#define APOGEE_ALT_THRESH 50
#define DEPLOY_MIN_ALT_THRESH 6500
#define PAYLOAD_A_ALT_THRESH 6000
#define PAYLOAD_B_ALT_THRESH 5000
#define COMPLETE_ALT_THRESH 100
#define TEMP_READ_PERIOD 1023
#define DOWN_COS_ANG_THRESH 0.60
// ---------------------------------

// ---------------------------------
// UPDATE PERIODS in milliseconds
#define SLOW_STAT_LOG_PERIOD 1003
#define SLOW_GYRO_LOG_PERIOD 1602
#define SLOW_IMUD_LOG_PERIOD 2005
#define SLOW_PRES_LOG_PERIOD 2540

#define FAST_STAT_LOG_PERIOD 485
#define FAST_GYRO_LOG_PERIOD 113
#define FAST_IMUD_LOG_PERIOD 302
#define FAST_PRES_LOG_PERIOD 510

#define PRESSURE_READ_PERIOD 500
// ---------------------------------

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#ifdef ENABLE_IMU
  #include <L3G.h>
  #include <LSM303.h>
#endif
#ifdef ENABLE_SERVO
  #include <Servo.h>
#endif
#ifdef ENABLE_BUZZER
  #include <AStar32U4.h>
#endif
#ifdef ENABLE_ALT
  #include "SparkFunMPL3115A2.h"
#endif

// State Machine States
// -------------------------------------------------
// Loaded state is when the payload is first loaded
// Ready is when the rocket has been raised
// Launch is takeoff
// Coast is burnout coasting
// Apogee is when the rocket has reached its highes
// Deploy is when the main payload tube is released
// Eject A/B is when we release the sub-payloads
// Complete is when the main payload has landed
#define STATE_UNKNOWN 0
#define STATE_LOADED  1
#define STATE_READY   2
#define STATE_LAUNCH  3
#define STATE_COAST   4
#define STATE_APOGEE  5
#define STATE_DEPLOY  6
#define STATE_EJECT_A 7
#define STATE_EJECT_B 8
#define STATE_COMPLETE 9

// Servo Pin assignments
#define SERVO_A 22
#define SERVO_B 21

// Message type identifiers
#define SYS_MSG_TYP ((char *)"$SYMSG,")
#define SYS_ERR_TYP ((char *)"$SYERR,")
#define STATUS_MSG_TYP ((char *)"$SYSTA,")
#define UPDATE_MSG_TYP ((char *)"$SYUPD,")
#define BREAK_MSG ((char *)",-----------")

#ifdef SD_LOG_STATUS
  long statusTime = 0;
  int statusLogPeriod = 0;
#endif

#ifdef ENABLE_GYRO
  #define GYRO_MSG_TYP ((char *)"$IMGYR,")
  L3G gyro;
  int gyroStatus = 0; 
  long gyroTime = 0;
  int gyroLogPeriod = 0;
#endif

#ifdef ENABLE_IMU
  #define ACC_MSG_TYP ((char *)"$IMACC,")
  #define CMP_MSG_TYP ((char *)"$IMCMP,")
  LSM303 imu;
  long imuTime = 0;
  int imuLogPeriod = 0;
#endif

#ifdef ENABLE_ALT
  #define ALT_MSG_TYP ((char *)"$BPDAT,")
  MPL3115A2 pressure;
  
  long pressureTime = 0;
  long pressureReadTime = 0;
  int pressureReadPeriod = PRESSURE_READ_PERIOD;
  int pressureLogPeriod = 0;
  double pressureRawAltitude = 0.0;
  #ifdef SD_READ_TEMP
    long tempTime = 0;
  #endif
#endif

#ifdef ENABLE_GPS
  char buf[MAX_BUF];
  int buf_idx = 0;
#endif

#ifdef ENABLE_SERVO
  Servo servoA;
  Servo servoB;
#endif

#ifdef ENABLE_BUZZER
   AStar32U4Buzzer buzzer;
#endif

#ifdef ENABLE_SD
  // File for SD card logging
  File file;
#endif

int serialLogCnt = 0;
int logSerialOut = LOG_ALL_SERIAL_OUT;

// State Machine values
int curState = STATE_UNKNOWN;
long stateChangeTime_ms = 0;
long takeoffTime_ms = 0;
long startTime_ms = 0;
long updateTime_ms = 0;
int updatePeriod_ms = 50;
int is_a_ejected = false;
int is_b_ejected = false;

// Accel values
double curAx = 0.0;
double curAy = 0.0;
double curAz = 0.0;
double dwnAx = 0.0;
double dwnAy = 0.0;
double dwnAz = 0.0;
double curAccelMag = 0;
double cosAngle = 0.0;

int curDownCount = 0;
int curLoadCount = 0;
int curFreeFallCount = 0;
int curAccelCount = 0;

// Temperature and altitude
int curTemp = 0;
float curAltitude = 0.0;
long minAltitude = 10000;
long maxAltitude = 0;

// -------------------------------------------------------------
void setup() {
  Wire.begin();        // Join i2c bus

  #ifdef ENABLE_SERIAL_OUT
    #ifdef USB_SERIAL_OUT
      SERIAL_OUT.begin(115200);
    #endif
  #endif
  #ifdef ENABLE_GPS
    SERIAL_GPS.begin(9600);
  #endif
  #ifdef ENABLE_SERVO
    // Attaching then deteaching to save power
    attachServos();
    closeServos();
    detachServos();
  #endif

  #ifdef SERIAL_WAIT
    #ifdef USB_SERIAL_OUT
      # while (!SERIAL_OUT); // wait for serial port to connect. Needed for testing only
    #endif
  #endif

  #ifdef ENABLE_BUZZER
     playNotes(6, NOTE_C(4), 50);
  #endif
  
  logFileMsgLine(SYS_MSG_TYP, (char *)"INIT");
  #ifdef ENABLE_SD
    if (!SD.begin(CHIP_SELECT)) {
      logFileMsgLine(SYS_ERR_TYP, (char *)"SD_INIT");
      #ifdef ENABLE_BUZZER
        playNotes(3, NOTE_E(4), 200);
      #endif
    } else {
      file = SD.open(LOG_FILE, FILE_WRITE);
      if (!file) {
        logFileMsgLine(SYS_ERR_TYP, (char *)"SD_OPEN");
        #ifdef ENABLE_BUZZER
          playNotes(3, NOTE_A(4), 400);
        #endif
      }
    }
  #else
    #ifdef ENABLE_BUZZER
      playNotes(6, NOTE_C(4), 50);
    #endif
  #endif

  startTime_ms = millis();
  updateTime_ms = startTime_ms;

  #ifdef SD_LOG_STATUS
    statusTime = startTime_ms;
  #endif
  #ifdef ENABLE_ALT
    pressure.begin(); // Get sensor online  
    pressure.setModeAltimeter();   // Measure altitude above sea level in meters
    pressure.setOversampleRate(7); // Set Oversample to the recommended 128
    pressure.enableEventFlags();   // Enable all three pressure and temp event flags 
    pressureTime = startTime_ms;
    pressureReadTime = startTime_ms;
  #endif
  #ifdef ENABLE_IMU
    imu.init();
    imu.enableDefault();
    imu.writeReg(imu.CTRL2, 0x20); // Set 16g acc mode
    imuTime = startTime_ms;
  #endif
  #ifdef ENABLE_GYRO
    if (!gyro.init()) {
      logFileMsgLine(SYS_ERR_TYP, (char *)"GYRO");
      gyroStatus = 0;
    } else {
      gyroStatus = 1;
      gyro.enableDefault();
    }
    gyroTime = startTime_ms;
  #endif

  delay(5000); // startup delay to let everything have time to initialize

  logSerialOut = true;
  logFileMsgLine(SYS_MSG_TYP, (char *)"START");
  logSerialOut = LOG_ALL_SERIAL_OUT;
  
  #ifdef ENABLE_BUZZER
    playNotes(3, NOTE_C(4), 50);
  #endif

  #ifdef ENABLE_ALT
    curAltitude = pressure.readAltitudeFt();
    pressureRawAltitude = curAltitude;
  #endif
}
// -------------------------------------------------------------

// -------------------------------------------------------------
// Loop: various timers for reading sensors and logging
void loop() {
  long curTime_ms = millis();

  if ((curTime_ms - updateTime_ms) > updatePeriod_ms) {  // 20 Hz
    updateTime_ms = curTime_ms;
    updateStatus();  // Main accel and alt updata & filtering
    updateState();   // Update the state machine
  }

  #if defined(ENABLE_GPS)
    if (buf_idx == -1) {  // Only log '$GPR' messages (short 'recommended' info)
      if ((buf[1] == 'G') && (buf[3] == 'R')) logFileMsgLine(NULL, buf);
      buf_idx = 0;
    }
  #endif
  #if defined(SD_LOG_STATUS)
    curTime_ms = millis();
    if ((statusLogPeriod > 0) && ((curTime_ms - statusTime) > statusLogPeriod)) {
      statusTime = curTime_ms;
      logStatus();
    }
  #endif
  #if defined(ENABLE_ALT)
    #ifdef SD_LOG_ALT
      curTime_ms = millis();
      if ((pressureLogPeriod > 0) && ((curTime_ms - pressureTime) > pressureLogPeriod)) {
        pressureTime = curTime_ms;
        logAlt();
      }
    #endif
    #ifdef SD_READ_TEMP
      curTime_ms = millis();
      if ((curTime_ms - tempTime) > TEMP_READ_PERIOD) {
        tempTime = curTime_ms;
        curTemp = (int) pressure.readTempF();
      }
    #endif
  #endif
  #if defined(ENABLE_GYRO) && defined(SD_LOG_GYRO)
    curTime_ms = millis();
    if ((gyroLogPeriod > 0) && ((curTime_ms - gyroTime) > gyroLogPeriod)) {
      gyroTime = curTime_ms;
      if (gyroStatus) {
        gyro.read();
        logGyro();
      }
    }
  #endif
  #if defined(ENABLE_IMU)
    #if defined(SD_LOG_ACC) || defined(SD_LOG_CMP)
      curTime_ms = millis();
      if ((imuLogPeriod > 0) && ((curTime_ms - imuTime) > imuLogPeriod)) {
        imuTime = curTime_ms;
        logIMU();
      }
    #endif
  #endif
}
// -------------------------------------------------------------

// -------------------------------------------------------------
// Log and sound buzzer on state changes
void statusChange(int nextState) {
  if (curState != nextState) {
    stateChangeTime_ms = millis();
    #ifdef ENABLE_BUZZER
      if (nextState > 0) {
        if (nextState < STATE_DEPLOY) {
          playNotes(1, NOTE_C(5), 200);
          playNotes(1, NOTE_C(nextState+2), 100);
        } else {
          playNotes(1, NOTE_C(5), 200);
          playNotes(1, NOTE_C(5), 200);
          playNotes(1, NOTE_C(nextState-3), 100);          
        }
      }
    #endif
    #ifdef LOG_STATUS_CHANGE
      logStatusChange(nextState);
    #endif
    curState = nextState;
  }
}

// -------------------------------------------------------------
// Update sensor values - accellerometer and altitude filtering
void updateStatus() {
  #ifdef ENABLE_IMU
    imu.read();
    long x = imu.a.x;
    long y = imu.a.y;
    long z = imu.a.z;

    double am = ((x*x) + (y*y) + (z*z))/1000.0;    
    curAccelMag = (0.80*curAccelMag) + (0.20*am);

    if (curAccelMag > LAUNCH_ACCEL_THRESH) {
      curAccelCount++;
    } else {
      curAccelCount = 0;
    }
    if (curAccelMag < FREE_FALL_THRESH) {
      curFreeFallCount++;
    } else {
      curFreeFallCount = 0;
    }
  #endif 
  
  #ifdef ENABLE_ALT
    long curTime_ms = millis();
    if ((curTime_ms - pressureReadTime) > pressureReadPeriod) {
      pressureReadTime = curTime_ms;
      pressureRawAltitude = pressure.readAltitudeFt();
      curAltitude = (0.60*curAltitude) + (0.40*pressureRawAltitude);
      
      long curAlt = (long) curAltitude;
      if (curState < STATE_LAUNCH) {  // Only check min if not launched yet
        if (curAlt < minAltitude) minAltitude = curAlt;
      }
      if (curAlt > maxAltitude) maxAltitude = curAlt;
    }
  #endif 
}

// -------------------------------------------------------------
// Check the angle for detecting rocket raising to launch position
int checkDownAngle() {
  cosAngle = calcCosAng(curAx, curAy, curAz, dwnAx, dwnAy, dwnAz);
  if (cosAngle < DOWN_COS_ANG_THRESH) {
    curDownCount++;
  } else {
    curDownCount = 0;
  }
  return (curDownCount > DOWN_CNT_THRESH);
}

// Use the dot product to calculate the cosine of the angle of the two vectors
double calcCosAng(double x1, double y1, double z1, double x2, double y2, double z2) {
  double dp = (x1*x2) + (y1*y2) + (z1*z2);
  double m1 = sqrt(x1*x1 + y1*y1 + z1*z1);
  double m2 = sqrt(x2*x2 + y2*y2 + z2*z2);
  return (dp/(m1*m2));
}


// Update the filtered Accel vector
void updateAccel() {
  curAx = (0.80*curAx) + 0.20*((float)imu.a.x);
  curAy = (0.80*curAy) + 0.20*((float)imu.a.y);
  curAz = (0.80*curAz) + 0.20*((float)imu.a.z);
}

// -------------------------------------------------------------
// Update the state machine
void updateState() {
  long curAlt = (long) curAltitude;
  long curTime_ms = millis();
  long t_time_ms = get_t_time_ms();
  int stateDur = (curTime_ms - stateChangeTime_ms)/1000;
  int cur_alt_agl = (curAlt-minAltitude);
  
  if (curState < STATE_LOADED) {
    updateAccel();
    curLoadCount++;
    if (curLoadCount > LOAD_COUNT_THRESH) {
      statusChange(STATE_LOADED);
      setSlowLogging();
      minAltitude = curAltitude;
      dwnAx = curAx;
      dwnAy = curAy;
      dwnAz = curAz;
    }
  } else if (curState == STATE_LOADED) {
    updateAccel();
    if (checkDownAngle()) {
      statusChange(STATE_READY);
      minAltitude = curAltitude - 40;

      #ifdef ENABLE_SERVO
         attachServos();
         closeServos();
      #endif
    }
  }

  // Detect launch regardless for any state less before launch state
  if (curState <= STATE_READY) {
    if (curAccelCount > ACCEL_CNT_THRESH) {
      setFastLogging();
      statusChange(STATE_LAUNCH);
      takeoffTime_ms = curTime_ms;
    }
  } else if (curState == STATE_LAUNCH) {
    //                                               Should be 10000 ms!
    if (((curFreeFallCount<FREE_FALL_CNT_THRESH) && (t_time_ms>10)) || (t_time_ms>25000)) {
      statusChange(STATE_COAST);     
    }
  } else if (curState == STATE_COAST) {
    if (((maxAltitude - curAlt) > APOGEE_ALT_THRESH)             || (t_time_ms > 40000)) {
      statusChange(STATE_APOGEE);      
    }
  } else if (curState == STATE_APOGEE) {
    if ((cur_alt_agl < DEPLOY_MIN_ALT_THRESH)                    || (t_time_ms > 55000)) {
      statusChange(STATE_DEPLOY);       
    }
  } else if (curState >= STATE_DEPLOY) {
    if (!is_a_ejected && ((cur_alt_agl < PAYLOAD_A_ALT_THRESH)   || (t_time_ms > 120000))) {
      is_a_ejected = true;
      statusChange(STATE_EJECT_A); 
      openServo(SERVO_A);
    }
    if (!is_b_ejected && ((cur_alt_agl < PAYLOAD_B_ALT_THRESH)   || (t_time_ms > 180000))) {
      is_b_ejected = true;
      statusChange(STATE_EJECT_B);     
      openServo(SERVO_B);
    }
    if ((cur_alt_agl < COMPLETE_ALT_THRESH)                      || (t_time_ms > 240000)) {
      setSlowLogging();
      statusChange(STATE_COMPLETE);
      detachServos();  // Power down servos just in case
    }
  }
}

// -------------------------------------------------------------
// Time since takeoff
long get_t_time_ms() {
  return (takeoffTime_ms > 0) ? (millis()-takeoffTime_ms):0;
}

// Log message for status change events
#ifdef LOG_STATUS_CHANGE
  void logStatusChange(int nextState) {
    logFileMsgLine(SYS_MSG_TYP, BREAK_MSG);
    
    logSerialOut = true;
    logFileMsg(UPDATE_MSG_TYP, NULL);
    logFileInt(nextState);
    logFileInt(get_t_time_ms());
    logFileInt((stateChangeTime_ms - startTime_ms));
    logFileLine();
    logSerialOut = LOG_ALL_SERIAL_OUT;    
    
    logFileMsgLine(SYS_MSG_TYP, BREAK_MSG);  
  }
#endif

// Log current status
#ifdef SD_LOG_STATUS
  void logStatus() {
    long curTime_ms = millis();
    long t_time_ms  = get_t_time_ms();

    serialLogCnt++;
    if ((serialLogCnt % SERIAL_STAT_SKIP) == 0) logSerialOut = true;
    logFileMsg(STATUS_MSG_TYP, NULL);
    logFileInt(curState);
    logFileInt(t_time_ms);
    logFileInt((curTime_ms - startTime_ms));
    logFileInt(cosAngle*100);
    logFileInt(curAccelMag);
    logFileInt(curAccelCount);
    logFileInt(curTemp);
    logFileInt(minAltitude);
    logFileInt(curAltitude-minAltitude);
    logFileInt(maxAltitude-minAltitude);
    logFileLine();
    logSerialOut = LOG_ALL_SERIAL_OUT;
  }
#endif

// Log raw altitude data
#if defined(ENABLE_ALT) && defined(SD_LOG_ALT)
  void logAlt() {
    logFileMsg(ALT_MSG_TYP, NULL);
    logFileInt(millis() - startTime_ms);
    logFileInt((long)pressureRawAltitude);
    logFileInt(curTemp);
    logFileLine();
  }
#endif

// Log raw Gyro data
#if  defined(ENABLE_GYRO) && defined(SD_LOG_GYRO)
  void logGyro() {
    logFileMsg(GYRO_MSG_TYP, NULL);
    logFileInt((long)gyro.g.x);
    logFileInt((long)gyro.g.y);
    logFileInt((long)gyro.g.z);
    logFileLine();
  }
#endif

// Log raw accelerometer and compass data
#ifdef ENABLE_IMU
  #if (defined(SD_LOG_ACC) || defined(SD_LOG_CMP))
    void logIMU() {
      #ifdef SD_LOG_ACC
        logFileMsg(ACC_MSG_TYP, NULL);
        logFileInt(millis() - startTime_ms);
        logFileInt(imu.a.x);
        logFileInt(imu.a.y);
        logFileInt(imu.a.z);
      #endif
      #ifdef SD_LOG_CMP
        logFileMsg(CMP_MSG_TYP, NULL);
        logFileInt(imu.m.x);
        logFileInt(imu.m.y);
        logFileInt(imu.m.z);
      #endif
      logFileLine();
    }
  #endif
#endif

// Log an integer (actually a long)
void logFileInt(long i) {
  #ifdef ENABLE_SERIAL_OUT
    if (SERIAL_OUT && logSerialOut) {
      SERIAL_OUT.print(i);
      SERIAL_OUT.print(',');
    }
  #endif
  #ifdef ENABLE_SD
    if (file) {
      file.print(i);
      file.print(',');
    }
  #endif
}

// Log the message type and a string
// to serial port and sd file
void logFileMsg(char *typ, char *msg) {
  #ifdef ENABLE_SERIAL_OUT
    if (SERIAL_OUT && logSerialOut) {
      if (typ != NULL) SERIAL_OUT.print(typ);
      if (msg != NULL) SERIAL_OUT.print(msg);
    }
  #endif
  #ifdef ENABLE_SD
    if (file) {
      if (typ != NULL) file.print(typ);
      if (msg != NULL) file.print(msg);
    }
  #endif
}

// Log the message type and a string with a newline
// to serial port and sd file
void logFileMsgLine(char *typ, char *msg) {
  logFileMsg(typ, msg);
  logFileLine();
}

// Log a newline (to serial/sd file)
void logFileLine() {
  #ifdef ENABLE_SERIAL_OUT
    if (SERIAL_OUT && logSerialOut) SERIAL_OUT.println();
  #endif
  #ifdef ENABLE_SD
    if (file) { file.println(); file.flush(); }
  #endif
}

#ifdef ENABLE_BUZZER
  void playNotes(int num, int note, int ms) {
    for (int i=0; i<num; i++) {
      buzzer.playNote(note, ms, 15);
      delay(ms + 50);
    }
  }
#endif

#ifdef ENABLE_SERVO
  void attachServos() {
    servoA.attach(SERVO_A);
    servoB.attach(SERVO_B);
  }
  void detachServos() {
    servoA.detach();
    servoB.detach();  
  }

  void openServo(int servoNum) {
    if (servoNum == SERVO_A) {
      if (servoA.read() != SERVO_OPEN) {
        logFileMsgLine(SYS_MSG_TYP, (char *)"OPEN_A");
        if (!servoA.attached()) attachServos();
        servoA.write(SERVO_OPEN);
      }
    } else if (servoNum == SERVO_B) {
      if (servoB.read() != SERVO_OPEN) {
        logFileMsgLine(SYS_MSG_TYP, (char *)"OPEN_B");
        if (!servoB.attached()) attachServos();
        servoB.write(SERVO_OPEN);
      }
    }
  }
  void closeServos() {
    servoA.write(SERVO_CLOSED);
    servoB.write(SERVO_CLOSED);
  }
#endif

// GPS string processing
#ifdef ENABLE_GPS
  void serialEvent1() {
    char c;
    while (SERIAL_GPS.available() > 0) {
      if (buf_idx == -1) return;
  
      c = SERIAL_GPS.read();
      if ((c == 10) || (buf_idx > (MAX_BUF-2))) {
        buf_idx = -1;
      } else if (c > 31) {
        buf[buf_idx++] = c;
        buf[buf_idx] = 0;
      }
    }
  }
#endif


void setFastLogging() {  
  #ifdef SD_LOG_STATUS
    statusLogPeriod = FAST_STAT_LOG_PERIOD;
  #endif
  #ifdef ENABLE_GYRO
    gyroLogPeriod = FAST_GYRO_LOG_PERIOD;
  #endif
  #ifdef ENABLE_IMU
    imuLogPeriod = FAST_IMUD_LOG_PERIOD;
  #endif
  #ifdef ENABLE_ALT
    pressureLogPeriod = FAST_PRES_LOG_PERIOD;
  #endif
}

void setSlowLogging() {  
  #ifdef SD_LOG_STATUS
    statusLogPeriod = SLOW_STAT_LOG_PERIOD;
  #endif
  #ifdef ENABLE_GYRO
    gyroLogPeriod = SLOW_GYRO_LOG_PERIOD;
  #endif
  #ifdef ENABLE_IMU
    imuLogPeriod = SLOW_IMUD_LOG_PERIOD;
  #endif
  #ifdef ENABLE_ALT
    pressureLogPeriod = SLOW_PRES_LOG_PERIOD;
  #endif
}

