#ifndef UTILITIES
#define UTILITIES

#include <Arduino.h>
#include <TeensyStep.h>
#include <SD.h>
#include <SPI.h>


// --------------- global variables: ---------------

extern const int chipSelect;

extern StepControl *master_controllers[4];
extern StepControl *slave_controllers[2];
extern Stepper *master_steppers[4];
extern Stepper *slave_steppers[2];
extern File logfile;

extern const unsigned int
indicator, // light
  estop, // input from the e-stop.
  microsteps,
  log_time, // in ms, how often to report postitions.
  sd_write_time, // defines SD read/write operations, so don't make this too small
  motor_steps_per_rev; // how many steps per one motor revolution
extern int max_speed, // top speed
  acceleration ;  // acceleration = steps/sec/sec;

extern long avg_sens_widths[6];
extern int activeletter; // for testing purposes
extern String command; // logical control

extern long
whole_revs[6],
  positions[6],
  prev_positions[6],
  targets[6]; // to compare with positions[]// real build, measured manually
extern const long offsets[6]; // distance from sensor 0 to home position
extern const int revolutions[6]; // number of times to revolve per cycle
extern int
  period, // time in seconds for one cycle
  certainties[6],
  speeds[6],
  lpins[][5];
extern boolean
is_primary,
  debug,  // flag to turn on/off serial output
  searching,
  mockup,
  powertoggle, // indicator as to whether we've disabled the motors or not
  cyclestarted,
  positionreset,
  need_to_log,
  SDokay,
  force_log,
  log_pos, sd_log, already_written,
  slave_has_queried; // flags to note whether we've logged positions during moves

extern boolean flips[6];
extern float thetas[6];

extern const byte numChars;
extern char receivedChars[32];
extern char tempChars[32];        // temporary array for use when parsingdata
extern char HWmsg[32]; // parsed msg
extern int HWint1; // parsed int
extern int HWint2; // parsed int

extern boolean newData;





// ---------------  utility functions: ---------------

void hwsend(char c, int i, int j);

boolean sense(int pin);
void log_position();
void meta_calibrate(int letter);
time_t getTeensy3Time();
void printDigits(int digits);
void SD_read_positions();
void relativemove(int m, long tgt);
void absolutemove(int m, long tgt);
void startnewcycle();
void setspeed(float s);
void setaccel(int a);
void list(int m);
void customcycle(float a, float b, float c, float d, float e, float f);
void overwriteposition(int m, long pos);

void SD_read_positions();
void log_position();
void absolutemove(int m, long tgt);
void overwriteposition(int m, long pos);

void recvWithStartEndMarkers();
void parseData();
time_t getTeensy3Time();
void printDigits(int digits);



#endif
