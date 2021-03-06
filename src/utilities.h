#ifndef UTILITIES
#define UTILITIES

#include <Arduino.h>
#include <TeensyStep.h>
#include <SD.h>
#include <SPI.h>


// --------------- global variables: ---------------

extern const int chipSelect,
  start_hour, start_minute,
  end_hour, end_minute,
  reboot_hour, reboot_minute,
  cycles_before_calibrating, // how many cycles before re-calibration
  mockup_period, // time in seconds for one cycle
  build_period,
  interval_seconds,
  time_check_interval;

extern StepControl *master_controllers[4];
extern StepControl *slave_controllers[2];
extern Stepper *master_steppers[4];
extern Stepper *slave_steppers[2];

extern File logfile;

extern const unsigned int
indicator, // light
  estop, // input from the e-stop.
  mockup_microsteps,
  build_microsteps,
  log_time, // in ms, how often to report postitions.
  sd_write_time, // defines SD read/write operations, so don't make this too small
  motor_steps_per_rev; // how many steps per one motor revolution
extern int max_speed; // top speed
extern float acceleration;  // acceleration = steps/sec/sec;

extern long avg_sens_widths[6];
extern int activeletter, // for testing purposes
  calib_sum;
extern const char* command; // logical control

extern long
whole_revs[6],
  positions[6],
  prev_positions[6],
  targets[6]; // to compare with positions[]// real build, measured manually

extern const long offsets[6], // distance from sensor 0 to home position
  mockup_revs[], // measured manually on the mockup
  // build_revs[], hand measured
  mockup_sens_widths[],
  build_sens_widths[],
  mockup_speeds[],
  build_speeds[]
;

extern const float build_revs[];

extern const int revolutions[6]; // number of times to revolve per cycle

extern int
cycle_count,
  certainties[6],
  speeds[6],
  accelerations[6],
  lpins[][5];

extern bool
is_primary,
  debug,  // flag to turn on/off serial output
  force_off,
  loop_cycle,
  searching,
  mockup,
  powertoggle, // indicator as to whether we've disabled the motors or not
  cyclestarted,
  wasrunning,
  lunchtime,
  alreadyreset,
  fully_calibrated,
  timecheck,
  need_to_log,
  SDokay,
  force_log,
  log_pos, sd_log, already_written,
  slave_has_queried; // flags to note whether we've logged positions during moves

extern bool mockup_flips[6],
  build_flips[6],
  homed[6],
  should_be_on_zero[6],
  already_moved_off[6],
  done_calibrating[6];
extern float thetas[6];

extern const byte numChars;
extern char receivedChars[32];
extern char tempChars[32];        // temporary array for use when parsingdata
extern char HWmsg[32]; // parsed msg
extern int HWint1; // parsed int
extern int HWint2; // parsed int

extern bool newData;





// ---------------  utility functions: ---------------

int aprintf(char const * str, ...);
void hwsend(char c, int i, int j);
bool sense(int pin);
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
bool targetsreached();
bool proportional();
void verify(int letter);

void SD_read_positions();
void log_position();
void absolutemove(int m, long tgt);
void overwriteposition(int m, long pos);

void recvWithStartEndMarkers();
void parseData();
time_t getTeensy3Time();
void printDigits(int digits);

void home(int letter);
void startnewcycle();
void customcycle();

void syncmove(long a, long b);
void syncmove(long a, long b, long c, long d);


#endif
