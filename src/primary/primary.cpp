#include <Arduino.h>
#include <TeensyStep.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
#include "utilities.h"
#include "calibrate2.h"
// #include "calibrate1.h"

/*
  last time set:
  111637587512
  Monday, November 22, 2021 1:25:12 PM
 */

#define HWSERIAL Serial1 // define hardware serial driver (connection to 2nd teensy)

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)

bool is_primary = true; // used in shared functions to determine which teensy is asking

const int chipSelect = BUILTIN_SDCARD; // use built-in SD card slot for data logging

StepControl *master_controllers[4] = {NULL, NULL, NULL, NULL}; // controller objects -- one per stepper, to avoid Bresenham. first four steppers here, last two on slave.
StepControl *slave_controllers[2] = { NULL, NULL }; // included here so that the two can share functions

File logfile; // where to store the position logs

time_t last_cycle_end_time = 0; // timestamp of last cycle's finish

const unsigned int indicator = LED_BUILTIN, // light
  estop = 17, // input from the e-stop.
  log_time = 250, // in ms, how often to report postitions.
  sd_write_time = 1000, // governs SD read/write operations, so don't make this too small
  motor_steps_per_rev = 400; // how many steps per one motor revolution
float acceleration = 0.5;  // acceleration. multiplier; each will be set to speeds[i] * acceleration

long avg_sens_widths[]; // rough number of steps it takes to traverse a sensor. defined in setup.

int cycle_count = 0, // running cycle tally
  activeletter = 4; // for testing purposes
const char* command = ""; // logical control


long whole_revs[6], // steps in a whole revolution
  positions[6], // current position
  prev_positions[6], // last position
  targets[6]; // to compare with positions[]

int speeds[6], // speeds of each individual ring
  accelerations[6]; // accelerations of each ring

int certainties[] = { 0, 0, 0, 0, 0, 0 }; // do we know where we are? 0 = no, 1 = yes, 2 = ish (in the quadrant with no sensors)
bool searching = false;
float thetas[6]; // rotational position (0-1)

const unsigned int txtransfergap = 50; // how often to sync with 2nd teensy (in ms)
bool shouldtxtransfer = true; // flag used to avoid repeat xfers

/* TEENSY */
int lpins[][5] = { // letter pins. two dimensional array of [letters][pins].

  // 0      1   2     3      4
  // step, dir, en, sens1, sens2
  {2, 3, 4, 33, 34}, // 0: G1
  {5, 6, 7, 35, 36}, // 1: O1
  {8, 9, 10, 37, 38}, // 2: 02
  {11, 12, 24, 39, 14}, // 3: G2
  {25, 26, 27, 15, 0},  // 4: L -- note that these aren't used here
  {28, 29, 30, 16, 0}  // 5: E -- ditto (they're on the secondary board)
  // keeping them in here makes the math of accessing each letter simpler

};

bool
powertoggle = true, // indicator as to whether we've disabled the motors or not
  need_to_log = true, // flag used in log timing
  SDokay = true, // sd card status
  force_log = false, // used for certain logging overrides
  log_pos = true, sd_log = true, already_written = false; // flags to note whether we've logged positions during moves

// initiate stepper array (will be populated in setup)
Stepper *master_steppers[4] = { NULL, NULL, NULL, NULL };
Stepper *slave_steppers[2] = { NULL, NULL }; // see above, unused here

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary arrays used in parsing serial i/o

// variables to hold the parsed data
char HWmsg[numChars] = {0};
int HWint1 = 0;
int HWint2 = 0;

bool newData = false;

//============


void secondary_setup() {
  // set of hw serial commands to initiate the secondary teensy.
  // this is the response handshake (secondary sends a "g" when it's ready for this)
  if (debug) Serial.println("sending lengths + speeds");
  hwsend('!', 0, 0); // begin preflight, reset counters to 0
  hwsend('I', avg_sens_widths[4], avg_sens_widths[5]); // send sensor widths
  hwsend('U', speeds[4], speeds[5]); // send speeds
  hwsend('P', positions[4], positions[5]); // send positions
  hwsend('Y', accelerations[4], accelerations[5]); // send accelerations
  hwsend('O', whole_revs[4], whole_revs[5]); // send whole_rev measurements
}

void dealwithit() {
  // called for every incoming serial event
  // the way it's set up, every serial event must be in the format [string int int],
  // these are stored as [HWmsg HWint1 HWint2]

  if (strcmp(HWmsg, "g") == 0) {
    // secondary teensy is asking for setup
    secondary_setup();
  }

  else if (strcmp(HWmsg, "a") == 0) {
    // answer from secondary teensy re: position of specific motor
    positions[HWint1 + 4] = HWint2; // store positions
    log_position(); // update log
  }

  else if (strcmp(HWmsg, "d") == 0) {
    // secondary letter is done calibrating
    done_calibrating[HWint1+4] = true;
    if (debug) aprintf("letter %d done calibrating.\n", HWint1+4);
  }

  else if (strcmp(HWmsg, "C") == 0) {
    // query from slave; send positions, then calibration greenlight
    if (debug) Serial.print("calibration request for slave letter ");
    if (debug) Serial.println(HWint1);
    hwsend('w', HWint1, positions[HWint1 + 4]); // send position
    hwsend('C', HWint1, 0); // green light signal for calibration
  }
}

void check_hwserial() {
  // utility function to parse serial data
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    dealwithit();
    newData = false;
  }
}
void calibrate_all() {
  // sequentially calibrate each letter

  fully_calibrated = false; // flag to indicate we're not ready
  for (int i = 0; i < 6; i++) { // set each individual "done" flag false to start
    done_calibrating[i] = false;
  }

  if (debug) Serial.println("calibrating everything... ");
  for (int i = 0; i < 6; i++) { // for each letter
    if (i > 3) hwsend('c', i-4, 0); // if it's on the secondary teensy, request calibration
    else calibrate2(i); // otherwise calibrate it
  }

  while (!fully_calibrated) { // stay here until everything is calibrated
    check_hwserial(); // don't forget to check for secondary calibrations
    int sum = 0; // to track "doneness"
    for (int i = 0; i < 6; i++) {
      sum += done_calibrating[i]; // increase doneness
    }
    if (debug && millis() % 1000 < 100) aprintf("calibrations: [%d %d %d %d %d %d], sum = %d\n",
                       done_calibrating[0],
                       done_calibrating[1],
                       done_calibrating[2],
                       done_calibrating[3],
                       done_calibrating[4],
                       done_calibrating[5],
                       sum);
    if (sum == 6) fully_calibrated = true; // set flag when all are calibrated
  }

  // while loop exits once everything is calibrated
  if (debug) aprintf("all letters calibrated.\n");
}
void setup() {
  // use internal RTC for time
  setSyncProvider(getTeensy3Time);

  // begin hardware serial
  HWSERIAL.begin(115200);

  // initial logging
  if (debug) {
    Serial.begin(9600);

    while(!Serial && millis() < 4000); // wait for everything to settle
    Serial.println("=================== primary setup =====================");
    Serial.println("source file: " __FILE__ " compiled " __DATE__ " " __TIME__);
    Serial.println("=======================================================");


    if (timeStatus()!= timeSet) {
      Serial.println("Unable to sync with the RTC");
    } else {
      Serial.println("RTC has set the system time");
    }

    Serial.print("current time: ");
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year());
    Serial.println();
  }

  // sd init
  if (!SD.begin(chipSelect)) {
    if (debug) Serial.println("SD card initialization failed");
    SDokay = false;
  }
  else {
    if (debug) Serial.println("SD card initialization ok");
  }

  if (SDokay) SD_read_positions(); // load in last positions from SD card
  else if (!SDokay) {
    // start up serial in this case and hang here
    Serial.begin(9600);
    delay(1000);
    Serial.println("SD FAILURE, DOING NOTHING");
    while(true);
  }


  // initiate controllers + steppers
  for (int i = 0; i < 4; i++) {
    // new controller object in each of the 4 array positions (only 4 because the other 2 are on secondary)
    master_controllers[i] = new StepControl();
    master_steppers[i] = new Stepper(lpins[i][0], lpins[i][1]); // step and dir
  }

  // calculate speeds, accel, + sensor widths (use gsheet)
  if (debug) Serial.print("speeds / accelerations: ");
  for (int i = 0; i < 6; i++) {
    if (mockup) {
      whole_revs[i] = mockup_revs[i] * mockup_microsteps;
      avg_sens_widths[i] = mockup_sens_widths[i] * mockup_microsteps;
      speeds[i] = mockup_microsteps * mockup_revs[i] * revolutions[i]/mockup_period;
    }
    else {
      whole_revs[i] = build_revs[i] * build_microsteps;
      avg_sens_widths[i] = build_sens_widths[i] * build_microsteps;
      speeds[i] = build_microsteps * build_revs[i] * revolutions[i]/build_period; // preset speeds for each ring based on revolutions + period
    }
    accelerations[i] = speeds[i] * acceleration; // multiply acceleration

    if (debug) aprintf("[%d/%d] ",
                       speeds[i],
                       accelerations[i]);

    if (i < 4) { // only set speeds, accel, position on local controllers
      master_steppers[i]->setMaxSpeed(speeds[i]);
      master_steppers[i]->setAcceleration(accelerations[i]);
      master_steppers[i]->setPosition(positions[i]);
    }
  }
  if (debug) Serial.println();

  secondary_setup(); // fire setup on secondary teensy

  // pin declarations
  for (int s = 0; s < 6; s++) { // loop through letters
    if (debug) aprintf("letter %d: output: s:%d, d:%d, e:%d  |  input_pullup: s1:%d, s2: %d\n",
                       s, lpins[s][0], lpins[s][1], lpins[s][2], lpins[s][3], lpins[s][4] );
    for (int p = 0; p < 5; p++) { // loop through pins
      if (lpins[s][p] != 0) { // if this pin is relevant
        if (p < 3 && s < 4) pinMode(lpins[s][p], OUTPUT); // step, dir, en
        else if (p >= 3 && s < 4) pinMode(lpins[s][p], INPUT_PULLUP); // the two sensors
      }
    }

    if (force_off) {
      if (debug) Serial.println("\nFORCE_OFF FLAG SET -- MOTORS DISABLED. SET FALSE IN UTILITIES.CPP TO ENABLE\n");
      digitalWrite(lpins[s][2], LOW); // disable all motors
    }
    else digitalWrite(lpins[s][2], HIGH); // enable all motors
  }

  pinMode(indicator, OUTPUT);
  pinMode(estop, INPUT);

  if (debug) aprintf("-----------\nestop status: %s\n-----------\n",
                     (digitalRead(estop) == HIGH) ? "high / ok to run" : "low / stopped");

  while (digitalRead(estop) == LOW) {
    if (debug) Serial.println("estop active. doing nothing.");
    delay(1000);
  } // hang until estop is reset



  if (debug) Serial.println("waiting 5secs at end of setup, just... in case.");
  for(int i = 5; i > 0; i--) {
    if (debug) Serial.println(i);
    digitalWrite(indicator, HIGH);
    delay(250);
    digitalWrite(indicator, LOW);
    delay(750);
  }
  if (debug) Serial.println("ok going...");
  last_cycle_end_time = now(); // reset cycle timer
}

void loop() {
  if (cyclestarted == false) {
    // don't stop awkwardly mid-cycle

    // setup off hours behavior
    while ((hour() < start_hour || hour() > end_hour)
           || (hour() == start_hour && minute() < start_minute)
           || (hour() == end_hour && minute() >= end_minute)
           || weekday() == 1 || weekday() == 7) { // off hours; do nothing

      if ( timecheck && second() == 0 ) { // periodic check every minute
        timecheck = false;

        // auto restart:
        if (debug) aprintf("was running: %d, reboot at: %d:%d", wasrunning, reboot_hour, reboot_minute);
        if (wasrunning && hour() >= reboot_hour && minute() >= reboot_minute) {
          // include wasrunning to ensure that we only do this once
          hwsend('R', 0, 0); // restart secondary
          if (debug) aprintf("REBOOTING");
          CPU_RESTART; // restart primary
        }

        if (debug) aprintf("sleeping. current time: %dh%dm. set to run between %dh%dm and %dh%dm\n", hour(), minute(), start_hour, start_minute, end_hour, end_minute);
      }
      if ( !timecheck && second() > 1) timecheck = true;
    };
  }

  check_hwserial(); // check for comms from secondary teensy

  // check for estop condition:
  if (digitalRead(estop) == LOW) command = "shutoff";
  else if (digitalRead(estop) == HIGH) {
    if (!powertoggle) powertoggle = true;
  }

  // update positions:
  for (int i = 0; i < 6; i++) {
    if (i < 4) positions[i] = master_steppers[i]->getPosition(); // local query
    else {
      // secondary query:
      if (millis() % txtransfergap < txtransfergap * 1 / 3 && shouldtxtransfer) {
        shouldtxtransfer = false;
        hwsend('q', 0, 0); // query positions
      }
      if (millis() % txtransfergap > txtransfergap * 2 / 3) shouldtxtransfer = true;
    }
  }

  /*           SERIAL INPUT             */

  if (debug && Serial.available() > 0) {
    int incoming = Serial.read();
    Serial.print("input: ");
    Serial.print(incoming);
    Serial.print("\t letter: ");
    Serial.write(incoming);
    Serial.println();

    if (incoming > 47 && incoming < 54) {
      // input is one of 0-5
      // subtract 49 because the ascii code for 1 is 49
      activeletter = incoming - 48;
      if (debug) Serial.print("new active letter: ");
      if (debug) Serial.println(activeletter);
    }

    switch(incoming) {
    case 99: // c
      {
        // command = "calibrate2";
        int l = Serial.parseInt();
        if (debug) Serial.print("calibration command for ");
        if (debug) Serial.println(l);
        if (l > 3) hwsend('c', l-4, 0);
        else calibrate2(l);
        break;
      }
    case 67: // C
      {
        calibrate_all();
        break;
      }
    case 32: // space
      command = "shutoff";
      break;
    case 104: // h
      command = "softstop";
      break;
    case 111: // o
      proportional();
      break;
    case 112: // p
      command = "powertoggle";
      break;
    case 120: // x
      command = "cycle";
      break;
    case 88: // X
      customcycle(
                  Serial.parseFloat(),
                  Serial.parseFloat(),
                  Serial.parseFloat(),
                  Serial.parseFloat(),
                  Serial.parseFloat(),
                  Serial.parseFloat()
                  );
      break;
    case 113: // q
      SD_read_positions();
      break;
    case 81: // Q
      force_log = true;
      log_position();
      break;
    case 109: // m
      {
        int m = Serial.parseInt();
        int tgt = Serial.parseInt();
        if (m < 4) absolutemove(m, tgt); // motor is on this teensy
        else hwsend('m', m-4, tgt); // send to 2nd teensy
        break;
      }
    case 119: // w
      {
        int m = Serial.parseInt();
        int pos = Serial.parseInt();
        if (m < 4) {
          overwriteposition(m, pos);
          force_log = true;
        }
        else hwsend('w', m-4, pos);
        break;
      }
    case 87: // W
      {
        if (debug) Serial.println("reset all to zero:");
        for(int i = 0; i < 6; i++) {
          (i < 4) ? overwriteposition(i, 0) : hwsend('w', i-4, 0);
        }
        break;
      }
    case 118: // v
      {
        int m = Serial.parseInt();
        if (m < 4) verify(m);
        else hwsend('v', m-4, 0);
        break;
      }
    case 115: // s
      {
        int l = Serial.parseInt();
        int c1 = sense(lpins[l][3]);
        int c2 = sense(lpins[l][4]);
        if (debug) aprintf("current sensor readings for letter %d: [%d %d]\n", l, c1, c2);
        break;
      }
    case 108: // l
      {
      loop_cycle = !loop_cycle;
      if (debug) aprintf("toggle loop: now set to %s\n", (loop_cycle) ? "true" : "false");
      break;
      }
    } // end switch

    if (debug && strcmp(command, "") != 0) aprintf("new command: %s\n", command);
  }

  /*             COMMANDS           */

  if (strcmp(command, "softstop") == 0) {
    if (debug) Serial.println("soft stopping...");
    hwsend('h', 0, 0);
    for (int i = 0; i < 4; i++) {
      master_controllers[i]->stopAsync();
    }
    if (debug) Serial.print("final positions: ");
    for (int i = 0; i < 4; i++) {
      if (debug) Serial.print(master_steppers[i]->getPosition());
      if (debug) Serial.print("\t");
    }
    if (debug) Serial.println();
    force_log = true;
    log_position();
  }

  else if (strcmp(command, "powertoggle") == 0) {
    powertoggle = !powertoggle;
    for (int i = 0; i < 6; i++) {
      digitalWrite(lpins[i][2], powertoggle);
    }
    if (debug) Serial.print("power toggled. enable pins ");
    if (debug) Serial.println((powertoggle) ? "ON" : "OFF");
  }

  else if (strcmp(command, "shutoff") == 0) {
    if (debug) Serial.print("POWER SHUTOFF! checking again in ");

    for (int i = 0; i < 6; i++) {
      digitalWrite(lpins[i][2], LOW);
    }
    for (int i = 0; i < 4; i++) {
      master_controllers[i]->emergencyStop();
    }
    powertoggle = false;
    for (int i = 3; i > 0; i--) {
      if (debug) Serial.print(i);
      if (debug) Serial.print("... ");
      delay(1000);
    }
    if (debug) Serial.println();
  }

  else if (strcmp(command, "cycle") == 0) {

    startnewcycle();
  }

  command = "";


  if (cyclestarted && targetsreached()) { // end of cycle
    cyclestarted = false; // reset start flag
    last_cycle_end_time = now(); // reset timer
    cycle_count++; // increment counter

    if (debug) aprintf("TARGETS REACHED, END OF CYCLE #%d. time: %d (%dh%dm%ds).\n", cycle_count, now(), hour(), minute(), second());
    force_log = true; // force log at final position
    log_position();
    if (debug && loop_cycle) aprintf("looping again in %d seconds.\n", interval_seconds);
  }

  if (second() % time_check_interval == 0 && timecheck) { // check interval between cycles
    timecheck = false;

    int interval = now() - last_cycle_end_time; // time elapsed
    if (debug) aprintf("time check, today is day %d (sunday = 1): %d (%dh%dm%ds), time since last cycle = %ds\n", weekday(), now(), hour(), minute(), second(), interval);

    if (hour() >= 11 && hour() <= 14) lunchtime = true;
    else if (hour() < 11 || hour() > 14) lunchtime = false;

    if (debug) aprintf("lunchtime = %d ", lunchtime);

    // if (now() - last_cycle_end_time >= interval_seconds && cyclestarted == false) { // interval mode
    // interval has elapsed, and we're in between cycles
    // if (((lunchtime && minute() % 30 == 0) || (!lunchtime && minute() == 0)) && cyclestarted == false) {
      // on the hour, except during lunchtime (on the half hour)

    if (( minute() % 20 == 0) && cyclestarted == false) { // run every 20 minutes all day


      if (debug) aprintf("should start now.\n");
      wasrunning = true;

      if (loop_cycle) {
        if (cycle_count >= cycles_before_calibrating) { // time to recalibrate
          if (debug) aprintf("calibrating before next run...\n");
          cycle_count = 0; // reset cycle count
          calibrate_all(); // calibrate everything
          startnewcycle(); // fire new cycle
        }
        else startnewcycle();
      }
    }
  }


  if (!timecheck && second() % time_check_interval >= 1) timecheck = true;

  log_position(); // log position if necessary

}
