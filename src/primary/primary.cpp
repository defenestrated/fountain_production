#include <Arduino.h>
#include <TeensyStep.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
#include "utilities.h"

#define HWSERIAL Serial1

bool is_primary = true;

const int chipSelect = BUILTIN_SDCARD;

StepControl *master_controllers[4] = {NULL, NULL, NULL, NULL}; // controller objects -- one per stepper, to avoid Bresenham. first four steppers here, last two on slave.
StepControl *slave_controllers[2] = { NULL, NULL };

File logfile;

const unsigned int indicator = LED_BUILTIN, // light
  estop = 17, // input from the e-stop.
  microsteps = 16,
  log_time = 250, // in ms, how often to report postitions.
  sd_write_time = 1000, // defines SD read/write operations, so don't make this too small
  motor_steps_per_rev = 400; // how many steps per one motor revolution
int max_speed =10000, // top speed
  acceleration = 6000;  // acceleration = steps/sec/sec;

long avg_sens_widths[]; // rough number of steps it takes to traverse a sensor

int activeletter = 4; // for testing purposes
String command = ""; // logical control

boolean mockup = true; // set false for uploading to the real build

const long mockup_revs[] = { 4572, 9440, 2229, 6983, 400, 5082 }, // measured manually on the mockup
  build_revs[] = { 29128, 60082, 14060, 44378, 1091, 29880 }, // real build
  offsets[] = {0, 0, 0, 0, 0, 0}, // distance from sensor 0 to home position
  mockup_sens_widths[] = {122, 124, 118, 100, 51, 110},
  build_sens_widths[] = {115, 112, 98, 100, 175, 128},
  mockup_speeds[] = { 1942, 2003, 1875, 1972, 33, 1992 },
  build_speeds[] = { 1830, 1890, 1782, 1854, 54, 2022 }
;

long whole_revs[6];
const int revolutions[] = {6, 3, 12, 4, 2, 6}; // number of times to revolve per cycle
long positions[6];
long prev_positions[6];
int speeds[6];
int period = 30; // time in seconds for one cycle
long targets[6]; // to compare with positions[]

int certainties[] = { 0, 0, 0, 0, 0, 0 };
boolean searching = false;
boolean flips[] = {false, true, true, false, false, false}; // real build
float thetas[6];

const unsigned int txtransfergap = 50;
boolean shouldtxtransfer = true;

/* TEENSY */
int lpins[][5] = { // letter pins. two dimensional array of [letters][pins].

  // 0      1   2     3      4
  // step, dir, en, sens1, sens2
  {2, 3, 4, 33, 34}, // 0: G1
  {5, 6, 7, 35, 36}, // 1: O1
  {8, 9, 10, 37, 38}, // 2: 02
  {11, 12, 24, 39, 14}, // 3: G2
  {25, 26, 27, 15, 0},  // 4: L -- note that the s d e pins aren't used here
  {28, 29, 30, 16, 0}  // 5: E -- ditto (they're on the secondary board)

};

boolean debug = true,  // flag to turn on/off serial output
  powertoggle = true, // indicator as to whether we've disabled the motors or not
  cyclestarted = false,
  positionreset = false,
  need_to_log = true,
  SDokay = true,
  force_log = false,
  log_pos = true, sd_log = true, already_written = false; // flags to note whether we've logged positions during moves

// initiate stepper array (will be populated in setup)
Stepper *master_steppers[4] = { NULL, NULL, NULL, NULL };
Stepper *slave_steppers[2] = { NULL, NULL };

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char HWmsg[numChars] = {0};
int HWint1 = 0;
int HWint2 = 0;

boolean newData = false;

//============

void dealwithit() {
  if (debug) {
    // Serial.print("c: ");
    // Serial.println(HWmsg);
    // Serial.print("i1: ");
    // Serial.println(HWint1);
    // Serial.print("i2: ");
    // Serial.println(HWint2);
  }

  if (strcmp(HWmsg, "a") == 0) {
    // answer from secondary re: position of specific motor
    positions[HWint1 + 4] = HWint2;
    // need_to_log = true;
    // sd_log = true;
    log_position();

    if (debug) Serial.print("received secondary positions. new positions: ");
    for (int i = 0; i < 6; i++) {
      if (debug) Serial.print(positions[i]);
      if (debug) Serial.print(" ");
    }
    if (debug) Serial.println();
  }
  else if (strcmp(HWmsg, "C") == 0) {
    // query from slave; send positions, then calibration greenlight
    if (debug) Serial.print("calibration request for slave letter ");
if (debug) Serial.println(HWint1);
     hwsend('w', HWint1, positions[HWint1 + 4]);
     hwsend('C', HWint1, 0); // green light signal for calibration
  }
}

void setup() {

  for (int i = 0; i < 6; i++) {
    if (mockup) {
      whole_revs[i] = mockup_revs[i] * microsteps;
      avg_sens_widths[i] = mockup_sens_widths[i] * microsteps;
      speeds[i] = mockup_speeds[i] * microsteps;
      // speeds[i] = mockup_speeds[i] * 2;
    }
    else {
      whole_revs[i] = build_revs[i] * microsteps;
      avg_sens_widths[i] = build_sens_widths[i] * microsteps;
      speeds[i] = build_speeds[i] * microsteps;
      // speeds[i] = build_speeds[i] * 2;
    }
  }

  setSyncProvider(getTeensy3Time);

  HWSERIAL.begin(115200);

  // initial logging
  if (debug) {
    Serial.begin(9600);



    while(!Serial && millis() < 4000);
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

    Serial.print("max speed: ");
    Serial.print(max_speed);
    Serial.print( " accel: ");
    Serial.print(acceleration);
    Serial.println();

  }

  while(!HWSERIAL && millis() < 4000);

  hwsend('O', whole_revs[4], whole_revs[5]); // send whole_rev measurements
  hwsend('I', avg_sens_widths[4], avg_sens_widths[5]); // send sensor widths
  hwsend('U', speeds[4], speeds[5]); // send speeds


  // sd init
  if (!SD.begin(chipSelect)) {
    if (debug) Serial.println("SD card initialization failed");
    SDokay = false;
  }
  else {
    if (debug) Serial.println("SD card initialization ok");
  }

  if (SDokay) SD_read_positions();
  else if (!SDokay) {
    Serial.println("SD FAILURE, DOING NOTHING");
    while(true);
  }


  // initiate controllers
  for (int i = 0; i < 4; i++) {
    master_controllers[i] = new StepControl();
  }

  while (millis() < 2000);

  // initiate steppers
  for (int i = 0; i < 6; i++) {

    if (i < 4) {
      master_steppers[i] = new Stepper(lpins[i][0], lpins[i][1]); // step and dir

      // set speeds + accelerations
      /* master_steppers[i]->setMaxSpeed(whole_revs[i]*revolutions[i]/period); */
      master_steppers[i]->setMaxSpeed(speeds[i]);
      master_steppers[i]->setAcceleration(acceleration);
      master_steppers[i]->setPosition(positions[i]);
    }
    else {
if (debug) Serial.print("to secondary... ");
      if (debug) Serial.println(i);
      hwsend('s', i-4, speeds[i]);
      hwsend('a', i-4, acceleration);
      hwsend('w', i-4, positions[i]);
    }
  }

  // pin declarations
  for (int s = 0; s < 6; s++) {
    if (debug) Serial.print("== letter ");
    if (debug) Serial.print(s);
    if (debug) Serial.println(" ==");
    for (int p = 0; p < 5; p++) {
      if (lpins[s][p] != 0) {
        if (p < 3 && s < 4) {
          if (debug) Serial.print("setting pin to output: ");
          if (debug) Serial.println(lpins[s][p]);
          pinMode(lpins[s][p], OUTPUT);
        }
        else {
          if (debug) Serial.print("setting pin to input_pullup: ");
          if (debug) Serial.println(lpins[s][p]);
          pinMode(lpins[s][p], INPUT_PULLUP);
        }
      }
    }

    digitalWrite(lpins[s][2], HIGH); // enable all motors
  }

  pinMode(indicator, OUTPUT);
  pinMode(estop, INPUT);

  if (debug) Serial.println("---------");
  if (debug) Serial.print("estop status: ");
  if (debug) Serial.println((digitalRead(estop) == HIGH) ? "high" : "low");

  while (digitalRead(estop) == LOW) {
    if (debug) Serial.println("estop active. doing nothing.");
    delay(1000);
  } // hang until estop is reset

  if (debug) Serial.println("waiting 3secs at end of setup, just... in case.");
  for(int i = 3; i > 0; i--) {
    if (debug) Serial.println(i);
    digitalWrite(indicator, HIGH);
    delay(250);
    digitalWrite(indicator, LOW);
    delay(750);
  }
  if (debug) Serial.println("ok going...");
}

void loop() {

  if (digitalRead(estop) == LOW) command = "shutoff";
  for (int i = 0; i < 6; i++) {
    if (i < 4) positions[i] = master_steppers[i]->getPosition();
    else {
      if (millis() % txtransfergap < txtransfergap * 1 / 3 && shouldtxtransfer) {
        shouldtxtransfer = false;
        hwsend('q', 0, 0);
      }
      if (millis() % txtransfergap > txtransfergap * 2 / 3) shouldtxtransfer = true;
    }
  }

  log_position();

  if (cyclestarted && !need_to_log && !positionreset) { // i.e. not moving, but started a cycle

    if (debug) Serial.println("RESETTING POSITIONS");
    for(int i=0; i<6; i++){
      int mod = positions[i] % whole_revs[i];
      int fromzero = (mod > whole_revs[i]/2) ? mod - whole_revs[i] : mod;
      positions[i] = fromzero;
      if (i < 4) { master_steppers[i]->setPosition(positions[i]);
        if (debug) Serial.print(master_steppers[i]->getPosition());
        if (debug) Serial.print(" ");

        // TODO : send HW signal to setPosition
      }
    }
    if (debug) Serial.println();
    positionreset = true;
    cyclestarted = false;
    // need_to_log = true;
    // sd_log = true;
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
    case 67: // C
      command = "calibrate1";
      break;
    case 99: // c
      command = "calibrate2";
      break;
    case 32: // space
      command = "shutoff";
      break;
    case 104: // h
      command = "softstop";
      break;
    case 112: // p
      command = "powertoggle";
      break;
    case 120: // x
      command = "cycle";
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
    }
  }

  /*             COMMANDS           */

  if (command == "calibrate1") {
    // calibrate1(activeletter);
  }
  else if (command == "calibrate2") {
    // calibrate2(activeletter);
  }
  else if (command == "softstop") {
    if (debug) Serial.println("soft stopping...");
    for (int i = 0; i < 4; i++) {
      master_controllers[i]->stop();
    }
    if (debug) Serial.print("final positions: ");
    for (int i = 0; i < 6; i++) {
      if (debug) Serial.print(master_steppers[i]->getPosition());
      if (debug) Serial.print("\t");
    }
    if (debug) Serial.println();
    force_log = true;
    log_position();

  }

  else if (command == "powertoggle") {
    powertoggle = !powertoggle;
    for (int i = 0; i < 6; i++) {
      digitalWrite(lpins[i][2], powertoggle);
    }
    if (debug) Serial.print("power toggled. enable pins ");
    if (debug) Serial.println((powertoggle) ? "ON" : "OFF");
  }

  else if (command == "shutoff") {
    if (debug) Serial.print("POWER SHUTOFF! checking again in ");

    for (int i = 0; i < 6; i++) {
      digitalWrite(lpins[i][2], LOW);
    }
    for (int i = 0; i < 4; i++) {
      master_controllers[i]->emergencyStop();
    }    powertoggle = false;
    for (int i = 3; i > 0; i--) {
      if (debug) Serial.print(i);
      if (debug) Serial.print("... ");
      delay(1000);
    }
    if (debug) Serial.println();
  }

  else if (command == "cycle") {
    if (debug) Serial.println("starting cycle!");
    // startnewcycle();
  }
  command = "";

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
