#include <Arduino.h>
#include <TeensyStep.h>
#include "utilities.h"
#include "calibrate1.h"

#define HWSERIAL Serial1

bool is_primary = false;

const unsigned int indicator = LED_BUILTIN, // light
  microsteps = 16,
  export_interval = 250;
int max_speed =10000, // top speed
  acceleration = 6000;  // acceleration = steps/sec/sec;

String command = ""; // logical control

long positions[] = {0, 0},
  prev_positions[] = {0, 0},
  avg_sens_widths[6],
  whole_revs[6],
  targets[6]; // to compare with positions[]

float thetas[6];
int certainties[6];

int speeds[] = { 1830, 1890 }; // calculated speeds per ring.

/* TEENSY */
int lpins[][5] = {
  // letter pins. two dimensional array of [letters][pins].
  // this one has no sensors (those are all on the primary board)

  // 0      1   2
  // step, dir, en, s1, nothing
  {2, 3, 4, 33, 0}, // 0: L
  {5, 6, 7, 34, 0}, // 1: E
};

boolean debug = true,
  shouldexport = true,
  need_to_log = true,
  slave_has_queried = false,
  sd_log = true;  // flag to turn on/off serial output

StepControl *master_controllers[4] = {NULL, NULL, NULL, NULL};
StepControl *slave_controllers[2] = { NULL, NULL }; // controller objects -- one per stepper, to avoid Bresenham.

Stepper *master_steppers[4] = { NULL, NULL, NULL, NULL };
Stepper *slave_steppers[2] = { NULL, NULL }; // same deal for the steppers

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char HWmsg[numChars] = {0};
int HWint1 = 0;
int HWint2 = 0;

boolean newData = false;


//============

void export_position() {
  if (millis() % export_interval < export_interval/3 && shouldexport) {
    shouldexport = false;

    if (positions[0] != prev_positions[0]) {
      hwsend('a', 0, slave_steppers[0]->getPosition());
      prev_positions[0] = positions[0];
    }
    if (positions[1] != prev_positions[1]) {
      hwsend('a', 1, slave_steppers[1]->getPosition());
      prev_positions[1] = positions[1];
    }
  }

  if (millis() % export_interval > export_interval * 2/3) shouldexport = true;
}

void dealwithit() {
  if (debug) {
    /* Serial.print("c: "); */
    /* Serial.println(HWmsg); */
    /* Serial.print("i1: "); */
    /* Serial.println(HWint1); */
    /* Serial.print("i2: "); */
    /* Serial.println(HWint2); */
  }
  if (strcmp(HWmsg, "q") == 0) {
    // query from primary

    // only send if something's changed
    shouldexport = true;
    export_position();
  }

  else if (strcmp(HWmsg, "m") == 0) {
    if (debug) {
      Serial.print("moving ");
      Serial.print(HWint1);
      Serial.print(" to ");
      Serial.println(HWint2);
    }

    absolutemove(HWint1, HWint2);
  }

  else if (strcmp(HWmsg, "O") == 0) {
    if (debug) Serial.print("setting whole_revs: ");
    whole_revs[0] = HWint1;
    whole_revs[1] = HWint2;
    if (debug) Serial.print(whole_revs[0]);
    if (debug) Serial.print(", ");
    if (debug) Serial.println(whole_revs[1]);
  }

  else if (strcmp(HWmsg, "I") == 0) {
    if (debug) Serial.print("setting avg_sens_widths: ");
    avg_sens_widths[0] = HWint1;
    avg_sens_widths[1] = HWint2;
    if (debug) Serial.print(avg_sens_widths[0]);
    if (debug) Serial.print(", ");
    if (debug) Serial.println(avg_sens_widths[1]);
  }

  else if (strcmp(HWmsg, "U") == 0) {
    if (debug) Serial.print("setting speeds: ");
    speeds[0] = HWint1;
    speeds[1] = HWint2;
    if (debug) Serial.print(speeds[0]);
    if (debug) Serial.print(", ");
    if (debug) Serial.println(speeds[1]);
  }

  else if (strcmp(HWmsg, "s") == 0) {
    speeds[HWint1] = HWint2;
    slave_steppers[HWint1]->setMaxSpeed(HWint2);

    if (debug) Serial.print("motor ");
    if (debug) Serial.print(HWint1);
    if (debug) Serial.print(" speed set to ");
    if (debug) Serial.println(HWint2);
  }
  else if (strcmp(HWmsg, "a") == 0) {
    acceleration = HWint2;
    slave_steppers[HWint1]->setAcceleration(HWint2);
    if (debug) Serial.print("motor ");
    if (debug) Serial.print(HWint1);
    if (debug) Serial.print(" acceleration set to ");
    if (debug) Serial.println(HWint2);
  }
  else if (strcmp(HWmsg, "w") == 0) {
    overwriteposition(HWint1, HWint2);
  }

  else if (strcmp(HWmsg, "C") == 0) {
    // green light for calibration
    if (debug) Serial.println("calibration green light received.");
    calibrate1_finish(HWint1);
  }
}

void setup() {
  HWSERIAL.begin(115200);

  // initial logging
  if (debug) {
    Serial.begin(9600);

    while(!Serial && millis() < 4000);
    Serial.println("=================== secondary setup =====================");
    Serial.println("source file: " __FILE__ " " __DATE__ " " __TIME__);
    Serial.println("=======================================================");
  }

  // initiate controllers
  for (int i = 0; i < 2; i++) {
    slave_controllers[i] = new StepControl();
  }

  // initiate steppers
  for (int i = 0; i < 2; i++) {
    if (debug) Serial.println("initializing steppers");
    slave_steppers[i] = new Stepper(lpins[i][0], lpins[i][1]); // step and dir/
    slave_steppers[i]->setMaxSpeed(speeds[i]);
    slave_steppers[i]->setAcceleration(acceleration);
    slave_steppers[i]->setPosition(positions[i]);
  }

  // pin declarations
  for (int s = 0; s < 2; s++) {
    if (debug) Serial.print("== letter ");
    if (debug) Serial.print(s);
    if (debug) Serial.println(" ==");
    for (int p = 0; p < 3; p++) {
      if (debug) Serial.print("setting pin to output: ");
      if (debug) Serial.println(lpins[s][p]);
      pinMode(lpins[s][p], OUTPUT);
    }

    digitalWrite(lpins[s][2], HIGH); // enable all motors
  }

  pinMode(indicator, OUTPUT);
}

void loop() {
  positions[0] = slave_steppers[0]->getPosition();
  positions[1] = slave_steppers[1]->getPosition();

  export_position();

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
