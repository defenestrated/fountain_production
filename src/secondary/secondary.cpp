#include <Arduino.h>
#include <TeensyStep.h>
#include "utilities.h"
#include "calibrate1.h"

#define HWSERIAL Serial1

bool is_primary = false;

const unsigned int indicator = LED_BUILTIN, // light
  export_interval = 250;
// /* int */ acceleration = 6000;  // acceleration = steps/sec/sec;

const char* command = ""; // logical control

long positions[] = {0, 0},
  prev_positions[] = {0, 0},
  avg_sens_widths[6],
  whole_revs[6],
  targets[6]; // to compare with positions[]

float thetas[6];
int certainties[6];

int speeds[] = { 1830, 1890 },
  accelerations[6]; // calculated speeds per ring.

/* TEENSY */
int lpins[][5] = {
  // letter pins. two dimensional array of [letters][pins].
  // this one has no sensors (those are all on the primary board)

  // 0      1   2
  // step, dir, en, s1, nothing
  {2, 3, 4, 33, 0}, // 0: L
  {5, 6, 7, 34, 0}, // 1: E
};

bool
  shouldexport = true,
  need_to_log = true,
  slave_has_queried = false,
  sd_log = true;  // flag to turn on/off serial output

StepControl *master_controllers[4] = { NULL, NULL, NULL, NULL };
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

bool ready = false;
const int readycount = 5;
bool checklist[] = {
  false, // 0 whole revolution distance
  false, // 1 sensor widths
  false, // 2 speeds
  false, // 3 positions
  false  // 4 accelerations
};
bool newData = false;


//============

void export_position() {
  positions[0] = slave_steppers[0]->getPosition();
  positions[1] = slave_steppers[1]->getPosition();

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

void preflight(int which, int status) {
  int readysum = 0;
  checklist[which] = status;
  if (debug) Serial.print("ready states: ");
  for (int i = 0; i < readycount; i++) {
    if (debug) Serial.print(checklist[i]);
    readysum += checklist[i];
  }
  if (readysum == readycount) ready = true;
  else ready = false;
  if (debug) Serial.println();
}

void dealwithit() {

  if (strcmp(HWmsg, "q") == 0) {
    // query from primary

    // only send if something's changed
    shouldexport = true;
    export_position();
  }

  else if (strcmp(HWmsg, "!") == 0) {
    ready = false;
    for (int i = 0; i < 5; i++) {
      preflight(i,0);
    }
  }

  else if (strcmp(HWmsg, "m") == 0) {
    if (debug) aprintf("moving %d to %d\n", HWint1, HWint2);
    absolutemove(HWint1, HWint2);
  }

  else if (strcmp(HWmsg, "h") == 0) {
    if (debug) Serial.println("soft stop");
    for (int i = 0; i < 2; i++) {
      slave_controllers[i]->stopAsync();
    }
    shouldexport = true;
  }

  else if (strcmp(HWmsg, "O") == 0) {
    if (debug) Serial.print("setting whole_revs: ");
    whole_revs[0] = HWint1;
    whole_revs[1] = HWint2;
    if (debug) Serial.print(whole_revs[0]);
    if (debug) Serial.print(", ");
    if (debug) Serial.println(whole_revs[1]);
    preflight(0, 1);
  } // whole rev set

  else if (strcmp(HWmsg, "I") == 0) {
    if (debug) Serial.print("setting avg_sens_widths: ");
    avg_sens_widths[0] = HWint1;
    avg_sens_widths[1] = HWint2;
    if (debug) Serial.print(avg_sens_widths[0]);
    if (debug) Serial.print(", ");
    if (debug) Serial.println(avg_sens_widths[1]);
    preflight(1, 1);
  } // sensor width set

  else if (strcmp(HWmsg, "U") == 0) {
    if (debug) Serial.print("setting speeds: ");
    speeds[0] = HWint1;
    speeds[1] = HWint2;
    if (debug) Serial.print(speeds[0]);
    if (debug) Serial.print(", ");
    if (debug) Serial.println(speeds[1]);
    slave_steppers[0]->setMaxSpeed(speeds[0]);
    slave_steppers[1]->setMaxSpeed(speeds[1]);
    preflight(2, 1);
    // change to be one-off calls for speeds + accel
  } // speed set

  else if (strcmp(HWmsg, "P") == 0) {
    if (debug) Serial.println("setting positions: ");
    overwriteposition(0, HWint1);
    overwriteposition(1, HWint2);
    preflight(3, 1);
  } // positions set

  else if (strcmp(HWmsg, "Y") == 0) {
    if (debug) Serial.print("setting accelerations: ");
    accelerations[0] = HWint1;
    accelerations[1] = HWint2;
    slave_steppers[0]->setAcceleration(accelerations[0]);
    slave_steppers[1]->setAcceleration(accelerations[1]);

    if (debug) Serial.print(accelerations[0]);
    if (debug) Serial.print(", ");
    if (debug) Serial.println(accelerations[1]);
    preflight(4, 1);
  } // accelerations set

  else if (strcmp(HWmsg, "s") == 0) {
    speeds[HWint1] = HWint2;
    slave_steppers[HWint1]->setMaxSpeed(HWint2);

    if (debug) Serial.print("motor ");
    if (debug) Serial.print(HWint1);
    if (debug) Serial.print(" speed set to ");
    if (debug) Serial.println(HWint2);
  }
  else if (strcmp(HWmsg, "a") == 0) {
    accelerations[HWint1] = HWint2;
    slave_steppers[HWint1]->setAcceleration(HWint2);
    if (debug) Serial.print("motor ");
    if (debug) Serial.print(HWint1);
    if (debug) Serial.print(" acceleration set to ");
    if (debug) Serial.println(HWint2);
  }
  else if (strcmp(HWmsg, "w") == 0) {
    overwriteposition(HWint1, HWint2);
  }

  else if (strcmp(HWmsg, "c") == 0) {
    calibrate1(HWint1);
  }

  else if (strcmp(HWmsg, "v") == 0) {
    verify(HWint1);
  }

  else if (strcmp(HWmsg, "C") == 0) {
    // green light for calibration
    if (debug) Serial.println("calibration green light received.");
    calibrate1_finish(HWint1);
  }

  else if (strcmp(HWmsg, "x") == 0) {
    // start full cycle
    if (debug) aprintf("starting cycle! targets: %d, %d\n", HWint1, HWint2);
    syncmove(HWint1, HWint2);
  }
}

void setup() {
  ready = false;
  HWSERIAL.begin(115200);

  // initial logging
  if (debug) {
    Serial.begin(9600);

    while(!Serial && millis() < 4000);
    Serial.println("=================== secondary setup =====================");
    Serial.println("source file: " __FILE__ " " __DATE__ " " __TIME__);
    Serial.println("=========================================================");
  }

  // initiate controllers
  for (int i = 0; i < 2; i++) {
    slave_controllers[i] = new StepControl();
  }

  // initiate steppers
  if (debug) Serial.print("initializing steppers... ");
  for (int i = 0; i < 2; i++) {
    if (debug) aprintf("%d(%d, %d) ", i, lpins[i][0], lpins[i][1]);
    slave_steppers[i] = new Stepper(lpins[i][0], lpins[i][1]); // step and dir/
  }
  if (debug) Serial.println();

  // pin declarations

  if (debug) Serial.println("-- pin setup --");
  for (int s = 0; s < 2; s++) {
    if (debug) aprintf("letter %d: output: s:%d, d:%d, e:%d  |  input_pullup: s1:%d\n",
                       s, lpins[s][0], lpins[s][1], lpins[s][2], lpins[s][3] );

    for (int p = 0; p < 5; p++) {
      if (lpins[s][p] != 0) {
        if (p < 3) pinMode(lpins[s][p], OUTPUT);
        else pinMode(lpins[s][p], INPUT_PULLUP);
      }
    }



    digitalWrite(lpins[s][2], HIGH); // enable all motors
  }

  pinMode(indicator, OUTPUT);
  if (!ready) hwsend('g', 0, 0); // ask for setup info

}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    dealwithit();
    newData = false;
  }

  export_position();
}
