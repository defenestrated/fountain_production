#include <Arduino.h>
#include <TeensyStep.h>
#include <SD.h>
#include "utilities.h"

#define HWSERIAL Serial1

const int chipSelect = BUILTIN_SDCARD;

StepControl *controllers[4] = {NULL, NULL, NULL, NULL}; // controller objects -- one per stepper, to avoid Bresenham. first four steppers here, last two on slave.

File logfile;

const int indicator = LED_BUILTIN, // light
  estop = 17, // input from the e-stop.
  microsteps = 8,
  log_time = 250, // in ms, how often to report postitions.
  sd_write_time = 1000, // defines SD read/write operations, so don't make this too small
  motor_steps_per_rev = 200; // how many steps per one motor revolution
int max_speed =10000, // top speed
  acceleration = 6000;  // acceleration = steps/sec/sec;

int avg_sens_widths[] = {122, 124, 118, 100, 51, 110}; // rough number of steps it takes to traverse a sensor
/* const int avg_sens_widths[] = {115, 112, 98, 100, 175, 128}; // rough number of steps it takes to traverse a sensor @ 400ustep */
/* const int avg_sens_widths[] = {460, 448, 392, 400, 700, 512}; // rough number of steps it takes to traverse a sensor @ 1600ustep */
/* const int avg_sens_widths[] = {14720, 14336, 12544, 12800, 22400, 16384}; // rough number of steps it takes to traverse a sensor @ 128kustep */

int activeletter = 4; // for testing purposes
String command = ""; // logical control

boolean mockup = true; // set false for uploading to the real build

//long whole_revs[] = { 4572, 9440, 2229, 6983, 400, 5082 }; // measured manually on the mockup
long whole_revs[] = { 29128, 60082, 14060, 44378, 1091, 29880 }; // real build, measured manually
/* long whole_revs[] = { 116512, 240328, 56240, 177512, 6000, 119520 }; // real build, measured * 4 for 1600ustep */
/* long whole_revs[] = { 3728384, 7690496, 1799680, 5680384, 192000, 3824640 }; */
const long offsets[] = {0, 0, 0, 0, 0, 0}; // distance from sensor 0 to home position
const int revolutions[] = {6, 3, 12, 4, 2, 6}; // number of times to revolve per cycle
long positions[6];
long prev_positions[6];
long period = 30; // time in seconds for one cycle
long targets[6]; // to compare with positions[]

int certainties[] = { 0, 0, 0, 0, 0, 0 };
boolean searching = false;
boolean flips[] = {false, true, true, false, false, false}; // real build
float thetas[6];
/* int speeds[] = { 1830, 1890, 1782, 1854, 54, 2022 }; // calculated speeds per ring. */
int speeds[] = { 1942, 2003, 1875, 1972, 33, 1992 };

/* TEENSY */
int lpins[][5] = { // letter pins. two dimensional array of [letters][pins].

                  // 0      1   2     3      4
                  // step, dir, en, sens1, sens2
                  {2, 3, 4, 33, 34}, // 0: G1
                  {5, 6, 7, 35, 36}, // 1: O1
                  {8, 9, 10, 37, 38}, // 2: 02
                  {11, 12, 24, 39, 14}, // 3: G2
                  {25, 26, 27, 15, 0},  // 4: L
                  {28, 29, 30, 16, 0}  // 5: E

};


/* MEGA */
/* int lpins[][5] = { // letter pins. two dimensional array of [letters][pins]. */

/*   // 0      1   2     3      4 */
/*   // step, dir, en, sens1, sens2 */
/*   {A0, A1, 38, 27, 7}, // 0: G1 */
/*   {A6, A7, A2, 33, 4}, // 1: O1 */
/*   {46, 48, A8, 2, 22}, // 2: 02 */
/*   {26, 28, 24, 50, 5}, // 3: G2 */
/*   {36, 34, 30, 3, 0},  // 4: L */
/*   {47, 32, 45, 35, 0}  // 5: E */

/* }; */

boolean debug = true,  // flag to turn on/off serial output
  powertoggle = true, // indicator as to whether we've disabled the motors or not
  cyclestarted = false,
  positionreset = false,
  need_to_log = true,
  SDokay = true,
  force_log = false,
  log_pos = true, sd_log = true; // flags to note whether we've logged positions during moves (either to serial or eventually to SD)


// initiate stepper array (will be populated in setup)
Stepper *steppers[4] = { NULL, NULL, NULL, NULL };

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char HWmsg[numChars] = {0};
int HWint1 = 0;
int HWint2 = 0;

boolean newData = false;

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (HWSERIAL.available() > 0 && newData == false) {
        rc = HWSERIAL.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {
      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(HWmsg, strtokIndx); // copy it to HWmsg

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    HWint1 = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    HWint2 = atoi(strtokIndx);     // convert this part to a float

}

//============

void dealwithit() {
    Serial.print("c: ");
    Serial.println(HWmsg);
    Serial.print("i1: ");
    Serial.println(HWint1);
    Serial.print("i2: ");
    Serial.println(HWint2);
}

void setup() {


  pinMode(indicator, OUTPUT);
  Serial.begin(9600);

  HWSERIAL.begin(115200);

  while(!Serial && millis() < 4000);
  Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  Serial.println("hi. i'm primary");

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  if (!SD.begin(chipSelect)) {
    if (debug) Serial.println("SD card initialization failed");
    SDokay = false;
  }
  if (debug) Serial.println("SD card initialization ok");
}

void loop() {

  if (Serial.available() > 0) {
    int incoming = Serial.read();
    Serial.print("input: ");
    Serial.print(incoming);
    Serial.print("\t letter: ");
    Serial.write(incoming);
    Serial.println();

    switch(incoming) {
    case 113: // q
      hwsend('q', 0, 0);
      break;
    case 109: // m
      hwsend('m', Serial.parseInt(), Serial.parseInt());
    }
  }

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
