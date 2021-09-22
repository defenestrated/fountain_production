#include <Arduino.h>
#include "utilities.h"

#define HWSERIAL Serial1

int led = LED_BUILTIN;

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

void parseData() {      // split the data into its parts

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

    if (strcmp(HWmsg, "q") == 0) {
      hwsend('a', random(1000), random(1000));
    }
    else if (strcmp(HWmsg, "m") == 0) {
      Serial.print("moving to ");
      Serial.print(HWint1);
      Serial.print("/");
      Serial.println(HWint2);
    }
}

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(9600);



  HWSERIAL.begin(115200);

  while(!Serial && millis() < 4000);
  Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  Serial.println("hi. i'm secondary");
  delay(500);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
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
}
