#include <Arduino.h>
#include <TeensyStep.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
#include "utilities.h"

/* #pragma once */

#define HWSERIAL Serial1

int aprintf(char const * str, ...) {
  // from https://gist.github.com/EleotleCram/eb586037e2976a8d9884
	int i, j, count = 0;

	va_list argv;
	va_start(argv, str);
	for(i = 0, j = 0; str[i] != '\0'; i++) {
		if (str[i] == '%') {
			count++;

			Serial.write(reinterpret_cast<const uint8_t*>(str+j), i-j);

			switch (str[++i]) {
				case 'd': Serial.print(va_arg(argv, int));
					break;
				case 'l': Serial.print(va_arg(argv, long));
					break;
				case 'f': Serial.print(va_arg(argv, double));
					break;
				case 'c': Serial.print((char) va_arg(argv, int));
					break;
				case 's': Serial.print(va_arg(argv, char *));
					break;
				case '%': Serial.print("%");
					break;
				default:;
			};

			j = i+1;
		}
	};
	va_end(argv);

	if(i > j) {
		Serial.write(reinterpret_cast<const uint8_t*>(str+j), i-j);
	}

	return count;
}

void hwsend(char c, int i, int j){
  /* Serial.println("sending:"); */
  /* Serial.print("<"); */
  /* Serial.print(c); */
  /* Serial.print(", "); */
  /* Serial.print(i); */
  /* Serial.print(", "); */
  /* Serial.print(j); */
  /* Serial.print(">"); */
  /* Serial.println(); */
  HWSERIAL.print("<");
  HWSERIAL.print(c);
  HWSERIAL.print(", ");
  HWSERIAL.print(i);
  HWSERIAL.print(", ");
  HWSERIAL.print(j);
  HWSERIAL.print(">");
  HWSERIAL.println();
}

boolean sense(int pin) {
  // utility function to flip the values of sensor readings
  // (and shorten all the digitalRead calls)
  return !digitalRead(pin);
}

void SD_read_positions() {
  logfile = SD.open("positions.txt", FILE_READ);
  if (!logfile){
    if (debug) Serial.println("can't open position log file");
  }
  else {
    if (debug) Serial.println("SD positions log file opened ok");
  }

  if (debug) Serial.println("----------");
  if (logfile.available()) {
    if (debug) Serial.print("positions: ");
    for (int i = 0; i < 6; i++) {
      positions[i] = logfile.parseInt();
      if (debug) Serial.print(positions[i]);
      if (debug) Serial.print("\t");
    }
    if (debug) Serial.println("// EOF");
  }
  logfile.close();
}

void log_position() {
  // logs the position once every so often (interval defined global log_time variable)
  int sum = 0;

  if (force_log) {
    need_to_log = true;
    sd_log = true;
  }

  for (int i = 0; i < 6; i++) {
    sum += (prev_positions[i] == positions[i]) ? 0 : 1;
  }
  if (sum == 0) {
    sd_log = false;
    need_to_log = false;
  }
  else {
    sd_log = true;
    need_to_log = true;
  }

  if (need_to_log) {
    need_to_log = false;

    if ((millis() % log_time < log_time/3 && log_pos == true) || force_log == true) {
      log_pos = false;
      if (debug) Serial.println("need to log");

      /* if (debug && sense(E_pins[3]) && sense(E_pins[4])) Serial.print("------->"); */
      if (debug) Serial.print("positions: ");
      if (debug) {
        for (int i = 0; i < 6; i++) {
          Serial.print(positions[i]);
          if (i < 5) Serial.print("\t");
        }
        Serial.println();
      }


      if ((millis() % sd_write_time < sd_write_time/3 && sd_log == true && !already_written) || force_log == true) {
        sd_log = false;
        if (debug) Serial.println("-- logging to SD card --");
        logfile = SD.open("positions.txt", FILE_WRITE);
        if (logfile) {
          logfile.seek(0);
          logfile.truncate();
          for (int i = 0; i < 6; i++) {
            logfile.print(positions[i]);
            logfile.print((i < 5) ? ',' : '\n');
          }
          logfile.close();
          already_written = true;
        }
        for (int i = 0; i < 6; i++) {
          prev_positions[i] = positions[i];
        }
      }



    }

    if (millis() % log_time > log_time*2/3) log_pos = true;
    if (millis() % sd_write_time > sd_write_time*2/3) {
      sd_log = true;
      already_written = false;
    }

  }
  force_log = false;
}

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

void absolutemove(int m, long tgt) { // BROKEN
  if (debug) Serial.print("absolute move: motor ");
  if (debug) Serial.print(m, DEC);
  if (debug) Serial.print(", target: ");
  if (debug) Serial.println(tgt, DEC);
  if (is_primary) {
    if (debug) Serial.println("primary");
    master_steppers[m]->setTargetAbs(tgt);
    master_controllers[m]->moveAsync(*master_steppers[m]);
  }
  else {
    if (debug) Serial.print("secondary ");
    if (debug) Serial.print(m);
    if (debug) Serial.print(" ");
    if (debug) Serial.println(tgt);
    slave_steppers[m]->setTargetAbs(tgt);
    slave_controllers[m]->moveAsync(*slave_steppers[m]);
  }

  need_to_log = true;
  sd_log = true;
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
  Serial.print('0');
  Serial.print(digits);
}

void overwriteposition(int m, long pos) {
  if (debug) Serial.print("overwriting motor ");
  if (debug) Serial.print(m);
  if (debug) Serial.print(" to ");
  if (debug) Serial.print(pos);
  if (debug) Serial.print(". confirmed at: ");
  positions[m] = pos;

  if (is_primary) {
    master_steppers[m]->setPosition(pos);
    if (debug) Serial.println(master_steppers[m]->getPosition());
  }
  else {
    slave_steppers[m]->setPosition(pos);
    if (debug) Serial.println(slave_steppers[m]->getPosition());
    hwsend('a', m, pos);
    // force the log in primary.cpp, not here (log has to happen on primary teensy)
  }


}

void home(int letter) {
  if (!is_primary) letter -= 4; // shift down if we're on the secondary board

  int mod = positions[letter] % whole_revs[letter];
  /* int fromzero = (mod > whole_revs[letter]/2) ? mod - whole_revs[letter] : mod; */

  int target = (mod > whole_revs[letter]/2) ? whole_revs[letter] : 0; // either go to full rev or 0, whichever's closer

  if (is_primary) {
    // primary movement.
    master_steppers[letter]->setTargetAbs(target);
    master_controllers[letter]->move(*master_steppers[letter]);
    master_steppers[letter]->setPosition(0);
  }

  else {
    // secondary movement, this one's easy:
    slave_steppers[letter]->setTargetAbs(target);
    slave_controllers[letter]->move(*slave_steppers[letter]);
  }
  need_to_log = true;
  sd_log = true;

  if (debug) aprintf("motor %d reset to 0\n", letter);
}
