#include <Arduino.h>
#include <TeensyStep.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
#include "utilities.h"

/* #pragma once */

#define HWSERIAL Serial1

const int revolutions[] = {6, 3, 12, 4, 24, 6}; // number of times to revolve per cycle
const long offsets[] = {0, 0, 0, -300, 750, 0},
  mockup_revs[] = { 4572, 9440, 2229, 6983, 400, 5082 }, // measured manually on the mockup
  // build_revs[] = { 29128, 60082, 14060, 44378, 1500, 29880 }, // real build
  // build_revs[] = {466000, 963000, 228000, 710000, 24000, 478000}, // real build, doubled to avoid fractions. compensate by halving build_microsteps.
  mockup_sens_widths[] = {120, 125, 117, 94, 51, 110},
  build_sens_widths[] = {115, 125, 94, 150, 175, 128},
  mockup_speeds[] = { 1942, 2003, 1875, 1972, 33, 1992 },
  build_speeds[] = { 1830, 1890, 1782, 1854, 54, 2022 }
;

const float build_revs[] = { 29125, 60069, 14070, 44375, 1500, 29875 };

bool
  debug = true,
  force_off = false, // set true to pull motors low on startup -- disables everything. useful if you want to run the fountain without movement.
  mockup = false,
  loop_cycle = false,
  cyclestarted = false,
  alreadyreset = false,
  timecheck = true,
  fully_calibrated = false,
  wasrunning = false,
  lunchtime = false;

const int
  start_hour = 8,
  start_minute = 0,
  end_hour = 18,
  end_minute = 0,
  reboot_hour = 21,
  reboot_minute = 0,
  mockup_period = 30, // time in seconds for one cycle
  build_period = 600,
  cycles_before_calibrating = 2,
  interval_seconds = 60,
  time_check_interval = 5;

const unsigned int
  mockup_microsteps = 16,
  build_microsteps = 32;

bool
  homed[] = {false, false, false, false, false, false},
  build_flips[] = {false, true, true, true, false, false}, // real build
  mockup_flips[] = {false, false, true, true, false, false},
  should_be_on_zero[] = {true, true, true, true, true, true},
  already_moved_off[] = {false, false, false, false, false, false},
  done_calibrating[] = {false, false, false, false, false, false}
;


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
  if (debug && c != 'q' && c != 'a') aprintf("sending <%c, %d, %d>\n", c, i, j);
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

  // shouldn't need to do this, since it happens at the beginning of the loop:
  // for (int i = 0; i < 4; i++) {
  //   positions[i] = master_steppers[i]->getPosition();
  // }


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
      // if (debug) Serial.println("need to log");

      /* if (debug && sense(E_pins[3]) && sense(E_pins[4])) Serial.print("------->"); */
      if (debug) Serial.print("positions[sensors]: ");
      if (debug) {
        for (int i = 0; i < 6; i++) {
          if (debug) aprintf("%d", positions[i]);
          if (i < 5) {
            if (i < 4) {
              if (debug) aprintf(" [%d %d]", sense(lpins[i][3]), sense(lpins[i][4]));
            }
            if (debug) Serial.print("\t");
          }
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

void absolutemove(int m, long tgt) {

  int tempspeed = (mockup) ? 1000 * mockup_microsteps : 1000 * build_microsteps;

  if (debug) Serial.print("absolute move: motor ");
  if (debug) Serial.print(m, DEC);
  if (debug) Serial.print(", target: ");
  if (debug) Serial.println(tgt, DEC);
  if (is_primary) {
    if (debug) Serial.println("primary");
    master_steppers[m]->setMaxSpeed(tempspeed);
    master_steppers[m]->setAcceleration(tempspeed/2);
    master_steppers[m]->setTargetAbs(tgt);
    master_controllers[m]->moveAsync(*master_steppers[m]);
  }
  else {
    if (debug) Serial.print("secondary ");
    if (debug) Serial.print(m);
    if (debug) Serial.print(" ");
    if (debug) Serial.println(tgt);
    slave_steppers[m]->setMaxSpeed(tempspeed);
    slave_steppers[m]->setAcceleration(tempspeed*2);
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
  positions[m] = pos;

  if (is_primary) {
    master_steppers[m]->setPosition(pos);
  }
  else {
    slave_steppers[m]->setPosition(pos);
    hwsend('a', m, pos);
    // force the log in primary.cpp, not here (log has to happen on primary teensy)
  }

  if (debug) aprintf("overwriting motor %d to %d. confirmed at: %d\n",
                     m, pos, is_primary ?
                     master_steppers[m]->getPosition()
                     : slave_steppers[m]->getPosition());
}

void home(int letter) {
  homed[letter] = true;
  int mod = positions[letter] % whole_revs[letter];
  /* int fromzero = (mod > whole_revs[letter]/2) ? mod - whole_revs[letter] : mod; */
  int target = (mod > whole_revs[letter]/2) ? whole_revs[letter] : 0; // either go to full rev or 0, whichever's closer

  int off =  (is_primary) ? offsets[letter] : offsets[letter+4];
  off *= (mockup) ? mockup_microsteps : build_microsteps;
  target += off;
  if (debug) aprintf("off: %d, mult.: %d\n", off, (mockup) ? mockup_microsteps : build_microsteps);

  if (debug) aprintf("homing letter %d. position: %d, mod: %d, target: %d\n",
                     letter, positions[letter], mod, target);



  if (positions[letter] != target) {
    if (is_primary) {
      if (debug) aprintf("homing primary letter %d\n", letter);
      // primary movement.

      switch (letter) {
      case 0:
        {
          // collision: 0.750 < g1 < 	0.833	and	o1	0.833	< o1 <	0.931
          // collsion:  0.967	< g1 <	1.000	and o1	0.056	< o1 <	0.167
        }
      case 1:
        {

        }
      case 2:
        {

        }
      case 3:
        {

        }
      }



      master_steppers[letter]->setTargetAbs(target);
      master_controllers[letter]->move(*master_steppers[letter]);
      master_steppers[letter]->setPosition(0);
    }

    else {
      if (debug) aprintf("homing secondary letter %d\n", letter);
      // secondary movement

      switch (letter) {
      case 0:
        {
          // collision: 0.750 < g1 < 	0.833	and	o1	0.833	< o1 <	0.931
          // collsion:  0.967	< g1 <	1.000	and o1	0.056	< o1 <	0.167
        }
      case 1:
        {

        }
      }
      // secondary movement, this one's easy:
      slave_steppers[letter]->setTargetAbs(target);
      slave_controllers[letter]->move(*slave_steppers[letter]);
      slave_steppers[letter]->setPosition(0);
    }
  }

  need_to_log = true;
  sd_log = true;

  if (is_primary) done_calibrating[letter] = true;
  else hwsend('d', letter, 0);
  if (debug) aprintf("motor %d reset to 0\n", letter);
}

void startnewcycle() {

  if (debug) Serial.println("-- resetting positions for new cycle --");
  for(int i=0; i<6; i++){
    int mod = positions[i] % whole_revs[i];
    int fromzero = (mod > whole_revs[i]/2) ? mod - whole_revs[i] : mod;
    positions[i] = fromzero;
    if (i < 4) { master_steppers[i]->setPosition(positions[i]);
      if (debug) Serial.print(master_steppers[i]->getPosition());
      if (debug) Serial.print(" ");
    }
    else hwsend('w', i-4, positions[i]);
  }

  cyclestarted = true;
  // alreadyreset = false;
  need_to_log = true;
  sd_log = true;

  customcycle(1, 1, 1, 1, 1, 1);
}

bool targetsreached() {
  int reached[6];
  for (int i = 0; i < 6; i++) {
    reached[i] = (positions[i] == targets[i]) ? 1 : 0;
  }

  int sum = 0;
  for (int s = 0; s < 6; s++) {
    sum += reached[s];
  }
  if (sum == 6) return true;
  else return false;
}

bool proportional() {
  int sum = 0;
  float cmp[6];

  for (int i = 0; i < 6; i++) {
    cmp[i] = float(positions[i]) / (float(whole_revs[i])*float(revolutions[i]));
  }

  if (debug) aprintf("proportions: %f %f %f %f %f %f\n",
                     cmp[0], cmp[1], cmp[2], cmp[3], cmp[4], cmp[5]);
  if (sum == 6) return true;
  else return false;
}

void customcycle(float a, float b, float c, float d, float e, float f) {
  float fracs[] = {a, b, c, d, e, f};
  if (debug) Serial.println("custom cycle:");
  for (int i = 0; i < 6; i++) {
    targets[i] = whole_revs[i]*revolutions[i]*fracs[i];
    if (debug) aprintf("[%d]: %d * %f = %d\t",
                       i, whole_revs[i]*revolutions[i], fracs[i], targets[i]);
  }
  if (debug) Serial.println();

  syncmove(targets[0],targets[1],targets[2],targets[3]);
  hwsend('x', targets[4], targets[5]);
}

void syncmove(long a, long b) {
  long tgts[] = {a, b};
  if (debug) Serial.print("secondary sync move: ");
  for (int i = 0; i < 2; i++) {
    if (debug) aprintf("[%d]: %d\t",
                       i, tgts[i]);

    slave_steppers[i]->setMaxSpeed(speeds[i]);
    slave_steppers[i]->setAcceleration(accelerations[i]);

    slave_steppers[i]->setTargetAbs(tgts[i]);
    slave_controllers[i]->moveAsync(*slave_steppers[i]);
  }
  if (debug) Serial.println();
}

void syncmove(long a, long b, long c, long d) {
  long tgts[] = {a, b, c, d};
  if (debug) Serial.print("primary sync move: ");
  for (int i = 0; i < 4; i++) {
    if (debug) aprintf("[%d]: %d\t",
                       i, tgts[i]);

    master_steppers[i]->setMaxSpeed(speeds[i]);
    master_steppers[i]->setAcceleration(accelerations[i]);

    master_steppers[i]->setTargetAbs(tgts[i]);
    master_controllers[i]->moveAsync(*master_steppers[i]);
  }
  if (debug) Serial.println();
}

void verify(int letter) {
  int dist,
    c1 = sense(lpins[letter][3]),
    c2 = sense(lpins[letter][4]),
    p1 = sense(lpins[letter][3]),
    p2 = sense(lpins[letter][4]), // current + previous values
    ss = 0, se = 0; // sensor start + sensor end
  bool started = false, not_done_verifying = true;

  StepControl *controller;
  Stepper *stepper;

  if (is_primary) {
    controller = master_controllers[letter];
    stepper = master_steppers[letter];
  }
  else {
    controller = slave_controllers[letter];
    stepper = slave_steppers[letter];
  }

  int mod = positions[letter] % whole_revs[letter];
  int fromzero = (mod > whole_revs[letter]/2) ? mod - whole_revs[letter] : mod;

  positions[letter] = fromzero;

  if (debug) aprintf("-- verifying 2 sensor letter %d; current position: %d; sensors: %d %d --\n",
                     letter, stepper->getPosition(), sense(lpins[letter][3]), sense(lpins[letter][4]));




  // int tempspeed = (mockup) ? 500 * mockup_microsteps : 500 * build_microsteps;

    stepper->setPosition(fromzero); // rotational location
    // stepper->setMaxSpeed(tempspeed);
    // stepper->setAcceleration(tempspeed*4);

    stepper->setTargetRel(avg_sens_widths[letter]*2); // move off the sensor
    controller->moveAsync(*stepper);

  while (not_done_verifying) {
    c1 = sense(lpins[letter][3]);
    c2 = sense(lpins[letter][4]);

    if ((!c1 && !c2) && (p1 || p2)) {
      if (!started) { // first end of the sensor / pair
        ss = stepper->getPosition();
        if (debug) aprintf("first end of sensor at %d\n", ss);
        controller->stop();
        stepper->setTargetRel(avg_sens_widths[letter] * -4); // backtrack across sensor
        controller->moveAsync(*stepper);
        started = true;
       }

      else { // second end of the sensor / pair
        se = stepper->getPosition();
        dist = (se-ss)/2;
        if (debug) aprintf("second end of sensor at %d, dist = %d, target = %d\n", se, dist, se+dist/2);
        controller->stop();

        if (debug) aprintf("after stop: %d ", stepper->getPosition());
        stepper->setTargetAbs(se + dist/2); // go to center
        controller->move(*stepper);

        if (debug) aprintf("after move: %d\n", stepper->getPosition());

        int c = sense(lpins[letter][3]) + (sense(lpins[letter][4]) * 2 + 2);
        if (debug) aprintf("current sensors: [%d %d] sum = %d; ",
                           sense(lpins[letter][3]), sense(lpins[letter][4]), c);

        switch(c) {
        case 2:
          if (debug) Serial.println("0 0");
          thetas[letter] = 0.5; // 0 0
          certainties[letter] = 2;
          break;
        case 3:
          if (debug) Serial.println("1 0");
          if (mockup) thetas[letter] = (mockup_flips[letter]) ? 0.75 : 0.25; // 1 0
          else thetas[letter] = (build_flips[letter]) ? 0.75 : 0.25; // 1 0
          certainties[letter] = 1;
          break;
        case 4:
          if (debug) Serial.println("0 1");
          if (mockup) thetas[letter] = (mockup_flips[letter]) ? 0.25 : 0.75; // 0 1
          else thetas[letter] = (build_flips[letter]) ? 0.25 : 0.75; // 0 1
          certainties[letter] = 1;
          break;
        case 5:
          if (debug) Serial.println("1 1");
          thetas[letter] = 0; // 1 1
          certainties[letter] = 1;
          break;
        }

        if (certainties[letter] == 1) {
          positions[letter] = thetas[letter]*whole_revs[letter];
          stepper->setPosition(positions[letter]);
          if (debug) aprintf("verified. letter %d theta: %f, position set to %d\n",
                             letter, thetas[letter], positions[letter]);
          not_done_verifying = false;
          need_to_log = true;
          sd_log = true;
        }

      } // end else (i.e. already started)
    } // end if no sensor

    p1 = c1;
    p2 = c2;
  } // end while
} // end verify
