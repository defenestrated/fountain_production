#include <Arduino.h>
#include <TeensyStep.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
#include "utilities.h"
#include "calibrate1.h"

int dir, c1, s1, p1;
long target, ss, se, dist;
boolean started;

void calibrate1_start(int letter) {
  if (debug) Serial.println("asking for positions...");
  hwsend('C', letter, 0); // ask for help calibrating
}

void calibrate1(int letter) {

 started = false;

  calibrate1_start(letter);
}

void calibrate1_finish(int letter) {
  target = 0;
  dir = 1;
  c1 = sense(lpins[letter][3]);
  p1 = sense(lpins[letter][3]);
  s1 = c1; // sensor state
  ss = 0; se = 0; // sensor start + sensor end

  int mod = positions[letter] % whole_revs[letter];
  int fromzero = (mod > whole_revs[letter]/2) ? mod - whole_revs[letter] : mod;


  if (!is_primary) slave_steppers[letter]->setPosition(fromzero);
  /* steppers[letter]->setPosition(0); */


  if (debug) aprintf("-- calibrating 1 sensor letter %d; current position: %d --\n",
                     letter, slave_steppers[letter]->getPosition());

  for (int i = 1; i <= 8; i++) {

    // start swinging back and forth:
    if (i % 2 == 0) {
      dir = -1; // switch directions each time
      // if (letter == 4) target = whole_revs[letter]*2*dir;
      // else
      target = fromzero + (whole_revs[letter]/8*(i/2)+avg_sens_widths[letter]*2)*dir; // progressively larger units, up to 1/2 turn
    }
    else {
      dir = 1; // switch directions each time
      // if (letter == 4) target = whole_revs[letter]*2*dir;
      // else
      target = fromzero + whole_revs[letter]/8*(i+1)/2+avg_sens_widths[letter]*2; // progressively larger units, up to 1/2 turn
    }

    if (debug) aprintf("target: %s %d\n", (dir == 1) ? "+ " : "- ", target);

    slave_steppers[letter]->setTargetAbs(target);
    if (!slave_controllers[letter]->isRunning()) slave_controllers[letter]->moveAsync(*slave_steppers[letter]);

    while (slave_controllers[letter]->isRunning()) {

      c1 = sense(lpins[letter][3]);
      // if (millis() % 100 < 10 && debug) Serial.println(c1);

      if (c1 != p1) { // state change

        if (c1 && !p1) { // leading edge
          started = true; // flag to avoid half-readings if we start on a sensor
          ss = slave_steppers[letter]->getPosition(); // set sensor start

          s1 = c1;

          if (debug) aprintf("begin, status: %s at %d\n", c1 ? "c1" : "__", ss);
        }

        if (!c1 && p1 && started) { // trailing edge

          se = slave_steppers[letter]->getPosition();  // set sensor end
          dist = se-ss; // calculate distance

          if (abs(dist) > 10) {

            if (debug) aprintf("end, status: %s / %s at %d; abs(dist): %d [avg. %d]\n",
                               c1 ? "c1" : "__",
                               p1 ? "p1" : "__",
                               se, dist, avg_sens_widths[letter]);

            if (abs(dist) > avg_sens_widths[letter]*3/4 && abs(dist) < avg_sens_widths[letter]*5/4) { // width range of the sensor CHECK ON REAL BUILD
              goto endsense;
            }
          }
        } // end trailing edge
      } // end state change

      p1 = c1;
    } // end while

  }
 endsense:

  if (abs(dist) > avg_sens_widths[letter]*3/4 && abs(dist) < avg_sens_widths[letter]*5/4) {
    thetas[letter] = 0;
    certainties[letter] = 1;
    target = ss + (dist)/2;

    if (debug) aprintf("--------> end of sensor. distance: %d, theta: %f, center = %d\n",
                       dist, thetas[letter], target);

    slave_controllers[letter]->stop();
    slave_steppers[letter]->setTargetAbs(target);
    slave_controllers[letter]->move(*slave_steppers[letter]);
    slave_steppers[letter]->setPosition(0);
    started = false;

    positions[letter] = slave_steppers[letter]->getPosition();

    if (debug) aprintf("letter %d calibrated.theta = %f, position = %d\n",
                     letter, thetas[letter], positions[letter]);

    hwsend('a', letter, positions[letter]);


    need_to_log = true;
    sd_log = true;
  }

  home(letter);

} // end test_calibrate
