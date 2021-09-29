#include <Arduino.h>
#include <TeensyStep.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
#include "utilities.h"
#include "calibrate1.h"

void calibrate2(int letter) {
  int target = 0, dir = 1, dist = 0,
    c1 = sense(lpins[letter][3]),
    c2 = sense(lpins[letter][4]),
    p1 = sense(lpins[letter][3]),
    p2 = sense(lpins[letter][4]), // current + previous values
    s1 = 0, s2 = 0,
    ss = 0, se = 0; // sensor start + sensor end

  boolean started = false,
    foundone = false;

  SD_read_positions();

  int mod = positions[letter] % whole_revs[letter];
  int fromzero = (mod > whole_revs[letter]/2) ? mod - whole_revs[letter] : mod;

  master_steppers[letter]->setPosition(fromzero); // rotational location
  positions[letter] = fromzero;

  if (debug) Serial.print("-- calibrating 2 sensor letter ");
  if (debug) Serial.println(letter);
  if (debug) Serial.print(" current position: ");
  if (debug) Serial.println(master_steppers[letter]->getPosition());



  for (int i = 1; i <= 8; i++) {
    // start swinging back and forth:
    if (i % 2 == 0) {
      dir = -1; // switch directions each time
      target = fromzero + (whole_revs[letter]/32*(i/2)+avg_sens_widths[letter]*4)*dir; // progressively larger units, up to 1/4 turn
    }
    else {
      dir = 1; // switch directions each time
      target = fromzero + whole_revs[letter]/32*(i+1)/2+avg_sens_widths[letter]*4; // progressively larger units, up to 1/4 turn
    }

    if (debug) Serial.print((dir == 1) ? "+ " : "- ");
    if (debug) Serial.println(target);
    master_steppers[letter]->setTargetAbs(target);
    if (!master_controllers[letter]->isRunning()) master_controllers[letter]->moveAsync(*master_steppers[letter]);

    while (master_controllers[letter]->isRunning()) {
      foundone = false;
      /* Serial.println(sense(lpins[0][3])); */

      /* sensors[letter]->update(); */

      c1 = sense(lpins[letter][3]);
      c2 = sense(lpins[letter][4]);

      if (c1 != p1 || c2 != p2) { // state change

        if ((c1 && !p1) || (c2 && !p2)) { // leading edge
          started = true;
          ss = master_steppers[letter]->getPosition(); // set sensor start

          s1 = c1;
          s2 = c2;

          if (debug) Serial.print("begin ");
          if (debug && c1) Serial.print("c1 ");
          if (debug && c2) Serial.print("c2 ");
          if (debug && p1) Serial.print("p1 ");
          if (debug && p2) Serial.print("p2 ");
          if (debug) Serial.println(ss);

          /* controller.stop(); */
          /* if (debug) Serial.print("stopped position: "); */
          /* if (debug) Serial.println(master_steppers[letter]->getPosition()); */
          /* if (debug) Serial.print("new target: "); */
          /* target = ss + 150 * dir; */
          /* if (debug) Serial.println(target); */
          /* master_steppers[letter]->setTargetAbs(target); */
          /* if (!controller.isRunning()) */
          /*   controller.moveAsync(*master_steppers[letter]); */


          if (c1 && c2) { // start of 1 1
            /* ss = master_steppers[letter]->getPosition(); */
            if (debug) Serial.print("1 1 leading edge: ");
            Serial.println(ss);
            Serial.println(master_steppers[letter]->getPosition());
          }
        }

        if (((!c1 && p1) || (!c2 && p2)) && started) { // trailing edge

          se = master_steppers[letter]->getPosition();  // set sensor end
          dist = se-ss; // calculate distance

          if (abs(dist) > 10) {

            if (debug) Serial.print("end ");
            if (debug && c1) Serial.print("c1 ");
            if (debug && c2) Serial.print("c2 ");
            if (debug && p1) Serial.print("p1 ");
            if (debug && p2) Serial.print("p2 ");


            if (debug) Serial.print(se);
            if (debug) Serial.print(" dist: ");
            if (debug) Serial.println(dist);


            if (p1 && p2) { // end of 1 1 given width range of the sensor CHECK ON REAL BUILD
              if (debug) Serial.println("1 1 trailing edge");
              /* goto endsense; */
            }

            if (abs(dist) > avg_sens_widths[letter]*3/4 && abs(dist) < avg_sens_widths[letter]*5/4) { // width range of the sensor CHECK ON REAL BUILD
              if (debug) Serial.print(":::::: ending sense, abs(dist) = ");
              if (debug) Serial.print(abs(dist));
              if (debug) Serial.print(" [avg. ");
              if (debug) Serial.print(avg_sens_widths[letter]);
              if (debug) Serial.println("] ::::::");

              foundone = true;
              goto endsense;
            }
          }
        } // end trailing edge
      }

      p1 = c1;
      p2 = c2;
    } // end while

  }


 endsense:
  if (debug) aprintf("p sensors: %d %d\n", s1, s2);
  if (foundone) {
    target = ss + (dist)/2;
    master_controllers[letter]->stop();
    master_steppers[letter]->setTargetAbs(target);
    if (debug) aprintf("------> endsense. dist: %d, target: %d. ",
                       dist,
                       target
                       );
    master_controllers[letter]->move(*master_steppers[letter]);
  }
  else {
    if (debug) aprintf("------> couldn't find one, returning to %d. ", positions[letter]);
    master_controllers[letter]->stop();
    master_steppers[letter]->setTargetAbs(positions[letter]);
    master_controllers[letter]->move(*master_steppers[letter]);
  }

    if (debug) aprintf("sensors: %d %d\n", sense(lpins[letter][3]), sense(lpins[letter][4]));

    int c = (foundone) ? c = s1 + (s2 * 2 + 2) : 2;

  /* int c = s1 + (s2 * 2 + 2); */
  if (debug) aprintf("current sensor sum = %d; ", c);

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

  if (certainties[letter] == 1) master_steppers[letter]->setPosition(thetas[letter]*whole_revs[letter]);

  /* if (certainties[letter] < 2) { */
  /*   if (debug) Serial.print("-------> end of sensor/pair. distance: "); */
  /*   if (debug) Serial.print(dist); */
  /*   if (debug) Serial.print(", theta: "); */
  /*   if (debug) Serial.println(thetas[letter]); */
  /*   if (debug) Serial.print("=== "); */
  /*   if (debug) Serial.print(target); */
  /*   if (debug) Serial.println(" ==="); */
  /*   if (debug) Serial.print("position set to "); */
  /*   if (debug) Serial.println(master_steppers[letter]->getPosition()); */
  /* } */
  /* else { */
  /*   if (debug) Serial.println("no sensor found, returning to start."); */
  /*   master_controllers[letter]->stop(); */
  /*   master_steppers[letter]->setTargetAbs(positions[letter]); */
  /*   master_controllers[letter]->moveAsync(*master_steppers[letter]); */
  /* } */


  positions[letter] = master_steppers[letter]->getPosition();

  if (debug) aprintf("letter %d calibrated. theta = %f, position = %d\n",
                     letter, thetas[letter], positions[letter]);

  need_to_log = true;
  sd_log = true;
  started = false;


  /* home(letter); */

} // end calibrate2