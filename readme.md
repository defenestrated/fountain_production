# 1098 Alta Orrery Fountain
### Muse & Co. //  Google
### installed summer 2021
---
This kinetic sculpture is based on the idea of an orrery, a mechanical model that depicts the movements of celestial bodies. Humans have been making these devices for thousands of years – the oldest known orrery dates back to c. 125 BCE. Early orreries relied on clockwork and hand-cut gears to drive the planets around their orbits; ours uses embedded computing and digital fabrication to illustrate the complexity and interconnectedness of the googleverse.

All six letters are coordinated, moving at relative speeds in classical harmonic ratios to each other: 1:2, 2:3, 1:3, etc. The diameter of each ring is proportional to the rotational speed.

Each ring is driven by its own dedicated stepper motor, so although the rings _appear_ to be geared together, they spin independently. For this reason, all accelerations and speeds must be carefully calculated so as to avoid collisions.

Here are some characteristics of how the rings relate to each other, with the stepper drivers set to 32 microsteps:
| letter | rotations per cycle | steps per one rotation* |
|:------:|:-------------------:|:-----------------------:|
| `G`    | 6                   | 29125                   |
| `o1`   | 3                   | 60069                   |
| `o2`   | 12                  | 14070                   |
| `g`    | 4                   | 44375                   |
| `l`    | 24                  | 1500                    |
| `e`    | 6                   | 29875                   |

*For some reason, these differ in real life from what they should be based on the gear ratios. Something got lost in translation… these values are measured manually on the real build.


## HARDWARE SETUP:
- 2x [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)
- 3x [74HCT245](https://www.digikey.com/en/products/detail/texas-instruments/SN74HCT245N/277258) level shifters (on custom pcb, to interface 3.3v Teensys with 5v stepper drivers)
- 10x custom hall sensors (2 on each of the first four letters, 1 on each of the last two)*
- 1x 2032 lithium coin cell battery (to keep the clock going in the event of power loss)
- 6x [NEMA 23 bipolar stepper motors](https://www.amazon.com/gp/product/B00PNEPW4C) powered via [Gecko high-resolution drivers](https://www.geckodrive.com/gr214v-bulletproof-high-resolution-stepper-drive.html)
- 1x [meanwell 5v 10a power supply](https://www.jameco.com/z/MDR-60-5-MEAN-WELL-AC-to-DC-DIN-Rail-Power-Supply-5-Volt-10-Amp-50-Watt_1943449.html) (for logic)
- 3x [meanwell 48v 10a power supplies](https://www.digikey.com/en/products/detail/mean-well-usa-inc/NDR-480-48/7705225) (for steppers)
- 1x [meanwell 24v 2.5a power supply](https://www.digikey.com/en/products/detail/mean-well-usa-inc/MDR-60-24/7705073) (for relays and e-stop)
- misc. switches and relays

note: to connect the Teensys, you’ll need two long usb micro-b cables, like [this](https://www.amazon.com/TetherPro-USB-C-Micro-B-Visibility-Orange/dp/B0794FX9DB/), and [TyCommander](https://github.com/Koromix/tytools) if you want to watch both serial ports simultaneously. A macbook pro won’t recognize the Teensys unless you shut off the main power before plugging them in (has to do, I think, with how USB works, but I’m not sure why exactly). I can always get them both to show up after a few tries plugging and unplugging them in various sequences.

*on the first four letters, the two hall sensors are positioned 90º away from each other. This enables the calibration code to look for states of `1 1` (home), `0 1` (90º), `0 0` (in the 180º quadrant), or `1 0` (270º). There are small (~1/4”) circles on the outer edges of the rings where the magnets are inserted.


## SOFTWARE SETUP:
The software is structured in C++ form, built with [Platformio](https://platformio.org/) rather than the typical Arduino IDE set of `.ino` files. The relevant structure is as follows:
```bash
fountain_v4
├── firmware
│   ├── primary
│   │   ├── debug_noloop.elf
│   │   └── nodebug_loop.elf
│   └── secondary
│       ├── debug_noloop.elf
│       └── nodebug_loop.elf
├── platformio.ini
└── src
    ├── calibrate1.cpp
    ├── calibrate1.h
    ├── calibrate2.cpp
    ├── calibrate2.h
    ├── primary
    │   └── primary.cpp
    ├── secondary
    │   └── secondary.cpp
    ├── utilities.cpp
    └── utilities.h
```

`utilities.cpp` contains functions and global variables shared by both Teensys; all the project-level configuration (start + stop time, debug mode, etc.) is in variables at the top of this file.

`primary.cpp` is the code for the primary Teensy – most of the logic flow lives here.

`secondary.cpp` is, you guessed it, code for the second Teensy.

`calibrate` 1 and 2 are the calibration functions for one- and two-sensor letters, respectively (`G`,`o`,`o`,`g` are all two-sensors, and controlled by the first Teensy, `l` and `e` each have one sensor and are controlled by the second.)

`platformio.ini` configures the platformio build.


Building the project with Platformio will create two firmware files for each of the Teensys in `.pio/build/primary` and `.pio/build/secondary`, respectively. These can then be uploaded simultaneously via [TyCommander](https://github.com/Koromix/tytools) (which is an amazing tool for lots of reasons, incl. simultaneous serial monitoring of two ports).

For convenience, I’ve left copies of the firmware for each Teensy in the `firmware/` folder, current as of 2021-11-23. There’s one file with `debug = true` and `loopcycle = false`, and one without, for each Teensy. `debug_noloop.elf` will enable debug mode, and prevent automatic looping. `nodebug_loop.elf` is the “production mode,” disabling serial output and looping automatically (during on hours).

## DEBUG MODE:
If, in `utilities.cpp`, `debug` is set to `true`, you can interact with the sculpture via serial commands sent to the primary Teensy. The commands / syntax are as follows:

| key command | function                         | syntax                                        | example                                                                                    |
|:-----------:|:--------------------------------:|:----------------------------------------------|:-------------------------------------------------------------------------------------------|
| `space`     | software e-stop                  |                                               |                                                                                            |
| `h`         | soft stop                        |                                               |                                                                                            |
| `p`         | toggle motor enable              |                                               |                                                                                            |
| `q`         | read positions from sd card      |                                               |                                                                                            |
| `Q`         | log current positions to sd card |                                               |                                                                                            |
| `m`         | move one letter to target        | `m [letter] [target]`                         | `m 3 50000` <br />moves second ‘g’ to position 50000<br />letters are zero-indexed.        |
| `w`         | set one letter’s position        | `w [letter] [new position]`                   | `w 4 2000` <br />set position of ‘l’ to 2000                                               |
| `W`         | set all positions to 0           |                                               |                                                                                            |
| `s`         | read sensors of letter           | `s [letter]`                                  | `s 1` <br />outputs sensor readings for first ‘o’                                          |
| `l`         | toggle automatic looping         |                                               |                                                                                            |
| `c`         | calibrate single letter*         | `c [letter]`                                  | `c 2` <br />calibrate second ‘o’<br />                                                     |
| `c`         | calibrate all letters*           |                                               |                                                                                            |
| `x`         | initiate full cycle**            |                                               |                                                                                            |
| `X`         | initiate fraction of a cycle\*** | `X [frac] [frac] [frac] [frac] [frac] [frac]` | `X 0.1 0.1 0.1 0.1 0.1 0.1` <br />move all letters in synchrony to 1/10th of a full cycle. |

***CAUTION: the calibration sequence is blocking (meaning, you can’t stop it once it’s going without hitting the e-stop), and will cause the letter to traverse 1/4 turn before either returning to where it started (no sensor found) or returning to home. Make triply sure it won’t hit anything en route!**

To avoid collisions, move the first ‘o’ 1/4 turn in either direction and leave everything else in home position.

__**make sure you’re starting at home or are still perfectly in sync__

__\***these don’t all have to be the same – i.e., you could do `X 0 0.5 0.2 1 0 0` but be REALLY sure you know what you’re doing, because letters will collide if you’re not careful.__ 

This is most useful for resetting the positions; if you start a cycle with `x`, decide you want to stop by sending `h`, you can then do `X 0 0 0 0 0 0` to run the cycle backwards to 0.
