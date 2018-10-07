# Stanford DOGGO
## Overview Of Stanford Doggo
This is the code for the Stanford Doggo quadruped robot. Stanford Doggo is a highly agile robot designed to provide an accessible platform for legged robot research. Currently, the robot holds the record (among all robots) for greatest vertical jumping agility<sup>1</sup>. Stanford Doggo can also jump twice as high as any existing quadruped robot. Weighing in at a little less than 5kg, Stanford Doggo is easy and safe to develop on, but at the same time, Stanford Doggo should not be expected to carry heavy loads or climb extremely aggressive terrain. The project is supported by Stanford Student Robotics http://roboticsclub.stanford.edu/.

<sup>1</sup>[Vertical jumping agility] = [maximum vertical jump height] / [time from onset of actuation to apogee of jump]

## Overview of the Firmware
The Stanford Doggo firmware (contained here), runs on a Teensy 3.5 microcontroller. The code is Arduino-based, which we hopes lowers some barriers to entry, and is simple in structure. While the robot currently lacks advanced features like autonomous navigation or whole-body kinematic control, Stanford Doggo is exceptional at the basic behaviors like trotting and jumping. As we continue to work the robot, we hope to gradually add more sophisticated features that expand the domain of what's possible. Some of the current projects include: Sensing ground reaction forces, detecting obstacles via the natural leg compliance, acrobatic manuevers, and many more. 

## Code Requirements
To download the necessary submodules, run the following shell command from the Doggo directory:
```
git submodule update
```
This should download the ChRt library (https://github.com/Nate711/ChRt) to the lib/ directory.
