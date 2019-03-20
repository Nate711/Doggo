# Stanford DOGGO
## Overview Of Stanford Doggo
This is the code for the Stanford Doggo quadruped robot. Stanford Doggo is a highly agile robot designed to provide an accessible platform for legged robot research. Currently, the robot holds the record (among all robots) for greatest vertical jumping agility<sup>1</sup>. Stanford Doggo can also jump twice as high as any existing quadruped robot. Weighing in at a little less than 5kg, Stanford Doggo is easy and safe to develop on, but at the same time, Stanford Doggo should not be expected to carry heavy loads or climb extremely aggressive terrain. The project is supported by Stanford Student Robotics http://roboticsclub.stanford.edu/.

<sup>1</sup>[Vertical jumping agility] = [maximum vertical jump height] / [time from onset of actuation to apogee of jump]

## Overview of the Firmware
The Stanford Doggo firmware (contained here), runs on a Teensy 3.5 microcontroller. The code is Arduino-based, which we hopes lowers some barriers to entry, and is simple in structure. While the robot currently lacks advanced features like autonomous navigation or whole-body kinematic control, Stanford Doggo is exceptional at the basic behaviors like trotting and jumping. As we continue to work the robot, we hope to gradually add more sophisticated features that expand the domain of what's possible. Some of the current projects include: Sensing ground reaction forces, detecting obstacles via the natural leg compliance, acrobatic manuevers, and many more.

## Code Requirements
To download the necessary submodules, run the following shell command from the Doggo directory:
```
git submodule init
git submodule update
```
This should download the ChRt library (https://github.com/Nate711/ChRt) to the lib/ directory.

## Notes
### Available serial commands
Use a serial monitor (we use the Arduino one) to send over these commands to Doggo in order to set the behavior or to change parameters.
#### Changing behavior
##### General behaviors
'S': Put the robot in the STOP state. The legs will move to the neutral position. This is like an software e-stop.  
'D': Toggle on and off the printing of (D)ebugging values
'R': (R)eset. Move the legs slowly back into the neutral position.

##### Working gaits  
'B': (B)ound. The gait is currently unstable.  
'E': Danc(e). Make the robot do a little bouncy dance.    
'F': (F)lip. Execute a backflip.  
'H': (Hop). Causes the robot to start making small vertical hops.  
'J': (J)ump. A full-torque upwards leap.  
'T': (T)rot. Begin a forward trot. This is currently the only working forward gait.  

##### Available, but not working
'W': (W)alk. Does not work currently.  
'P': (P)ronk. Much like hop, but this one doesn't work.  

#### Changing gait properties
'f {float}': Set the gait frequency in Hz.  
'l {float}': Set the stride length in meters.  
'h {float}': Set the stance height (neutral height) in meters.  
'u {float}': Set the distance (in meters) that the leg travels above the neutral line during a stride.  
'd {float}': Set the distance (in meters) the leg travels below the neutral line during a stride.  
'p {float}': Set the proportion of time which the leg is below the neutral line versus above it.    
#### Changing compliance (gains)
'g {float} {float} {float} {float}': Set the compliance gains. The ordering is {kp_theta} {kd_theta} {kp_gamma} {kd_gamma}. A good default is 'g 80 0.5 50 0.5'.  
