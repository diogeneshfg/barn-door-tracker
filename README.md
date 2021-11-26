# barn-door-tracker
Arduino barn door tracker implementation for the isosceles mount.

The code is free to use and share. but give the credits for those who helped with it.

The code and the mount construction was based in the previous works from:

https://fstop138.berrange.com/2014/01/building-an-barn-door-mount-part-1-arduino-stepper-motor-control/

https://www.nutsvolts.com/magazine/article/january2015_Wierenga

The main libraries were uploaded to maintain compatibility.

This is a simple code I'm not responsable for it's use or anything tha happens with your equipment.

## Usage:
Basically just the follwing parameters need to be changed:
 
MAX_THETA - degrees;

DISTANCE_ROD_TO_HINGE - milimeters;

INITIAL_APERTURE - milimeters;

STEPS_DEGREES - degrees;

MICRO_STEPS_PER_STEPS;

THREAD_ROD_PITCH - milimeters;

## Main Eletronic Parts:

Arduino Uno\
Stepper Motor Nema 17\
Stepper Motor Driver A4988

