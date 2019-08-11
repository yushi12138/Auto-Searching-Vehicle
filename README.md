# Auto-Searching-Vehicle
Embedded System Project
## Description
This project involves navigating a vehicle through a minefield by using audio
beacons of fixed frequencies located throughout the field. The vehicle will start at a fixed
location in the field. The mission is to locate the “next” audio beacon and steer the vehicle
toward that beacon while monitoring for the next beacon. The vehicle should continue to
search each consecutive beacon until the final beacon is located, indicating the vehicle has
exited the minefield. During the journey, the vehicle may not collide with any of the beacons
and must steer clear

## Components
1. (Motor part): FEETECH 2CH Servo Motor Controller for DC Motor FT-SMC-2CH
2. (Distance detection part): Eleoo HC-SR04 Ultrasonic Module
3. (Control part) teensy 3.0 
4. (Sensor part): microphone

## Reference

### K20 Manual
[K20 Sub-Family Reference Manual](https://www.nxp.com/docs/en/reference-manual/K20P64M72SF1RM.pdf)

### Teensy 3.2 Schematic
![alt text](https://www.pjrc.com/teensy/schematic32.gif)

### Teensy 3.2 PINs
#### Front side
![alt text](https://www.pjrc.com/teensy/card7a_rev1.png)
#### Back side
![alt text](https://www.pjrc.com/teensy/card7b_rev1.png)
