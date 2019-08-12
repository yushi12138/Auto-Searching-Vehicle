# Auto-Searching-Vehicle
Embedded System Project

## Demonstration
[![Watch the video](https://www.youtube.com/upload_thumbnail?v=odqYxbhW_Ps&t=hqdefault&ts=1565551223037)](https://youtu.be/odqYxbhW_Ps)

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
4. (Sound Sensor part): microphone

## IDE
Arduino IDE

## Connection(PIN numbers are on the reference picture)
- Battery pack: Vin and GND
- Motor Controller: PIN 5 and PIN 6 (which support PWM)
- Sonic Sensor: PIN A0 (which is an anlog pin, can directly use anlogread() function defined by arduino)
- Ultrasonic sensor: (trigPin,echoPin) pairs: (15,16),(0,1),(11,12), in final presentation we use three ultrasonic sensor in total.
- led : PIN 13 on chip
- all the ground pins of components should all connect to the GND of teensy duino.
## Setup
```c

// LED Pin Number
const int led = 13;

// Ultrasonic Sensor pin numbers
const int trigPin = 15;
const int echoPin = 16;
const int trigPin1 = 0;
const int echoPin1 = 1;//left
const int trigPin2 = 11;
const int echoPin2 = 12;

void setup()
{
  //---------------------------------FFT Section------------------------------
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
  Serial.begin(115200);

  //---------------------------------PWM Section------------------------------
  PORTD_PCR4 |= (1U << 10);
  PORTD_PCR7 |= (1U << 10);

  // 50% duty cycle ~ 1.6v with 8 BIT RESOLUTION
  // Bit 3 and bit 4 of status control, select system clock
  FTM0_SC |= _BV(3);
  FTM0_SC &= ~(_BV(4));

  //DISABLE WRITE PROTECTION
  FTM0_MODE |= (1U << 2);     // Mode register
  FTM0_MODE |= _BV(1);        // Enables FTM
  FTM0_SC |= 0x4;

  // Set MSB and ELSB, bit 5 and bit 3, which sets edge aligned PWM mode
  FTM0_C4SC |= (1U << 3) | (1U << 5);
  FTM0_C7SC |= (1U << 3) | (1U << 5);
  // Requirements for edge aligned up counter pwm is done,
  // MSnB 1 (Channel mode select B), QUADEN 0 (Quadrature Decoder),
  // DECAPEN 0 (Dual edge capture), COMBINE 0 (Pair dual channel),
  // CPWMS 0 (center aligned disabled and set as upcounting mode)

  FTM0_CNTIN = 0;         // Initial value is 0 for PWM counter
  FTM0_MOD = 29999;       // Counts up to MOD, 20ms per cycle

  //---------------------------------LED Section------------------------------
  pinMode(led, OUTPUT);

  //---------------------------------Ultasonic Sensor Section-------------------
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
}
```
## Software Implementation
### FFT
### PWM
### Ultrasonic Sensor

## Strategy
1. spin around(fixed angle per time), microphone collects the data of sound in each direction and do fft to figure out the magnitude of already known buzzer frequencies.

2. keep a table of directions of these frequencies when their magnitudes are the largest. And keep another table of these largest magnitudes with respect.

3. find if the magnitude of target frequecy is larger than the threshold decided by me.

4. if so turn to the direction find the target frequency, 
   if not turn to direction in which certain frequency has largest magnitude(which means this buzzer is nearest to the vehicle).
   
5. (Distance Detection)when approaching the buzzer, the ultrasonic sensor will detect the distance between the vehicle and beacon(a plastic mushroom-shaped contaienr where the buzzer put). 
   If the buzzer has target frequency, when the magnitude of target frequency is large enough and the distance is samll enough, the vehicle will stop and the led will start to blink.
   if the buzzer doesn't has the target frequency, when the vehecle can detect the beacon is in the right way the vehicle is moving(based on the ultrasonic sensor in the front of the vehicle) and the distance is small enough, it'll walk around the beacon and set it as visited in the visited table. Next time the vehicle will ignore the magnitude of this frequency and find other unvisited beacon until find the target one. 

## lib
### FFT
https://github.com/kosme/arduinoFFT

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
