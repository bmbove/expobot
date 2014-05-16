Balancing Tekbot
================

This is the code for a balancing robot built for the 2014 Engineering Expo at 
Oregon State University. The aim was to build something interesting using 
hardware that we had used in ECE labs. I chose to play around with the
Teensy 2.0 and the [Tekbot](http://eecs.oregonstate.edu/education/courses/ece272/) kit.

The control board consists of a TB6612FNG motor driver, a MMA7361LC 3-Axis accelerometer,
and a LPY550AL dual-axis gyro under the control of an ATmega328p microcontroller. Also
on board is an nRF24L01+ RF transceiver module.

Though I did initially write my own PID control, I found it was much easier to go with the 
Arduino PID library. I interfaced with the nRF24L01+ using this
[RF24 library](https://github.com/stanleyseow)- the included Raspberry Pi code made adjusting
and testing the PID very easy since there was no serial connection available.

The brains of the wireless controller is the Teensy 2.0 along with another nRF24L01+ module.
I used an old (wired) Xbox controller and soldered leads directly to the pots on one of the
analog sticks. The Teensy's built in 10-bit ADC did the rest of the work.

Note that this code is incomplete and quite messy. I haven't had the chance to clean it up yet
since my initial development and testing.

Libraries used:
*[digitalWriteFast](https://code.google.com/p/digitalwritefast/)
*[Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library/)
*[RF24 Library](https://github.com/stanleyseow)
