# picod Package

picod is an archive of programs which allow control of the Raspberry Pi Pico microcontroller.


## Features

* reading and writing GPIO singly and in groups
* hardware PWM
* hardware Servo pulses
* Read PWM frequency, dutycycle, and high edges
* callbacks on GPIO level change
* I2C wrapper
* SPI wrapper
* serial link wrapper
* ADC

## Components

The archive contains the following:

* The pico daemon which runs on the Pico. This is a C program (picod.c) which provides a command interpreter. It accepts commands via a serial link.
* The picod Python module to control the Pico. This is a Python program (picod.py) which runs user scripts. It sends commands to the pico daemon via a serial link.
* Example Python scripts.

