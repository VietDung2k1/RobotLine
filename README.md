# Line Following Robot

Simple source code for a line following robot using Arduino.

## Features

* Read 8 line sensors
* Control left/right motors with PWM
* Basic PD control algorithm
* Auto stop when start signal is disabled
* Serial debug support

## Hardware

* Arduino-compatible board
* 8 line sensor inputs
* 2 DC motors
* Motor driver
* Start/stop control using analog input

## Control Logic

The robot reads the line sensor array, calculates the line position error, then adjusts left and right motor speeds using a simple PD controller.

## Build & Upload

Open the source code in Arduino IDE, select the correct board and port, then upload.

![Demo](doc/demo2.gif)
