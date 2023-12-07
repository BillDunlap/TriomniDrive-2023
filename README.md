# TRIOMIDRIVE-2023

This project contains code for a triagular robot with three powered omniwheels.  The code uses the 2023 version of Wpilib.  The present file is a simple example of Markdown.

## Motors and controllers
The three drive motors are large CIM motors and they are controlled by Talon SRX motor controllers.

## LEDs
The code also has support for a 60-lamp Neopixel [e.g., https://www.adafruit.com/product/4689] strip.  Its power leads should be plugged into the 5 volt, 2 amp part of the voltage regulation module and its signal lead should be plugged into the Roborio's PWM slot 0.

## Camera and Apriltags
The various python files are for a Raspberry PI 3 that is attached via ethernet to the Roborio (using an ethernet hub so the radio can also be connected).  One python program just provides a camera feed, while the others look for Apriltags and report their positions via Network Tables.
