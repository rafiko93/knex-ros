Jon Stephan

# Introduction #

In an attempt to get my 6 year old son more interested in robotics, I hacked some K'nex motors and hooked them up to an Arduino.  Next I added some servos, then some wheel encoders, and a infrared range finder.  It's taken so long, my son had pretty much lost interest, but it's still a pretty cool robotics project.  This page is an attempt to document how I did it.

This is not really meant to be a detailed step-by-step instruction for you to follow exactly.  My intent is more to show you how I've done it, and maybe save you some time from re-inventing it yourself.


# Contents #
  1. [Hacking the motors](HackingTheMotors.md)
  1. [Connecting the motors to an Arduino](MotorsToArduino.md)
  1. [First Arduino code](FirstArduinoCode.md)
  1. [Connecting the Arduino to ROS](ArduinoRos.md)

# Current status #

As of 9/30/2012, I have the a differential-drive K'nex hacked robot working, communicating with the ROS host.  The joystick and virtual joystick drive work pretty well.  I have an infrared range finder posing as a laser scanner, and the Simultaneous Localization and Mapping library running.  I got all the pieces in place for the ROS navigation stack to work.

A few issues:  1) the infrared scanner creates some strange artifacts when trying to map oblique angles, which really screws up the localization routines.  2) My PID controller has some bugs that need to be worked out, so the navigation routine doesn't work too well.

# Materials #

These are the material required to build the project the way I did.  It's definitely not the only way to do it.

  * Two K'nex motor units
  * Enough K'nex pieces to build a robot.
  * PC running Linux  (preferably Ubuntu. A virtual machine will probably work)
  * An Arduino Uno
  * An Arduino motor shield
  * An infra red range finder
  * The Arduino software
  * Robot Operating System software
  * Basic electronics stuff: soldering iron, solder, wires, etc.
  * Lots of time and patience

Get started: [Hacking the motors](HackingTheMotors.md)