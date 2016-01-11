| Prev:  [FirstArduinoCode](FirstArduinoCode.md)| [-Top-](ProjectOverview.md) | Next:  [InstallingKnexRos](InstallingKnexRos.md) |
|:----------------------------------------------|:----------------------------|:-------------------------------------------------|

# Connecting the Arduino to ROS #

## Table of contents: ##


# Introduction #

The Robot Operating System [www.ros.org] contains a lot of the basic infrastructure to build your robot on.  It also contains many packages contributed by many sources.  There is a bit of a learning curve, and it's kind of a pain to get started with, but it will save you time in the long run.

# Prerequisites #

Before trying to use my code you will want to:
  * Be running Linux (ROS prefers Ubuntu)
    * If you're running windows, [Ubuntu here](http://www.ubuntu.com/download/desktop/windows-installer|get).  It's not that hard to install, and Windows and Ubuntu can peacefully coexist.
  * Install ROS following these instructions: http://www.ros.org/wiki/ROS/Installation
    * Make sure to follow all the way to the end to set up your .bashrc, and installing rosdep.
  * Go through the beginner and intermediate ROS tutorials: http://www.ros.org/wiki/ROS/Tutorials
    * The first "Installing and configuring ROS" is particularly important.
    * Don't miss the "Creating an Overlay" step.
  * Install the rosserial\_arduino package and install the Arduino IDE (`sudo apt-get install arduino`)
    * Possibly go through the [Arduino IDE Setup](http://www.ros.org/wiki/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) and [Hello World](http://www.ros.org/wiki/rosserial_arduino/Tutorials/Hello%20World) tutorials.


# Tips for getting rosserial\_arduino working #

There were a few things I found I needed to do to get rosserial\_arduino working with Eclipse that weren't listed in the tutorials:

  * Need to edit /usr/share/arduino/libraries/ros\_lib/ArduinoHardware.h and replace WProgram.h with Arduino.h

  * Need to download and install the time library here: http://arduino.cc/playground/Code/Time.  That page doesn't say how to install it:
    * Assuming arduino is installed at /usr/share/arduino
    * unzip Time.zip
    * sudo mv Time /usr/share/arduino/libraries.
    * If you've done it right, you'll see some time examples in the Arduino IDE
    * I had to add to Eclipse paths:
      * Project -> Properties
      * C/C++ general -> Paths and Symbols -> Source Location -> Link Folder
      * Folder name: Time
      * Check Link to a folder in the file system
      * ArduinoLibPath/Time

# Features of knex\_arduino #
  * knex\_arduino\_connector GUI:
    * Simple slider interface to the left and right motors.
    * Simple sliders for each of 5 servos.
    * Simple speed and angle sliders to control motors.
    * Simple arduino debug message output text box.
    * Interface to joystick for motor and servo 1 & 2 control.
  * Interfaces to the `differential_drive` package.
    * Compatible with the ROS navigation stack.
    * Decodes wheel encoder values and provides TF.
    * Implements a PID controller for the wheel motors.
    * Includes a simple virtual joystick.
    * See http://www.ros.org/wiki/differential_drive for more information.
  * range\_filter provides connection to an IR range finder.
  * range\_to\_laser combines the range\_filter information with servo control to emulate a laser range finder.

Ready?  Get started with [Installing knex\_ros](InstallingKnexRos.md)