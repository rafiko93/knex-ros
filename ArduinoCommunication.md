| Prev:  [InstallingKnexRos](InstallingKnexRos.md)| [-Top-](ProjectOverview.md) | Next:  [wheelEncoders](wheelEncoders.md) |
|:------------------------------------------------|:----------------------------|:-----------------------------------------|

# Introduction #

Now, let's get ROS talking to the Arduino.


# Overview #

First you'll need to program the complete Arduino code. You can get the code from.
## K'nex Arduino Features ##
  * Supports a dual h-bridge circuit on pins A2, 4, 6, and A0, 7, 5 (see below for pinout)
  * Supports up to 5 servos on pins 10-14
  * Supports 2 wheel encoders on pins 2 and 3
  * An analog range finder on pin A5
  * An analog monitor on pin A4 for oscilloscope-like debugging

More details on how the Arduino code works are listed below.  For now you'll just want to make sure you've got your dual H-bridge circuit hooked up as shown on in the [MotorsToArduino](MotorsToArduino.md) page.

Get the code with:
```
cd {wherever you want to put this code}
git clone https://code.google.com/p/knex-arduino
```

Then use the Arduino IDE or Eclipse to upload the code to the arduino.  Once the code is uploaded, make sure the Arduino is connected and test the communication:

```
roscore
```

In another terminal, type:

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```


You should see:
```
[INFO] [WallTime: 1355284161.246447] ROS Serial Python Node
[INFO] [WallTime: 1355284161.250347] Connected on /dev/ttyACM0 at 57600 baud
[INFO] [WallTime: 1355284163.371418] Setup Publisher on arduino_debug [std_msgs/String]
[INFO] [WallTime: 1355284163.375902] Setup Publisher on lwheel [std_msgs/Int16]
[INFO] [WallTime: 1355284163.381377] Setup Publisher on rwheel [std_msgs/Int16]
[INFO] [WallTime: 1355284163.386614] Setup Publisher on range [std_msgs/Int16]
[INFO] [WallTime: 1355284163.394238] Setup Publisher on scope [std_msgs/Int16]
[INFO] [WallTime: 1355284163.398891] Setup Subscriber on rmotor_cmd [std_msgs/Float32]
[INFO] [WallTime: 1355284163.407153] Setup Subscriber on lmotor_cmd [std_msgs/Float32]
[INFO] [WallTime: 1355284163.414809] Setup Subscriber on servo1_cmd [std_msgs/Int16]
[INFO] [WallTime: 1355284163.422793] Setup Subscriber on servo2_cmd [std_msgs/Int16]
[INFO] [WallTime: 1355284163.426593] Setup Subscriber on servo3_cmd [std_msgs/Int16]
[INFO] [WallTime: 1355284163.435279] Setup Subscriber on servo4_cmd [std_msgs/Int16]
[INFO] [WallTime: 1355284163.443304] Setup Subscriber on servo5_cmd [std_msgs/Int16]
```

Check the output in another terminal with:
```
rostopic echo /arduino_debug
```

You should get:
```
data: tick 324
---
data: tick 325
---
data: tick 326
---
data: tick 327
---
data: tick 328
---
data: tick 329
---
```

This is the Arduino code ticking to let you know it's alive.

In yet another terminal, launch the GUI with:

```
rosrun knex_ros knex_arduino_connector.py
```

You should see the gui again, but this time the left and right sliders should work.  The sliders are connected to directly output the /lmotor\_cmd and /rmotor\_cmd topics.

<a href='http://www.youtube.com/watch?feature=player_embedded&v=fhDVhsWsJ8k' target='_blank'><img src='http://img.youtube.com/vi/fhDVhsWsJ8k/0.jpg' width='425' height=344 /></a>

See http://code.google.com/p/knex-arduino/wiki/knexArduinoDoc for more detailed documentation on the Arduino code.

At this point the robot has no idea where it is, and we cannot even tell how fast the wheels are turning.  It's time for our first sensors.  We're going to need some [wheel encoders](wheelEncoders.md).