| Prev:  [ArduinoCommunication](ArduinoCommunication.md)| [-Top-](ProjectOverview.md) | Next:  [RVIZ](RVIZ.md) |
|:------------------------------------------------------|:----------------------------|:-----------------------|


# Introduction #

In order for the robot to know where it is, it needs to know how far the wheels have turned.  This is accomplished with rotary encoders.


# Details #

I used some very cheap optical rotary encoders from Robot Shop: http://www.robotshop.com/productinfo.aspx?pc=RB-Cyt-39&lang=en-US .  I mounted the disk on the wheel axle itself.  Unfortunately, this only gives me about 8 pulses per rotation:

<img src='https://lh3.googleusercontent.com/-ZCg80o2iuu8/UMlQXFwo8TI/AAAAAAAADCU/XsNRgjAgdyo/s1073/IMG_20121212_222317.jpg' width='300' height='200' />


I mounted the two sensors on a piece of cut plastic using my handy high-temperature glue gun:

<img src='https://lh6.googleusercontent.com/-JY7Rbhj1If4/UMlQXBt7r7I/AAAAAAAADCg/uYRpYwr0qQE/s1073/IMG_20121212_222539.jpg' width='300' height='200' />

This is what it looks like when it's all put together:

<img src='https://lh3.googleusercontent.com/-0c0Aj97owew/UMlQXGoY-HI/AAAAAAAADCY/khjE7DayJjE/s1073/IMG_20121212_223654.jpg"' width='300' height='200' />

The schematic is as simple as connecting the powers and grounds up and the signal pins to the Arduino pins 1 and 2.


# Connecting the encoders to ROS #

Launch the nodes the same way as last time:

```
roscore
```

In another terminal, type:

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

In yet another terminal, launch the GUI with:

```
rosrun knex_ros knex_arduino_connector.py
```

To monitor the output of the rotary encoders, launch rxplot:
```
rxplot /lwheel/data /rwheel/data -p 100
```

<a href='http://www.youtube.com/watch?feature=player_embedded&v=weg6feTEobU' target='_blank'><img src='http://img.youtube.com/vi/weg6feTEobU/0.jpg' width='425' height=344 /></a>

Now that the hardware and basic communication works, it's time to get everything hooked up to [RVIZ](RVIZ.md).