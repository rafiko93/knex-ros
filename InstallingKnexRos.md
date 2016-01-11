| Prev:  [ArduinoRos](ArduinoRos.md)| [-Top-](ProjectOverview.md) | Next:  [ArduinoCommunication](ArduinoCommunication.md) |
|:----------------------------------|:----------------------------|:-------------------------------------------------------|

# Introduction #

Once you've got ROS up and running, and you've gone through the ROS tutorials, you're ready to get the knex\_ros package installed.

This assumes you're running Ubuntu (12.04) with ROS fuerte.

# Installation #

First step is to download the knex\_ros package and its dependencies:
```
roscd
sudo apt-get install ros-fuerte-rosserial
rosws set knex_ros --git https://code.google.com/p/knex-ros
roslocate info differential_drive | rosws merge -
rosws update
source ~/fuerte_workspace/setup.sh
roscd knex_ros
rosmake
chmod +x nodes/*.py
```

Check to see if the stack was installed successfully:

```
roscore
```

Open another terminal and type:
```
rosrun knex_ros knex_arduino_connector.py
```

You should see:

![http://knex-ros.googlecode.com/files/img_screenshot_raw_gui.png](http://knex-ros.googlecode.com/files/img_screenshot_raw_gui.png)

If you got this far, then the K'nex ROS package is successfully installed.  Now, let's see if it works. [ArduinoCommunication](ArduinoCommunication.md)