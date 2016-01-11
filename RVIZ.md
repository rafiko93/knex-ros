| Prev:  [wheelEncoders](wheelEncoders.md)| [-Top-](ProjectOverview.md) | Next:  [next](next.md) |
|:----------------------------------------|:----------------------------|:-----------------------|

# Introduction #

Now that the wheel encoders are working, we should be able to get the odometry working, and the robot should be able to track its location through deductive reckoning.

The [differential drive package](http://www.ros.org/wiki/differential_drive) will do most of the work here, calculating the robot's position from the rotary encoders.  It also provides PID controllers to govern the wheels' velocity in a controlled way.

We've already got everything in blue, now we need to install the orange blocks in the diagram below:

![http://www.ros.org/wiki/differential_drive?action=AttachFile&do=get&target=differential_drive_overview.png](http://www.ros.org/wiki/differential_drive?action=AttachFile&do=get&target=differential_drive_overview.png)


# Details #


## Installation ##

If you've been following knex\_ros tutorial so far, you already have the differential\_drive package installed.  If not, head on over to the [differential\_drive install tutorial](http://www.ros.org/wiki/differential_drive/tutorials/Install) on the ROS wiki and come back here once you're done.

## Setting up the PID controllers ##

This tutorial also lives on the ROS wiki: [PID setup](http://www.ros.org/wiki/differential_drive/tutorials/setup).  Once you've completed that tutorial, you should be able to drive the robot around with the virtual joystick.


<a href='http://www.youtube.com/watch?feature=player_embedded&v=CfXK1_IjPig' target='_blank'><img src='http://img.youtube.com/vi/CfXK1_IjPig/0.jpg' width='425' height=344 /></a>

## Setting up URDF ##

The tutorial for setting up the URDF file is also on the ROS wiki page: [URDF tutorial](http://www.ros.org/wiki/differential_drive/tutorials/setup_urdf)

Once that is complete, you should be able to drive the robot around and it should be tracked in RVIZ:


<a href='http://www.youtube.com/watch?feature=player_embedded&v=CfXK1_IjPig' target='_blank'><img src='http://img.youtube.com/vi/CfXK1_IjPig/0.jpg' width='425' height=344 /></a>