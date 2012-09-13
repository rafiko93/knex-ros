#!/usr/bin/env python
"""
   pid_velocity - takes messages on lwheel_vtarget and rwheel_vtarget as the
      target velocities for the wheels and monitors lwheel and rwheel for feedback
"""

import rospy
import roslib

from std_msgs.msg import Int16

def lwheelCallback(msg):
    lwheel_latest = msg.data
    
def rwheelCallback(msg):
    rwheel_latest = msg.data
    
if __name__ == '__main__':
    """ main """
    rospy.init_node("pid_velocity")
    nodename = "pid_velocity"
    rospy.loginfo("%s started" % self.nodename)
    
    Kp = rospy.get_param('~Kp')
    rospy.loginfo("%s got Kp=%0.3f" % (nodename, Kp))
    
    