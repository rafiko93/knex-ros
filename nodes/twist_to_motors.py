#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
"""

import rospy
import roslib
from std_msgs.msg import Int16
from std_msgs.msg import Twist

#############################################################
def wheelCallback(msg):
#############################################################
    global target
    global ticks_since_target
    ticks_since_garget = 0
    target = msg.data
    
if __name__ == '__main__':
    """ main """
    rospy.init_node("twist_to_motors")
    nodename = rospy.get_name()
    rospy.loginfo("%s started" % nodename)
    
    