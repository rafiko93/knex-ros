#!/usr/bin/env python

""" 
   filters the range finder data - it seems to be rather noisy
"""
   
import rospy
import roslib
roslib.load_manifest('kinex_ros')

from std_msgs.msg import Int16
from std_msgs.msg import Float32
weight = 0.1  # 1 = this value only 0 = prev value only
cur_val = 0
rolling_ave = 0.0


def inputCallback(msg):
    cur_val = msg.data
    global rolling_ave
    if cur_val < 500:
        rolling_ave = cur_val * weight + rolling_ave * ( 1 - weight )
        filtered_pub.publish( rolling_ave)
        
    
if __name__ == '__main__':
    """ main"""
    rospy.init_node("range_filter")
    rospy.loginfo("range_filter started")
    
    rospy.Subscriber("range", Int16, inputCallback)
    
    filtered_pub = rospy.Publisher("range_filtered", Float32)
    rospy.spin()