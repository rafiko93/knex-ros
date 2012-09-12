#!/usr/bin/env python

""" 
   filters the range finder data - it seems to be rather noisy
"""
   
import rospy
import roslib
roslib.load_manifest('knex_ros')

from std_msgs.msg import Int16
from std_msgs.msg import Float32
weight = 0.1  # 1 = this value only 0 = prev value only
cur_val = 0
rolling_ave = 0.0


def inputCallback(msg):
    b = 112.7
    m = -1.31
    cur_val = msg.data
    global rolling_ave
    if cur_val < 900:
        rolling_ave = cur_val * weight + rolling_ave * ( 1 - weight )
        rolling_meters = b * rolling_ave ** m
        
        filtered_pub.publish( rolling_meters)
        
    
if __name__ == '__main__':
    """ main"""
    rospy.init_node("range_filter")
    rospy.loginfo("range_filter started")
    
    rospy.Subscriber("range", Int16, inputCallback)
    
    filtered_pub = rospy.Publisher("range_filtered", Float32)
    rospy.spin()