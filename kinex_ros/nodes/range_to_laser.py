#!/usr/bin/env python

"""
   range_to_laser.py - makes the sony IR range finder look like a laser scanner
"""

# see http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Sensors for LaserScan setup

import rospy
import roslib

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from std_msgs.msg import Float32

########################################################
def rangeCallback(msg):
########################################################
    global latest_range
    latest_range = msg.data


########################################################
def doAScan(direction):
########################################################
    scan_rate = 100   # hertz between steps
    
    if direction:
        angle_start = 0
        angle_stop = 180
        angle_step = 1
    else:
        angle_start = 180
        angle_stop = 0
        angle_step = -1
    
    servo_pub.publish( angle_start )
    
    scan = LaserScan()
    num_readings = 0
    scan_time = rospy.Time.now()
    r = rospy.Rate(scan_rate)
    ranges = []
    for angle in range(angle_start, angle_stop, angle_step):
        servo_pub.publish( angle )
        r.sleep()
        # rospy.loginfo("angle %d range:%0.2f" % (angle, latest_range))
        num_readings += 1
        ranges.append(latest_range)
        scan.intensities.append(1)
    
    if not direction:
        ranges.reverse()    
    scan.ranges = ranges
    scan.header.stamp = scan_time;
    scan.header.frame_id = 'base_link'
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = 3.14 / num_readings
    scan.time_increment = (1 / scan_rate)
    scan.range_min = 0.1
    scan.range_max = 100.0
    
    scan_pub.publish(scan)
    

########################################################
if __name__ == '__main__':
########################################################
    """main"""
    rospy.init_node("range_to_laser")
    rospy.loginfo("range_to_laser started")
    roslib.load_manifest('kinex_ros')
    
    rospy.Subscriber("range_filtered", Float32, rangeCallback)
    servo_pub = rospy.Publisher('servo1_cmd', Int16)
    scan_pub = rospy.Publisher('laser', LaserScan)
    
    latest_range = 0.0;
    
    r = rospy.Rate(10)  # hertz
    while not rospy.is_shutdown():
        doAScan(1)
        r.sleep()
        servo_pub.publish( 0 )
        rospy.sleep(1)
        
        #doAScan(0)
        #r.sleep()
        