#!/usr/bin/env python
#   Copyright 2012 Jon Stephan
#   jfstepha@gmail.com
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

""" 
   Filters IR range finder data, and scales it to meters.
   
"""
   
import rospy
import roslib
roslib.load_manifest('knex_ros')

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from numpy import array

############################################################################## 
############################################################################## 
class RangeFilter():
############################################################################## 
############################################################################## 

    #########################################################################
    def __init__(self):
    #########################################################################
        cur_val = 0

        self.rolling_ave = 0.0
        self.rolling_std = 0.0
        self.rolling_meters = 0.0
        rospy.init_node("range_filter")
        rospy.loginfo("-I- range_filter started")
        self.rolling_pts = rospy.get_param('~rolling_pts',4)
        self.m = rospy.get_param('~exponent', -1.31)
        self.b = rospy.get_param('~coefficient', 266.0)
        self.prev = [0] * self.rolling_pts
    
        rospy.Subscriber("range", Int16, self.inputCallback)
    
        self.filtered_pub = rospy.Publisher("range_filtered", Float32)
        self.std_pub = rospy.Publisher("range_std", Float32)
        while not rospy.is_shutdown():
            rospy.spin()
    

    #########################################################################
    def inputCallback(self, msg):
    #########################################################################
        rospy.loginfo("-D- range_filter inputCallback")
        cur_val = msg.data
    
        if cur_val < 900:
            self.prev.append(cur_val)
            del self.prev[0]
        
            p = array(self.prev)
            self.rolling_ave = p.mean()
            self.rolling_std = p.std()
        
            rolling_meters = self.b * self.rolling_ave ** self.m
        
            self.filtered_pub.publish( rolling_meters )
            self.std_pub.publish( self.rolling_std )
        
    
############################################################################## 
############################################################################## 
if __name__ == '__main__':
############################################################################## 
############################################################################## 
    """ main"""
    rangeFilter = RangeFilter()
    