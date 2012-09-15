#!/usr/bin/env python
"""
   pid_velocity - takes messages on wheel_vtarget 
      target velocities for the wheels and monitors wheel for feedback
"""

import rospy
import roslib

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from numpy import array

wheel_latest = 0
target = 0
wheel_prev = 0
integral = 0
previous_error = 0
ticks_since_target = 0

def wheelCallback(msg):
    global wheel_latest
    wheel_latest = msg.data
    
def targetCallback(msg):
    global target
    global ticks_since_target
    target = msg.data
    ticks_since_target = 0
    
def do_pid():
    global then
    dt_duration = rospy.Time.now() - then
    dt = dt_duration.to_sec()
    global integral, previous_error, wheel_prev
    global prev_vel
    then = rospy.Time.now()
    
    cur_vel = (wheel_latest - wheel_prev) / dt
    prev_vel.append(cur_vel)
    del prev_vel[0]
    p=array(prev_vel)
    vel = p.mean()
    
    
    wheel_prev = wheel_latest
    error = target - vel
    integral = integral + (error*dt)
    derivative = (error - previous_error) / dt
    previous_error = error
    
    motor = (Kp * error) + (Ki * integral) + (Kd * derivative)
    
    if motor > out_max:
        motor = out_max
        integral = integral - (error*dt)
    if motor < out_min:
        motor = out_min
        integral = integral - (error*dt)
        
    if (target == 0):
        motor = 0
    
    # rospy.loginfo("vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f ## motor:%d " % 
    #              (vel, target, error, integral, derivative, motor))
    
    
    pub_motor.publish(motor)
    
    
    
if __name__ == '__main__':
    """ main """
    global then 
    global ticks_since_target
    global integral
    then = 0
    rospy.init_node("pid_velocity")
    nodename = "pid_velocity"
    rospy.loginfo("%s started" % nodename)
    
    Kp = rospy.get_param('~Kp',10)
    Ki = rospy.get_param('~Ki',10)
    Kd = rospy.get_param('~Kd',10)
    out_min = rospy.get_param('~out_min',-255)
    out_max = rospy.get_param('~out_max',255)
    rate = rospy.get_param('~rate',20)
    rolling_pts = rospy.get_param('~rolling_pts',20)
    timeout_ticks = rospy.get_param('~timeout_ticks',20)
    prev_vel = [0.0] * rolling_pts
    rospy.loginfo("%s got Kp:%0.3f Ki:%0.3f Kd:%0.3f" % (nodename, Kp, Ki, Kd))
    
    rospy.Subscriber("wheel", Int16, wheelCallback) 
    rospy.Subscriber("wheel_vtarget", Float32, targetCallback) 
    pub_motor = rospy.Publisher('motor_cmd', Int16)
   
    r = rospy.Rate(rate) 
    then = rospy.Time.now()
    l_previous_error = 0
    r_previous_error = 0
    ticks_since_target = timeout_ticks
    
    while not rospy.is_shutdown():
        while not rospy.is_shutdown() and ticks_since_target < timeout_ticks:
            do_pid()
            r.sleep()
            ticks_since_target += 1
            if ticks_since_target == timeout_ticks:
                pub_motor.publish(0)
                prev_vel = [0.0] * rolling_pts
                integral = 0
        r.sleep()
    