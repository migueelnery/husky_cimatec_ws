#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import Twist

def callback(msg):
    move=Twist()
    if msg.ranges[170] > 1.7:
        move.linear.x=1
        move.angular.z=0
    else:
        move.linear.x=0
        move.angular.z=0
    
    pub.publish(move)

rospy.init_node('obstacle_avoidance')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist)

rospy.spin()
        
