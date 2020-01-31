#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import Twist

def talker():
    rospy.init_node('vel_publisher')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    move = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        move.linear.x=2
        move.angular.z=0
        pub.publish(move)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    
