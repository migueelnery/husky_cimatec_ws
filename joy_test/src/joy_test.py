#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('joy_test')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tf.msg import tfMessage

import sys, select, termios, tty

def callback(data):
    if data.buttons[4] == 1:
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.angular.z = data.axes[0]
        pub.publish(twist)
    
# Intializes everything
def start():
    global pub
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    rospy.set_param('/joy_node/autorepeat_rate', 10)
    start()