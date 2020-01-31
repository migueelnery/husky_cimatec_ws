#!/usr/bin/env python  
import roslib
roslib.load_manifest('joy_test')
import rospy
from math import sqrt, atan2, pow
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


if __name__ == '__main__':
    rospy.init_node('turtle_tf')
    turtle_vel = rospy.Publisher('/move_base_simple/goal', PoseStamped ,queue_size=10)
    goal = PoseStamped()
    while not rospy.is_shutdown():
        goal.pose.position.x = input("Posicao X: ")
        goal.pose.position.y = input("Posicao Y: ")
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.header.frame_id = "odom" 
        goal.header.stamp = rospy.Time()
        turtle_vel.publish(goal)