#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import Twist

def callback(msg):
    print('======================================')
    # for i in range(220, 300):
    #     print ('s ' + str(i))
    #     print msg.ranges[i]
    print msg.ranges[170]

rospy.init_node('laser_data')
sub = rospy.Subscriber('scan', LaserScan, callback)

rospy.spin()
