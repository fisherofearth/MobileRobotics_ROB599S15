#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sys import argv

x = 0;
y = 0;
az = 0;


def callback2(data):
    x = data.pose.pose.position.x 
    y = data.pose.pose.position.y
    record.write(str(x+2) + ", " + str(y+2))
    record.write("\n")

if __name__ == '__main__':

    filename = '/home/fisher/catkin_ws/src/lab2/scripts/' + raw_input('Enter a filename ')+'.txt'
    rospy.init_node('record')

    record = open(filename, 'w')

    rospy.Subscriber('odom', Odometry, callback2,queue_size= 100)
	
    rospy.spin();
    
