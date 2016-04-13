#!/usr/bin/env python

# Every python controller needs this line
import rospy

from nav_msgs.msg import Odometry
from sys import argv


def callback(data):
    global record
    x = data.pose.pose.position.x 
    y = data.pose.pose.position.y 
    
    record.write(str(x) + ", " + str(y))
    record.write("\n")
    #record.close(); 


if __name__ == '__main__':

    filename = '/home/fisher/catkin_ws/src/lab2/scripts/' + raw_input('Enter a filename ')+'.txt'
    rospy.init_node('record')

    record = open(filename, 'w')

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

	rospy.Subscriber('odom', Odometry, callback,queue_size= 100)

    
   	rate.sleep()
