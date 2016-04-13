#!/usr/bin/env python

# Every python controller needs this line
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sys import argv

x = 0;
y = 0;
az = 0;


def callback(data):

    rospy.Subscriber('odom', Odometry, callback2,queue_size= 100)
    

def callback2(data):
    global x
    global y
    global az

    xNew = data.pose.pose.position.x 
    yNew = data.pose.pose.position.y 
    #azNew = data.pose.pose.rotation.z 
    
    if(xNew != x and yNew != y):
	x = xNew
	y = yNew
	#az = azNew
	record.write(str(x+2) + ", " + str(y+2))
	record.write("\n")


if __name__ == '__main__':

    filename = '/home/fisher/catkin_ws/src/lab2/scripts/' + raw_input('Enter a filename ')+'.txt'
    rospy.init_node('record')

    record = open(filename, 'w')

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

	rospy.Subscriber('/scan', LaserScan, callback)

	

    
   	rate.sleep()
