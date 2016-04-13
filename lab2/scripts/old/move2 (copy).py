#!/usr/bin/env python

# Every python controller needs this line
import rospy

# The velocity command message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sys import argv

dist = 0 

stopT = 0.1 #set the stop threshold 
speed = 0.6 #normal speed
length = 0.24 #length 
accuracy = 0.01 

def callback(data):
    global dist
    dist = min(data.ranges)
     


if __name__ == '__main__':
    global dist
    global stopT 
    global speed
    global accuracy
    mul = 0.0
	
    #filename = raw_input('Enter a filename ')+'.txt'
    rospy.init_node('move')
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    command = Twist()
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0 

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	rospy.Subscriber('/scan', LaserScan, callback,queue_size= 1000)
 	
	if(dist < (stopT + length) +speed):
	    mul = ((dist - (stopT + length)) / speed)
	    command.angular.z = 1 
	    if(abs(mul) < accuracy):
		mul = 0
	else:
	    mul = 1
	    command.angular.z = 0.0
	command.linear.x = speed * mul
	print('dist: ' + str(dist-length) + '     ' + 'speed: ' + str(command.linear.x) )
	#data record
	#record = open(filename, 'w')
	#record.write( + ", " + )
	#record.write("\n")
	#record.close();

	pub.publish(command)
	rate.sleep()
    
   
