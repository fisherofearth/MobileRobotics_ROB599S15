#!/usr/bin/env python

# Every python controller needs this line
import rospy

# The velocity command message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

dist = 0
stopT = 0.5 #set the stop threshold 
speed = 0.5

def callback(data):
    global dist
    dist = min(data.ranges)
     


if __name__ == '__main__':
    global dist
    global stopT 
    global speed
   
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

 	print(dist)
	if(dist < stopT):
	    
	    command.linear.x = speed * ((dist - stopT) / 0.5)
	    #command.linear.x  = 0
	else:
	    if(dist < stopT+0.5):
		command.linear.x = speed * ((dist - stopT) / 0.5)
	    else:
	        command.linear.x = speed
	pub.publish(command)

	rate.sleep()
    
   
