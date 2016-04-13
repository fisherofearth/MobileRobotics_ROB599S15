#!/usr/bin/env python

# Every python controller needs this line
import rospy

# The velocity command message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import tanh

     
def mean(data):
    return sum(data)/len(data)
	
flg_obstacle = 0
dir_Obstacle = -1
class Runner:
    def __init__(self, stopDist, max_speed):
	self.stopDist = stopDist
	self.max_speed = max_speed
	
	self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
 	self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, msg):
	global flg_obstacle
	global dir_Obstacle
	command = Twist()
        #command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        #command.angular.z = 0.0

	right = mean(msg.ranges[320+200:639])
	left = mean(msg.ranges[0:319-200])
	
	if(flg_obstacle == 1):
	    command.angular.z = dir_Obstacle
	    if(min(msg.ranges[(320-100):(320+100)]) > 2*self.stopDist):
		flg_obstacle = 0;
			
	else:
	    if(min(msg.ranges[(320-200):(320+200)]) < 0.51):#obstacle-in-front
	        flg_obstacle = 1
		if(msg.ranges[0]<msg.ranges[639]):
		    dir_Obstacle = 1
		else:
		    dir_Obstacle = -1
	    elif(left < 1.2* self.stopDist):#obstacle-to-left
		error = 1.2*self.stopDist-left
		command.linear.x = self.max_speed * tanh(10.0 * error)
		command.angular.z =  (right - left)/(command.linear.x +1)
	    elif(right < 1.2*self.stopDist):#obstacle-to-right
		error = 1.2*self.stopDist - right
		command.linear.x = self.max_speed * tanh(10.0 * error)
		command.angular.z =  (left -right )/(command.linear.x +1)
	    else:# no obstacle
		error = mean(msg.ranges[(320-200):(320+200)]) - self.stopDist
		command.linear.x = self.max_speed * tanh(10.0 * error)

	

	print(str(left) + '   ' + str(right) + '  ' + str(command.angular.z) + '  ' +  str(flg_obstacle))

        self.pub.publish(command)


if __name__ == '__main__':
    rospy.init_node('move')

    speed = raw_input('speed = ')

    runner = Runner(0.5, float(speed))

    rospy.spin()

   
