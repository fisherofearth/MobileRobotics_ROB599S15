#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from math import tanh
import sys, termios, tty, select

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speed = 0.3
flg_return = False
distance = 0.5

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def NavigateTo(targetPose):
	planner = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size = 10)
	rospy.sleep(1)
	targetPose.header.stamp = rospy.Time.now()
	targetPose.header.frame_id = 'map'
	planner.publish(targetPose)

class Driver:

    def __init__(self):
		global speed
		global flg_return 		
		global distance
		self.flag_quit = 0
		
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
	 	self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, msg):
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0
		
		flg_return = rospy.get_param('ret')
		speed = rospy.get_param('speed')
		distance = rospy.get_param('dist')

		if flg_return == True:
			targetPose = PoseStamped()
			targetPose.pose.position.x = 2.0
			targetPose.pose.position.y = 2.0
			targetPose.pose.position.z = 0.0
			targetPose.pose.orientation.x = 0.0
			targetPose.pose.orientation.y = 0.0
			targetPose.pose.orientation.z = 0.0
			targetPose.pose.orientation.w = 0.99
			NavigateTo(targetPose)
			flg_return = False
		elif self.flag_quit == 0:
			key = getKey()
			if key in moveBindings.keys():
				mo = moveBindings[key][0]
				tu = moveBindings[key][1]
			else:
				mo = 0
				tu = 0
				if (key == '\x03'):
					rospy.signal_shutdown('END')		
			if tu == 0:
				error = float(min(msg.ranges[319-220:320+220]) - distance)	
			elif tu < 0:
				error = float(min(msg.ranges[0:319+100]) - distance) * 2
			elif tu > 0:
				error = float(min(msg.ranges[320-100:639]) - distance) * 2

			if error < 0:
				command.linear.x = speed * tanh(error) - 0.1
			else:
				command.linear.x = 0.4 * mo
				command.angular.z = tu * 0.5
			self.pub.publish(command)
	

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('Driver')

	speed = 0.5
	dist = 0.5

	Driver()

	rospy.spin()
	

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

