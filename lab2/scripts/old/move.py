#!/usr/bin/env python


# Every python controller needs this line
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

# The laser sensor message
from sensor_msgs.msg import LaserScan

# mathematical helper functions
from math import tanh


# This class encapsultes all of the functionality for the assignment.
class Stopper:
    def __init__(self, distance, max_speed):
        # Set the stopping distance and max speed
        self.distance = distance
        self.max_speed = max_speed
 
        # A publisher for the move data
        self.pub = rospy.Publisher('/mobile_base/commands/velocity',
                                   Twist,
                                   queue_size=10)

        # A subscriber for the laser data
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, msg):
        # Create an empty Twist message, and set all of the velocities to zero
        command = Twist()
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        
        # Calculate how far from the stopping distance we are, then
        # push this through a hyperbolic tangent function to smooth
        # out the stopping action.  We'll scale the error before
        # applying the tanh, to get a better speed profile.
        error = min(msg.ranges) - self.distance
        command.linear.x = self.max_spedistanceed * tanh(10.0 * error)

        # Publish the twist message to move the robot
        self.pub.publish(command)


if __name__ == '__main__':
    rospy.init_node('move')

    stopper = Stopper(1, 0.2)

    rospy.spin()

