#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from PIL import Image

from math import sin 
from math import cos
import numpy

mapData =  numpy.ones((100,100)) * 7

xOld = 0.0
yOld = 0.0
thetaOld = 0.0
updataCounter = 0;

def get_line(x1, y1, x2, y2):
    points = []
    issteep = abs(y2-y1) > abs(x2-x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    deltax = x2 - x1
    deltay = abs(y2-y1)
    error = int(deltax / 2)
    y = y1
    ystep = None
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    for x in range(x1, x2 + 1):
        if issteep:
	    points.append((y, x))
        else:
	    points.append((x, y))
        error -= deltay
        if error < 0:
           y += ystep
           error += deltax
    return points

def CalObjLocation(xRob, yRob, thetaRob, dist, psi):
    xObj = xRob + dist*cos(thetaRob+psi)
    yObj = yRob + dist*sin(thetaRob+psi)
    #print(xObj,yObj)
    return (xObj,yObj)

def UpdateMap(xRob,yRob,theta, ranges, scan_msg):
    global updataCounter

    probObj = numpy.zeros((100,100))#initialize prob. observation
    counter = numpy.zeros((100,100))

    x = xRob*10
    y = yRob*10
    #print(len(ranges))
    for laserIdx in xrange(640):#xrange(len(ranges)):
        dist = ranges[laserIdx]
        dist = dist*10
	
	pp = (320 - float(laserIdx)) / 320
        psi = scan_msg.angle_min * pp
        #print(laserIdx, pp, scan_msg.angle_max, psi)

        #if(dist >= (5*10)):
        #    dist = (5*10) #ignore range out of 5
            
        (xObj,yObj) = CalObjLocation(x,y,theta,dist,psi)
	
        #print(x,y,theta,dist,psi)
	#print(xObj,yObj) 

        x = int(round(x))
        if(x>=100):x= 100-1
        y = int(round(y))
        if(y>=100):y= 100-1
        xObj = int(round(xObj))
        if(xObj>=100):xObj= 100-1
        yObj = int(round(yObj))
        if(yObj>=100):yObj= 100-1
        
        points = []
        points = get_line(x, y, xObj, yObj)#get a grid-line between robot and object
        
        for p in points:
            counter[p[0]][p[1]] += 1
        if(dist<(5*10)):probObj[xObj][yObj] += 1 #if detect object nearer than 5m
	

    for ix in xrange(probObj.shape[0]):
       for iy in xrange(probObj.shape[1]):
           if(counter[ix][iy] > 0):#if sovered by scaner
                probObj[ix][iy] = probObj[ix][iy] / counter[ix][iy] 	 
		if(probObj[ix][iy] >= 0.25):#believe obstacles
		    if(mapData[ix][iy]<=12):mapData[ix][iy] += 3
		else:
		    if(mapData[ix][iy]>0):mapData[ix][iy] -= 1
		
    updataCounter += 1    
    print("update:" + str(updataCounter))
  
    
#calculate the angle from orientation
def GetTheta(ox, oy,oz, ow):
    quaternion = (ox, oy,oz, ow)
    euler = euler_from_quaternion(quaternion)
    theta = euler[2]
    return theta

              
class MAPPER:
    def __init__(self):
    	pose_sub = message_filters.Subscriber('odom',Odometry)
    	scan_sub = message_filters.Subscriber('scan',LaserScan)
    	ts = message_filters.ApproximateTimeSynchronizer([pose_sub, scan_sub], 1, 100)
    	ts.registerCallback(self.callback)

                            
    def callback(self, odom_msg, scan_msg):
    	global xOld, yOld, thetaOld
    	global img,mapData
    
    	x = odom_msg.pose.pose.position.x + 2
    	y = odom_msg.pose.pose.position.y + 2
    	theta = GetTheta(
			odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,
			odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w
			)
	
        
        if(abs(x - xOld)>0.1 or abs(y - yOld)>0.1 or abs(theta - thetaOld)>0.3):
	    ranges = scan_msg.ranges[0:640];
	    #UpdateMap(x,y,theta, ranges, scan_msg) #fun for updating grid map
            UpdateMap(x,y,theta, ranges, scan_msg) #fun for updating grid map
            (xOld, yOld, thetaOld) = (x, y, theta)#save current pose
            for ix in xrange(mapData.shape[0]):
                for iy in xrange(mapData.shape[1]):
                    clr = 255 - int(mapData[ix][iy] * 17)
                    for m in xrange(4):
                        for n in xrange(4):
                            img.putpixel(((ix*4)+m,(100-1-iy)*4 + n), (clr,clr,clr))
	    
            img.save("image.png","png")
 
                	
if __name__ == '__main__':

    img = Image.new("RGB", (400,400), (0,0,0))
    rospy.init_node('mapping')

    mapper = MAPPER()
    
    rospy.spin()
    
    
