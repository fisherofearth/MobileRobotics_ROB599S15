#!/usr/bin/env python

# Every python controller needs this line
import rospy

# The velocity command message
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid


import message_filters
from tf.transformations import euler_from_quaternion

from PIL import Image
import numpy
import numpy as np
from numpy.random import random_sample
import math
import random
# from math import sin
# from math import cos

# from math import tanh
#import numpy as np

sampleNumb = 100
# samplePoses = numpy.ones((sampleNumb,3)) *1.0
# sampleProb = numpy.ones((sampleNumb))*1.0
sample = numpy.ones((sampleNumb,4)) *1.0

sensorModelMean =  numpy.ones((50,50,40))*1.0
map2D =  numpy.ones((50,50))*1.0

markerArray = MarkerArray()
pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)

pose_last = [0.0, 0.0, 0.0]
pose_current = [0.0, 0.0, 0.0] 



#calculate the angle from orientation
def GetTheta(ox, oy,oz, ow):
    quaternion = (ox, oy,oz, ow)
    euler = euler_from_quaternion(quaternion)
    theta = euler[2]
    return theta

def InitMap():
    rospy.Subscriber('map',OccupancyGrid,callback, queue_size= 100) 
def callback(map):# !!!can this fn run forever?
    for x in range(50):
        for y in range(50):
            counter = 0
            for rx in range(10):
                for ry in range(10):
                    if(map.data[(((y*10)+ry)*500)+((x*10)+rx)] == 0.0): 
                        counter += 1
            if(counter >= 60):#if not occupancied
                map2D[x,y] = 0.0
            else:
                map2D[x,y] = 1.0
def CalDist(x1,y1, x2,y2):
    return math.sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))
             
def InitSensorModel():
    for x in xrange(50):
        for y in xrange(50):
            for th in xrange(40):
                for r in range(60):
                    yt = y +  math.sin((float(th)-20.0) * 0.157) * float(r)
                    xt = x + math.cos((20.0 - float(th)) * 0.157) * float(r)
                    sensorModelMean[x,y,th] = CalDist(xt,yt, float(x), float(y))
                    if(map2D[xt,yt] == 1):
                        break;

def InitSamples():
    for n in xrange(sampleNumb): 
        x = round(random.random() * (50.0 - 1.0),0)
        y = round(random.random() * (50.0 - 1.0),0)
        while(map2D[x][y] == 1):
            x = round(random.random() * (50.0 - 1.0),0)
            y = round(random.random() * (50.0 - 1.0),0)
        sample[n,0] = x
        sample[n,1] = y
        sample[n,2] = round(random.random() * (40.0 - 1.0),0)
        sample[n,3] = 1.0 / 100.0

def Normalize(list):
    
    s = float(sum(list))
#     print(list)
#     print(' ---> ' + str(s))
    if(s != 0):
        for n in range(len(list)):
            list[n] = float(list[n]) / s

def Gaussian(x, mean, var):
    return math.exp(-((x-mean)*(x-mean))/(2.0*(var * var))) / (math.sqrt(2.0*3.14*(var * var)))

class MCL:
    def __init__(self):
        self.flg_fistUpdate = 1
        
        odom_sub = message_filters.Subscriber('odom',Odometry)
        scan_sub = message_filters.Subscriber('scan',LaserScan)
        ts = message_filters.ApproximateTimeSynchronizer([odom_sub, scan_sub], 1, 100)
        ts.registerCallback(self.callback)
        
     
           
    def callback(self, odom_msg, scan_msg):
        global pose_last
        global pose_current
        # read the pose
        pose_current[0] = round((odom_msg.pose.pose.position.x+2)*10,0)
        pose_current[1] = round((odom_msg.pose.pose.position.y+2)*10,0)
      
        pose_current[2] = round((GetTheta(
            odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w
            )) / 0.157 ,0) + 20
        
        if(pose_current[0] > 49):pose_current[0] = 49
        if(pose_current[0] < 0):pose_current[0] = 0
         
        if(pose_current[1] > 49):pose_current[1] = 49
        if(pose_current[1] < 0):pose_current[1] = 0
         
        if(pose_current[2] > 39):pose_current[2] = 39
        if(pose_current[2] < 0):pose_current[2] = 0
        
        if(self.flg_fistUpdate == 1):
            self.flg_fistUpdate = 0
            for d in range(3):
                pose_last[d] = pose_current[d]
        
        
        # if movement
        dx = pose_current[0] - pose_last[0]
        dy = pose_current[1] - pose_last[1]
        dth = pose_current[2] - pose_last[2]
        
        if(abs(dx) > 0.0 or abs(dy) > 0.0 or abs(dth) > 0.0):
            
            for d in range(3):
                pose_last[d] = pose_current[d]
#             print('def = ' + str(dx) + ', ' + str(dy) + ', ' + str(dth))
            
            self.Update_Movement([dx,dy,dth])
            self.Update_SensorRead(scan_msg.ranges)
       
       

            
            
            UpdataMarkers()

    def Update_Movement(self, move):
        global sample
        
#         print(sample[0:10, :])
        
        sample = SampleFromPDF(sample, sample[:,3], sampleNumb)
        
        # add noise
        for d in range(3):
            sample[:,d] += move[d]  + np.random.normal(0.0, 0.4,100) # !!! may be out of range
            sample[:,d] = [round(elem,0) for elem in sample[:,d]]
            
        for n in range(sampleNumb):
            if(sample[n,0] < 0):sample[n,0] = 0
            elif (sample[n,0] > 49):sample[n,0] = 49
            
            if(sample[n,1] < 0):sample[n,1] = 0
            elif (sample[n,1] > 49):sample[n,1] = 49
            
            if(sample[n,2] < 0):sample[n,2] = 0
            elif (sample[n,2] > 39):sample[n,2] = 39
        
        Normalize(sample[:,3])
#         print(sample[0:10, :])
        
        
    def Update_SensorRead(self, ranges):
        for sr in range(64):
            idx = (640/2)-32 + sr
            if(ranges[sr*10]< 2.0):
                if not np.isnan(ranges[idx]):
                    for n in range(sampleNumb):
                        x = sample[n,0]
                        y = sample[n,1]
                        angle =  round((-0.5 *(320.0-float(idx))/640.0)/0.157, 0)
                        th = sample[n,2]+ angle
                        if(th>39):th = th-39
                        elif(th<0):th = 39+th
                        mean = float(sensorModelMean[x, y, th])
                        Pzl = Gaussian(ranges[idx]*10, mean, 1.0)
        #                 print('[' +str(x) + '-'+ str(y) + '-'+ str(th) + '] ['+ str(mean) + '-'+ str(Pzl) + ']')
                        sample[n,3] = sample[n,3] * Pzl 
        #                 print(Pzl)
        #             print(sample[0:10,:])
            Normalize(sample[:,3])
 
def InitialMarkers():
    for n in xrange(sampleNumb): 
#         rospy.sleep(0.001)
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
#         marker.header.stamp = rospy.Time.now()
        
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(n) / 20
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        markerArray.markers.append(marker)
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1                                
def UpdataMarkers():
    n = 0
    for m in markerArray.markers: 
        m.pose.position.x = sample[n,0]/10
        m.pose.position.y = sample[n,1]/10
        n += 1
    pub.publish(markerArray)
    
def SampleFromPDF(values, probs, size):
    bins = np.add.accumulate(probs)
    return values[np.digitize(random_sample(size),bins)]
    
    
if __name__ == '__main__':
     
    rospy.init_node('mcl')
    
    InitialMarkers()
    rospy.sleep(0.5)#need this delay for displaying markers, dont know why
    pub.publish(markerArray)
    

    InitMap()   #init a 50x50 map
    rospy.sleep(0.5)
    InitSamples()   # randomly get 100 sample with uniformal prob.
    UpdataMarkers()
    
    
    
    InitSensorModel()# init sensor model - for each pose compute the mean of sensor model
  
    print('start')
#     print(map2D[0:10,0:10])
#     print(sensorModelMean[0:10,0:10,0])


#     array1 = numpy.ones(3,2) *2.0
#     array2 = numpy.ones(3)*1.0
#     
#     print(array1)
#     print(array2)
#     
#     array3 = np.concatenate((array1, array2), axis =1)
#   
#     print(array3[0][0])
#     print(array3[0][1])
#     print(array3[0][2])
#     print(array3)
#     print()
    
    mcl = MCL()

    rospy.spin()

