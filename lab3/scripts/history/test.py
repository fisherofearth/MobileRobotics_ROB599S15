#!/usr/bin/env python

import rospy
import Image

from math import sin 
from math import cos
import numpy

if __name__ == '__main__':

    mapData =  numpy.ones((100,200))

    rospy.init_node('mapping')


    mapData[0][0] = 0.1
    print(str(mapData.shape[0]))
    print(str(mapData.shape[1]))

    a = 0
    a += 1
    print(a)
    point = []
    point.append((1,2))
    point.append((3,4))
    point.append((5,6))
    for p in point:
	
	print(p)

    #img.show()
    #rospy.spin()
	
    
