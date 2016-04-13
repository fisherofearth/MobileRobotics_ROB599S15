#!/usr/bin/env python

import rospy
from PIL import Image




if __name__ == '__main__':


    rospy.init_node('mapping')

    img = Image.new("RGB", (640,480), (0,0,0))

    for x in xrange(640):
	for y in xrange(480):
	   
	    img.putpixel((x,y), (x/3,(x+y)/6, y/2))

    img.save("imageTest.png","PNG")

    #rospy.spin()
	
    
