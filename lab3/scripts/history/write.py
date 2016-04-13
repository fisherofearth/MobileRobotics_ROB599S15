#!/usr/bin/env python

import rospy
import Image




if __name__ == '__main__':


    rospy.init_node('mapping')

    img = Image.new("RGB", (640,480), (0,0,255))
    for x in xrange(640):
	for y in xrange(480):
	    img.putpixel((x,y), (x/3,y/2,(x+y)/6))
    for x in xrange(640):
	img.putpixel((x,10), (0,0,0))
    for y in xrange(480):
	img.putpixel((10,y), (255,255,255))

    img.save("image.png","png")
    #img.show()
    #rospy.spin()
	
    
