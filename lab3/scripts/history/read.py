#!/usr/bin/env python

import rospy
import Image




if __name__ == '__main__':


    rospy.init_node('mapping')

   
    img = Image.open("image.png")

    for x in xrange(img.size[0]):
	for y in xrange(img.size[1]):
	    r,g,b = img.getpixel((x,y))
	    img.putpixel((x,y), (b,r,g))

    img.save("imageNEW","PNG")

    #img.show()
    #rospy.spin()
	
    
