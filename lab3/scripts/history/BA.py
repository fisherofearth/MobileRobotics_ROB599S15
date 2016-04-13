#!/usr/bin/env python

import rospy
import Image
import numpy

imageArray = numpy.zeros((640,480))

def draw_line( x0, y0, x1, y1):
    global img,imageArray;
    steep = abs(y1 - y0) > abs(x1 - x0)
    if steep:
        x0, y0 = y0, x0  
        x1, y1 = y1, x1
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
    deltax = x1 - x0
    deltay = abs(y1 - y0)
    error = 0
    y = y0
    for x in range(x0, x1):
	if steep:
	    imageArray[y][x] = 255
	else:
	    imageArray[x][y] = 255

        if (error << 1) >= deltax:
            y = y + 1
            error = error + 2*(deltay - deltax)
        else:
            error = error + 2*deltay

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
	    imageArray[y][x] = 255
        else:
	    imageArray[x][y] = 255
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax


if __name__ == '__main__':


    rospy.init_node('mapping')
    img = Image.new("RGB", (640,480), (0,0,0))

    #draw_line(90,10,100,100)
    #draw_line(100,10,100,100)
    #draw_line(120,10,100,100)
    get_line(100.5,100,120,120)
    get_line(100,100,100,30)
    get_line(100,100,60,200)
    get_line(100,100,80,30)

    for x in xrange(640):
	for y in xrange(480):
	    col = int(imageArray[x][y])
	    img.putpixel((x,480-1-y), (col,col,col))

    img.save("imageTest","png")

    #rospy.spin()
	
    
