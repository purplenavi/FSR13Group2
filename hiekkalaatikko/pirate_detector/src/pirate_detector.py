#!/usr/bin/env python
import roslib
roslib.load_manifest('pirate_detector')
import Image
import os
import sys
import cv
#from opencv2 import *
#from opencv2.highgui import *
import rospy
import numpy as np
import math
#import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Code samples fetched from http://www.pirobot.org/blog/0016/
# Draw contour from list of tuples.
def draw_contour( im , contour , color , thickness = 1 , linetype = 8 ,
                  shift = 0 ) :
  if thickness == -1 :
    cv.FillPoly( im , [contour] , color , linetype , shift )
  else :
    cv.PolyLine( im , [contour] , True , color , thickness , linetype , shift )

# Rotate contour around centre point using numpy.
def rotate_contour( contour , centre_point , theta ) :
  rotation = np.array( [ [ np.cos( theta ) , -np.sin( theta ) ] , 
                         [ np.sin( theta ) ,  np.cos( theta ) ] ] )
  centre = np.vstack( [ centre_point ] * len( contour ) )
  contour = np.vstack( contour ) - centre
  contour = np.dot( contour , rotation ) + centre
  return [ tuple ( each_row ) for each_row in contour ]

# Find centre of mass by drawing contour in closed form and using moments.
def find_centre_of_mass( contour ) :
  bottom_right = np.max( contour , axis = 0 )
  blank = cv.CreateImage( tuple ( bottom_right ) , 8 , 1 )
  cv.Set( blank , 0 )
  draw_contour( blank , contour , 1, -1 )
  matBlank = cv.GetMat(blank)
  moments = cv.Moments( matBlank , 1 )  
  sM00 = float ( cv.GetSpatialMoment( moments , 0 , 0 ) )
  sM01 = float ( cv.GetSpatialMoment( moments , 0 , 1 ) )
  sM10 = float ( cv.GetSpatialMoment( moments , 1 , 0 ) )
  if sM00 == 0:
    sM00 = 1
  return ( sM10 / sM00 , sM01 / sM00 )

THETA = np.pi / 3.0
COLOR = cv.CV_RGB( 255 , 0 , 0 )

class pirate_detector:
   
    def __init__(self):

        rospy.init_node('pirate_detector')
        print "Init"
        #Give the opencv display window a name
        self.cv_window_name = 'OpenCVImage'
        self.rgb_window_name = 'RGBimage'
        #create the window and make it re-sizeable (second parameter 0)
        cv.NamedWindow(self.cv_window_name)
        cv.NamedWindow(self.rgb_window_name)
        #create the cv_bridge object
        self.bridge = CvBridge()
        #subscribe to image data
        self.image_subscribe = rospy.Subscriber('/camera/rgb/image_mono', Image, self.image_callback)
        self.rgb_subscribe = rospy.Subscriber('/camera/rgb/image_color', Image, self.rgb_callback)
        self.rgb_image = None
        self.cv_image = None
        self.combined = None
        
    def image_callback(self, data):
        print "Callback"
        try:
            #convert image to opencv format
            cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        #cv.ShowImage(self.cv_window_name, cv_image)   
        #TODO: Detect edges, contours
        #Canny detecting
        cv.EqualizeHist(cv_image, cv_image)
        cv.Smooth(cv_image, cv_image, cv.CV_GAUSSIAN, 11, 11)
        yuv = cv.CreateImage(cv.GetSize(cv_image), 8, 3)
        gray = cv.CreateImage(cv.GetSize(cv_image), 8, 1)
        #cv.CvtColor(cv_image, yuv, cv.CV_BGR2YCrCb)
        #cv.ShowImage(self.cv_window_name,cv_image)
        cv.Split(yuv, gray, None, None, None)
        canny = cv.CreateImage(cv.GetSize(cv_image),8, 1)
        cv.Canny(cv_image,canny,50,250)
        storage = cv.CreateMemStorage()
        contour_pointer = cv.FindContours(canny, storage)
        contour_areas = []
        while contour_pointer is not None:
            contour = contour_pointer[ : ]
            if cv.ContourArea(contour) < 50.0:
                pass
            else:
                centre_of_mass = find_centre_of_mass(contour)
                rotated_contour = rotate_contour(contour, centre_of_mass, 0)
                lst = []
                for i in rotated_contour:
                    lst.append(list(i))
                rotated_contour = []
                for i in lst:
                    i[0] = int(i[0])
                    i[1] = int(i[1])
                    rotated_contour.append(tuple(i))
                draw_contour(cv_image, rotated_contour, COLOR, -1)
            contour_pointer = contour_pointer.h_next()

        cv.ShowImage(self.cv_window_name, cv_image)
        cv.WaitKey(0)

    def rgb_callback(self, data):
        print "RGB Callback"
        try:
            rgb_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print "RGB callback failure" + e
        self.rgb_image = rgb_image

        cv.ShowImage(self.rgb_window_name, rgb_image)
        cv.WaitKey(3000)

def main(args):
    pd = pirate_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down pd node.'
    cv.DestroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
