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
import numpy
import math
#import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Code samples fetched from http://www.pirobot.org/blog/0016/

class pirate_detector:
   
    def __init__(self):

        rospy.init_node('pirate_detector')
        print "Init"
        #Give the opencv display window a name
        self.cv_window_name = 'OpenCVImage'
        #create the window and make it re-sizeable (second parameter 0)
        cv.NamedWindow(self.cv_window_name)

        #create the cv_bridge object
        self.bridge = CvBridge()
        #subscribe to image data
        self.image_subscribe = rospy.Subscriber('/camera/depth/image', Image, self.image_callback)
        
    def image_callback(self, data):
        print "Callback"
        try:
            #convert image to opencv format
            cv_image = self.bridge.imgmsg_to_cv(data)
        except CvBridgeError, e:
            print e
            
        #TODO: Detect edges, contours
        edges = cv.Canny(cv_image,100,100)
        image_size = cv.GetSize(cv_image)
        image_width = image_size[0]
        image_height = image_size[1]
       
        """ Convert to HSV and keep the hue """
        #hsv = cv.CreateImage(image_size, 8, 3)
        #cv.CvtColor(cv_image, hsv, cv.CV_BGR2HSV)
        #self.hue = cv.CreateImage(image_size, 8, 1)
        #cv.Split(hsv, self.hue, None, None, None)

        """ Compute back projection """
        backproject = cv.CreateImage(image_size, 8, 1)

        """ Run the cam-shift algorithm """
        #cv.CalcArrBackProject( [self.hue], backproject, self.hist )
        #if self.track_window and is_rect_nonzero(self.track_window):
        #    crit = ( cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 10, 1)
        #    (iters, (area, value, rect), track_box) = cv.CamShift(backproject,self.track_window, crit)
        #    self.track_window = rect
        cv.ShowImage(self.cv_window_name, edges)
        cv.WaitKey(0)
        #self.image_subscribe = None


def main(args):
    pd = pirate_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down pd node.'
    cv.DestroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
