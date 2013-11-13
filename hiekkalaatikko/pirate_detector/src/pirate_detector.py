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
import tf
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3, Twist, PointStamped

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
  
def get_center_coordinates(x, y, w, h):
    center_x = float(x + w/2.0)
    center_y = float(y + h/2.0)
    return (center_x, center_y)

THETA = np.pi / 3.0
COLOR = cv.CV_RGB( 255 , 255 , 255 )

def pointcloud2_to_array(cloud_msg):
    dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
    return np.reshape(cloud_arr, (cloud_msg.width, cloud_msg.width)) 
 
def get_xyz_points(cloud_array, remove_nans=True):
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]
    points = np.zeros(list(cloud_array.shape) + [3], dtype=np.float)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']
    return points

def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    return get_xyz_points(pointcloud2_to_array(cloud_msg), remove_nans=remove_nans) 

class pirate_detector:
   
    def __init__(self):

        rospy.init_node('pirate_detector')
        print "Init"
        self.pirates = []
        #Give the opencv display window a name
        self.cv_window_name = 'OpenCVImage'
        self.rgb_window_name = 'RGBimage'
        self.depth_window = 'Depth'
        self.UPDATE_PIRATE_DATA = True
        #create the window and make it re-sizeable (second parameter 0)
        cv.NamedWindow(self.cv_window_name)
        #cv.NamedWindow(self.rgb_window_name)
        cv.NamedWindow(self.depth_window)
        #create the cv_bridge object
        self.bridge = CvBridge()
        #subscribe to image data
        self.publisher = rospy.Publisher('RosAria/cmd_vel', Twist)
        self.image_subscribe = rospy.Subscriber('/camera/rgb/image_mono', Image, self.image_callback)
        #self.rgb_subscribe = rospy.Subscriber('/camera/rgb/image_color', Image, self.rgb_callback)
        #self.point_cloud_subscribe = rospy.Subscriber('/camera/depth/image', Image, self.point_cloud_callback)
        self.point_cloud2_subscribe = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pcl2_callback)
        self.camera_publisher = rospy.Publisher('/ptu_servo_angles', Vector3, latch=False)
        rospy.sleep(1.0)
        self.camera_publisher.publish(Vector3(x=0.0, y=110.0, z=90.0))
        self.img = None
        self.listener = tf.TransformListener()
        
        
    def pcl2_callback(self, data):
        print 'PCL2 Callback'
        #print data.height
        #print data.fields
        xyzArr = pointcloud2_to_xyz_array(data, False)
        pirate = self.pirates.pop()
        
        camerapoint = PointStamped()
        camerapoint.header.frame_id = 'camera_link'
        camerapoint.header.stamp = rospy.Time.now()
        camerapoint.point.x = xyzArr[pirate[0]][pirate[1]][0]+0.135
        camerapoint.point.y = xyzArr[pirate[0]][pirate[1]][1]
        camerapoint.point.z = xyzArr[pirate[0]][pirate[1]][2]-0.31
        
        #worldpoint = PointStamped()
        #worldpoint = self.listener.transformPoint('map', camerapoint)
        
        print xyzArr.shape
        print xyzArr[pirate[0]][pirate[1]][0]+' -> '+camerapoint.point.x
        print xyzArr[pirate[0]][pirate[1]][1]+' -> '+worldpoint.point.y
        print xyzArr[pirate[0]][pirate[1]][2]+' -> '+worldpoint.point.z
        
    def point_cloud_callback(self, data):
        print 'IHQ'
        if not self.pirates:
            self.UPDATE_PIRATE_DATA = True
        else:
            exit = False
            while not exit:
                pirate = self.pirates.pop()
                try:
                    #convert image to opencv format
                    cv_image = self.bridge.imgmsg_to_cv(data)
                except CvBridgeError, e:
                    print e
                if pirate[1] > 240 and pirate[1] < 400:
                    print 'PIRATE AT:'
                    print pirate
                    print 'RANGE:'
                    pirate_locationz = cv_image[pirate[1],pirate[0]]
                    cv.Circle(self.img, (int(pirate[0]), int(pirate[1])), 3, COLOR, thickness=2)
                    cv.ShowImage(self.depth_window, cv_image)
                    break
            if pirate_locationz:
                twist = Twist()
                twist.linear.x = 0.1
                for i in range(int(10*(pirate_locationz))):
                    self.publisher.publish(twist)
                    rospy.sleep(0.1)
                twist = Twist()
                self.publisher.publish(twist)
                cv.ShowImage(self.cv_window_name, self.img)
        
    def image_callback(self, data):
        print 'Trololoo'
        if self.UPDATE_PIRATE_DATA:
            print "Callback"
            try:
                #convert image to opencv format
                cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
            except CvBridgeError, e:
                print e
            print cv_image
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
            bwimage = cv.CreateImage(cv.GetSize(canny), 8, 1)
            cv.Threshold(canny, bwimage, 128, 255, cv.CV_THRESH_BINARY)
            
            a = np.asarray(canny[:,:])
            cv.FloodFill(canny, (0, 0), 0)
            storage = cv.CreateMemStorage()
            contour_pointer = cv.FindContours(canny, storage, method=cv.CV_CHAIN_APPROX_TC89_KCOS,mode=cv.CV_RETR_EXTERNAL)
            contour_areas = []
            while contour_pointer is not None:
                print 'LOL'
                contour = contour_pointer[ : ]
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
                draw_contour(cv_image, rotated_contour, COLOR, 5)
                x, y, w, h = cv.BoundingRect(contour)
                self.pirates.append((get_center_coordinates(x, y, h, w)))
                cv.Rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), cv.CV_FILLED)
                contour_pointer = contour_pointer.h_next()
            self.img = cv_image
            cv.ShowImage(self.cv_window_name, cv_image)
            self.UPDATE_PIRATE_DATA = False
            cv.WaitKey(3000)

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
