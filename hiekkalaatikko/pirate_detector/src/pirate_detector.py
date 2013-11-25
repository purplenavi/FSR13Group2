#!/usr/bin/env python
import roslib
roslib.load_manifest('pirate_detector')
import rospy
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import math
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Vector3, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
import os
import sys
import cv
from pointcloud import *

class pirate_detector:
    def __init__(self, parent=None):
        self.parent = parent
        #Give the opencv display window a name
        self.cv_window_name = 'OpenCVImage'
        #create the window and make it re-sizeable (second parameter 0)
        cv.NamedWindow(self.cv_window_name)
        #create the cv_bridge object
        self.bridge = CvBridge()
        #subscribe to image data
        
        self.pirates = []
        self.pirate_coordinates = []
        self.dead_pirates = []
        self.dead_pirate_coordinates = []
        self.camera_publisher = rospy.Publisher('/ptu_servo_angles', Vector3, latch=False)
        self.pirate_publisher = rospy.Publisher('/Pirates', Path)
        self.dead_pirate_publisher = rospy.Publisher('/Dead', Path)
        self.image_subscribe = rospy.Subscriber('/camera/rgb/image_mono', Image, self.image_callback)
        self.point_cloud2_subscribe = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pcl2_callback)
        rospy.sleep(1.0)
        self.camera_publisher.publish(Vector3(x=0.0, y=115.0, z=90.0))
        self.tf = tf.TransformListener()
        
    def pcl2_callback(self, data):
        path1 = Path()
        path2 = Path()
        tmp = [[0.0,0.0]]
        while self.pirates:
            print 'WE has piratez'
            pirate = self.pirates.pop()
            cloud_data = pointcloud2_to_array(data)
            found = False
            for x in [0,1,-1,2,-2,3,-3,4,-4,5,-5]:
                for y in [0,1,-1,2,-2,3,-3,4,-4,5,-5]:
                    datapoint = []
                    datapoint.append(cloud_data['x'][pirate[1]+y][pirate[0]+x])
                    datapoint.append(cloud_data['y'][pirate[1]+y][pirate[0]+x])
                    datapoint.append(cloud_data['z'][pirate[1]+y][pirate[0]+x])
                    if not math.isnan(datapoint[0]) and not math.isnan(datapoint[1]) and not math.isnan(datapoint[2]):
                        found = True
                        break
                if found:
                    break
            if found:
                print datapoint
                #Create a P2P message to reach the figure
                camerapoint = PoseStamped()
                #z is depth in cloud data
                x = datapoint[2]
                y = -datapoint[0]
                z = datapoint[1]
                insert_pirate = True
                for i in tmp:
                    if abs(x-i[0]) < 0.1 or abs(y-i[1]) < 0.1:
                        insert_pirate = False
                        break
                if insert_pirate:
                    quaternion = quaternion_from_euler(0, 0, math.atan2(y-0, x-0))
                    camerapoint.header.frame_id = 'map'
                    camerapoint.header.stamp = rospy.Time.now()
                    camerapoint.pose.position.x = x
                    camerapoint.pose.position.y = y
                    camerapoint.pose.orientation.w = quaternion[3]
                    camerapoint.pose.orientation.z = quaternion[2]
                    path1.poses.append(camerapoint)
                    print 'Found pirate coordinates'
                    tmp.append([x, y])
                    cv.Circle(self.cv_image, (int(pirate[0]), int(pirate[1])), 20, (0, 100, 0), cv.CV_FILLED)
        #Now check dead pirate coordinates
        tmp = [[0.0,0.0]]
        while self.dead_pirates:
            pirate = self.dead_pirates.pop()
            cloud_data = pointcloud2_to_array(data)
            found = False
            for x in [0,1,-1,2,-2,3,-3,4,-4,5,-5]:
                for y in [0,1,-1,2,-2,3,-3,4,-4,5,-5]:
                    datapoint = []
                    datapoint.append(cloud_data['x'][pirate[1]+y][pirate[0]+x])
                    datapoint.append(cloud_data['y'][pirate[1]+y][pirate[0]+x])
                    datapoint.append(cloud_data['z'][pirate[1]+y][pirate[0]+x])
                    if not math.isnan(datapoint[0]) and not math.isnan(datapoint[1]) and not math.isnan(datapoint[2]):
                        found = True
                        break
                if found:
                    break
            if found:
                print datapoint
                #Create a P2P message to reach the figure
                camerapoint = PoseStamped()
                #z is depth in cloud data
                x = datapoint[2]
                y = -datapoint[0]
                z = datapoint[1]
                insert_pirate = True
                for i in tmp:
                    if abs(x-i[0]) < 0.1 or abs(y-i[1]) < 0.1:
                        insert_pirate = False
                        break
                if insert_pirate:
                    quaternion = quaternion_from_euler(0, 0, math.atan2(y-0, x-0))
                    camerapoint.header.frame_id = 'map'
                    camerapoint.header.stamp = rospy.Time.now()
                    camerapoint.pose.position.x = x
                    camerapoint.pose.position.y = y
                    camerapoint.pose.orientation.w = quaternion[3]
                    camerapoint.pose.orientation.z = quaternion[2]
                    path2.poses.append(camerapoint)
                    print 'Found dead pirate coordinates'
                    tmp.append([x, y])
                    cv.Circle(self.cv_image, (int(pirate[0]), int(pirate[1])), 15, (255, 255, 255), cv.CV_FILLED)
        print 'Publishing coordinates'
        cv.ShowImage(self.cv_window_name, self.cv_image)
        self.pirate_publisher.publish(path1)
        self.dead_pirate_publisher.publish(path2)
        
    def image_callback(self, data):
        try:
            #convert image to opencv format
            cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        #Canny detecting
        cv.EqualizeHist(cv_image, cv_image)
        cv.Smooth(cv_image, cv_image, cv.CV_GAUSSIAN, 11, 11)
        yuv = cv.CreateImage(cv.GetSize(cv_image), 8, 3)
        gray = cv.CreateImage(cv.GetSize(cv_image), 8, 1)
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
            contour = contour_pointer[ : ]
            centre_of_mass = self.find_centre_of_mass(contour)
            rotated_contour = self.rotate_contour(contour, centre_of_mass, 0)
            lst = []
    
            for i in rotated_contour:
                lst.append(list(i))
            rotated_contour = []
    
            for i in lst:
                i[0] = int(i[0])
                i[1] = int(i[1])
                rotated_contour.append(tuple(i))
            #self.draw_contour(cv_image, rotated_contour, cv.CV_RGB( 255 , 255 , 255 ), 5)
            x, y, w, h = cv.BoundingRect(contour)
            if h > w and h > 20 and h < 90:
                self.pirates.append((self.get_center_coordinates(x, y, h, w)))
                #cv.Rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), cv.CV_FILLED)
            elif w > h and w > 20 and w < 90 and (w-h) > 10:
                self.dead_pirates.append((self.get_center_coordinates(x, y, h, w)))
                #cv.Rectangle(cv_image, (x, y), (x+w, y+h), (0, 100, 0), cv.CV_FILLED)
            contour_pointer = contour_pointer.h_next()
        print self.pirates
        self.cv_image = cv_image
        #cv.ShowImage(self.cv_window_name, cv_image)
        cv.WaitKey(3000)
        
    #Code samples fetched from http://www.pirobot.org/blog/0016/
    # Draw contour from list of tuples.
    def draw_contour( self, im , contour , color , thickness = 1 , linetype = 8 ,
                    shift = 0 ) :
        if thickness == -1 :
            cv.FillPoly( im , [contour] , color , linetype , shift )
        else :
            cv.PolyLine( im , [contour] , True , color , thickness , linetype , shift )

    # Rotate contour around centre point using numpy.
    def rotate_contour( self, contour , centre_point , theta ) :
        rotation = np.array( [ [ np.cos( theta ) , -np.sin( theta ) ] , [ np.sin( theta ) ,  np.cos( theta ) ] ] )
        centre = np.vstack( [ centre_point ] * len( contour ) )
        contour = np.vstack( contour ) - centre
        contour = np.dot( contour , rotation ) + centre
        return [ tuple ( each_row ) for each_row in contour ]

    # Find centre of mass by drawing contour in closed form and using moments.
    def find_centre_of_mass( self, contour ) :
        bottom_right = np.max( contour , axis = 0 )
        blank = cv.CreateImage( tuple ( bottom_right ) , 8 , 1 )
        cv.Set( blank , 0 )
        self.draw_contour( blank , contour , 1, -1 )
        matBlank = cv.GetMat(blank)
        moments = cv.Moments( matBlank , 1 )  
        sM00 = float ( cv.GetSpatialMoment( moments , 0 , 0 ) )
        sM01 = float ( cv.GetSpatialMoment( moments , 0 , 1 ) )
        sM10 = float ( cv.GetSpatialMoment( moments , 1 , 0 ) )
        if sM00 == 0:
            sM00 = 1
        return ( sM10 / sM00 , sM01 / sM00 )
    
    def get_center_coordinates(self, x, y, w, h):
        center_x = float(x + w/2.0)
        center_y = float(y + h/2.0)
        return (center_x, center_y)

    def pointcloud2_to_array(self, cloud_msg):
        dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]
        cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
        return np.reshape(cloud_arr, (cloud_msg.width, cloud_msg.width)) 
    
    def get_xyz_points(self, cloud_array, remove_nans=True):
        if remove_nans:
            mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
            cloud_array = cloud_array[mask]
        points = np.zeros(list(cloud_array.shape) + [3], dtype=np.float)
        points[...,0] = cloud_array['x']
        points[...,1] = cloud_array['y']
        points[...,2] = cloud_array['z']
        return points

    def pointcloud2_to_xyz_array(self, cloud_msg, remove_nans=True):
        return self.get_xyz_points(self.pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)
        
def main(args):
    rospy.init_node('pirate_detector')
    rospy.sleep(1.0)
    pd = pirate_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down pd node.'
    cv.DestroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
