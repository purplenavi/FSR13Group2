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
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
        self.pirate_coordinates = None
        self.dead_pirates = []
        self.dead_pirate_coordinates = None
        self.camera_publisher = rospy.Publisher('/ptu_servo_angles', Vector3)
        self.pcpub = rospy.Publisher('vmp', PointCloud2)
        self.pirate_publisher = rospy.Publisher('/Pirates', Path)
        self.dead_pirate_publisher = rospy.Publisher('/Dead', Path)
        self.image_subscribe = None
        self.point_cloud2_subscribe = None
        rospy.sleep(1.0)
        # camera link transformation (maybe remeasure these?!)
        # self.camera_offset = [x y z tilt]
        self.camera_offset = [0.135, 0, -0.31, 115.0]
        self.tilt_camera()
        self.last_pcl = None
        self.activate_node()
        
    def activate_node(self):
        print self.tilt_camera()
        print 'Activating camera nodes'
        self.point_cloud2_subscribe = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pcl2_callback)
        rospy.sleep(1.0)
        #self.image_subscribe = rospy.Subscriber('/camera/rgb/image_mono', Image, self.img_cb)
        self.pirate_coordinates = None
        self.dead_pirate_coordinates = None

    def tilt_camera(self,x=0.0,y=115.0,z=90.0):
        self.camera_offset[3] = y
        self.camera_publisher.publish(Vector3(x, y, z))

    def deactivate_node(self):
        print 'Deactivating camera nodes'
        #self.image_subscribe.unregister()
        self.point_cloud2_subscribe.unregister()
        self.image_subscribe = None
        self.point_cloud2_subscribe = None
        
    def pcl2_callback(self, data):
        print 'PCL callback gotten'
        tmp = pointcloud2_to_array(data)
        tmp = self.downsample_pointcloud(tmp)
        tmp = self.remove_floor_and_segment(tmp)
        #fig = plt.figure()
        #ax = Axes3D(fig)
        #for y in range(len(tmp['x'])):
        #    ax.scatter(tmp['x'][y], tmp['y'][y], tmp['z'][y])
        #fig.add_axes(ax)
        #plt.show()
        cloud = array_to_pointcloud2(tmp)
        self.pcpub.publish(cloud)
        pirates = self.look_for_pirates(tmp)
        self.last_pcl = data
        
    def remove_floor_and_segment(self, pointcloud_array):
        tmp = pointcloud_array
        for y in range(len(tmp['x'])):
            for x in range(len(tmp['x'][0])):
                #If y value is close to zero, we can assume it's floor and set all values to zero :)
                #If z value is over 2, we remove it as we don't need to see that far
                if abs(tmp['y'][y][x]) < 0.005 or abs(tmp['z'][y][x]) > 2.0:
                    tmp['x'][y][x] = 999
                    tmp['y'][y][x] = 999
                    tmp['z'][y][x] = 999
        #TODO: Should probably remove empty rows
        return tmp
        
    def downsample_pointcloud(self, pointcloud_array):
        #downsampling cloud by 3x3 mask, and using center points as values to pass to downsampled cloud
        tmp = pointcloud_array
        result = tmp
        result_x = []
        result_y = []
        result_z = []
        j = 1
        for y in range(len(tmp['x'])):
            if y + j >= len(tmp['x']):
                break
            i = 1
            tmplistaX = []
            tmplistaY = []
            tmplistaZ = []
            for x in range(len(tmp['x'][0])):
                if x + i >= len(tmp['x'][0]):
                    break
                px = tmp['x'][y + j][x + i]
                py = tmp['y'][y + j][x + i]
                pz = tmp['z'][y + j][x + i]
                tmplistaX.append(px)
                tmplistaY.append(py)
                tmplistaZ.append(pz)
                i += 2
            j += 2
            result_x.append(tmplistaX)
            result_y.append(tmplistaY)
            result_z.append(tmplistaZ)
        result = np.resize(result, (len(result_x), len(result_x[0])))
        result['x'] = result_x
        result['y'] = result_y
        result['z'] = result_z
        return result
        
    def look_for_pirates(self, pointcloud_array, offset=0.02):
        tmp = pointcloud_array
        #Let's look something like 1X3 objects from the cloud, they should be pirates?
        #should look from up the ground or something
        pirates = []
        for y in range(len(tmp['x'])):
            if y > 7:
                for x in range(len(tmp['x'][0])):
                    if tmp['y'][y][x] > 0.04 and tmp['y'][y][x] < 0.06:
                        z1 = abs(tmp['z'][y][x])
                        z2 = abs(tmp['z'][y - 1][x])
                        z3 = abs(tmp['z'][y - 2][x])
                        z7 = abs(tmp['z'][y - 6][x])
                        if abs(z1 - z2) < offset and abs(z1 - z3) < offset and abs(z1 - z7) > offset:
                            accept = True
                            point = [tmp['z'][y][x], tmp['y'][y][x], tmp['x'][y][x]]
                            for pirate in pirates:
                                accept = self.distance(pirate, point)
                                if not accept:
                                    break
                            if accept:
                                print z7
                                pirates.append(point)
        for p in pirates:
            print p
        return pirates
        
    def distance(self, p1, p2):
        """
        Given two points, determine the distance
        """
        out = 0
        out += math.pow(p1[0] - p2[0], 2)
        out += math.pow(p1[2] - p2[2], 2)
        out = math.sqrt(out)
        if out < 0.2:
            return False
        else:
            return True
        
    def accept_pirate(self, x1, x2, z1, z2, offset=0.1):
        if abs(x1 - x2) < offset and abs(z1 - z2) < offset:
            return False
        return True
        
    def pcl2_parser(self):
        print 'Parsing pcl and image data'
        path1 = Path()
        path2 = Path()
        tmp = [[0.0,0.0]]
        while self.pirates:
            print 'WE has piratez'
            pirate = self.pirates.pop()
            cloud_data = pointcloud2_to_array(self.last_pcl)
            found = False
            for x in [0,1,-1,2,-2,3,-3,4,-4,5,-5]:
                for y in [0,1,-1,2,-2,3,-3,4,-4,5,-5]:
                    if pirate[1]+y < 0 or pirate[0]+x < 0 or len(cloud_data['x']) <= pirate[1]+y or len(cloud_data['x'][0]) <= pirate[0]+x:
                        continue
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
                # camera_link -> base_link conversion via simple trigonometria
                x = datapoint[2]*math.cos(self.camera_offset[3]-90)+self.camera_offset[0]
                y = -datapoint[0]+self.camera_offset[1]
                z = datapoint[1]*math.sin(self.camera_offset[3])+self.camera_offset[2]
                print 'Alive pirate found at ('+str(x)+', '+str(y)+', '+str(z)+')'
                if abs(z) > 0.20:
                    print 'Pirate is HIGH!'
                    continue
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
            cloud_data = pointcloud2_to_array(self.last_pcl)
            found = False
            for x in [0,1,-1,2,-2,3,-3,4,-4,5,-5]:
                for y in [0,1,-1,2,-2,3,-3,4,-4,5,-5]:
                    if pirate[1]+y < 0 or pirate[0]+x < 0 or len(cloud_data['x']) <= pirate[1]+y or len(cloud_data['x'][0]) <= pirate[0]+x:
                        continue
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
                # camera_link -> base_link conversion via simple trigonometria
                x = datapoint[2]*math.cos(self.camera_offset[3]-90.0)+self.camera_offset[0]
                y = -datapoint[0]+self.camera_offset[1]
                z = datapoint[1]*math.sin(self.camera_offset[3])+self.camera_offset[2]
                print 'Dead pirate found at ('+str(x)+', '+str(y)+', '+str(z)+')'
                if abs(z) > 0.10:
                    print 'Dead pirate HIGH! ('+str(abs(z))+')'
                    continue
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
        self.pirate_coordinates = path1
        self.dead_pirate_coordinates = path2
        self.pirate_publisher.publish(path1)
        self.dead_pirate_publisher(path2)
        #self.deactivate_node() # no need for restart every time 
        
    def img_cb(self, data):
        print 'image cb'
        try:
            #convert image to opencv format
            cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        cv.ShowImage(self.cv_window_name, cv_image)
        
    def image_callback(self, data):
        # Don't use data if it differs between image and point cloud too much
        if (abs(self.pcl_data.header.stamp.secs - data.header.stamp.secs)) > 5.0:
            print 'Data difference too big ('+str(abs(self.pcl_data.header.stamp.secs - data.header.stamp.secs))+') so skipping the callback!'
            return
        print 'Image callback '
        try:
            #convert image to opencv format
            cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        print 'Converted to openCV format'
        #Canny detecting
        cv.EqualizeHist(cv_image, cv_image)
        cv.Smooth(cv_image, cv_image, cv.CV_GAUSSIAN, 11, 11)
        yuv = cv.CreateImage(cv.GetSize(cv_image), 8, 3)
        gray = cv.CreateImage(cv.GetSize(cv_image), 8, 1)
        canny = cv.CreateImage(cv.GetSize(cv_image),8, 1)
        cv.Canny(cv_image,canny,50,250)
        bwimage = cv.CreateImage(cv.GetSize(canny), 8, 1)
        cv.Threshold(canny, bwimage, 128, 255, cv.CV_THRESH_BINARY)
        print 'Canny filtered, finding contours'
        a = np.asarray(canny[:,:])
        print 'numpy array created'
        cv.FloodFill(canny, (0, 0), 0)
        print 'canny image filled'
        storage = cv.CreateMemStorage()
        print 'memstorage created'
        contour_pointer = cv.FindContours(canny, storage, method=cv.CV_CHAIN_APPROX_TC89_KCOS,mode=cv.CV_RETR_EXTERNAL)
        contour_areas = []
        
        print 'Contours found'
    
        while contour_pointer is not None:
            contour = contour_pointer[ : ]
            centre_of_mass = self.find_centre_of_mass(contour)
            #self.draw_contour(cv_image, rotated_contour, cv.CV_RGB( 255 , 255 , 255 ), 5)
            x, y, w, h = cv.BoundingRect(contour)
            if h > w and h > 20 and h < 90:
                self.pirates.append((self.get_center_coordinates(x, y, h, w)))
                #cv.Rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), cv.CV_FILLED)
            elif w > h and w > 20 and w < 90 and (w-h) > 10:
                self.dead_pirates.append((self.get_center_coordinates(x, y, h, w)))
                #cv.Rectangle(cv_image, (x, y), (x+w, y+h), (0, 100, 0), cv.CV_FILLED)
            contour_pointer = contour_pointer.h_next()
            print 'Contour done'
        print self.pirates
        self.cv_image = cv_image
        #cv.ShowImage(self.cv_window_name, cv_image)
        print 'Image done'
        self.image_counter = 0
        if self.pirates and self.last_pcl is not None:
            self.pcl2_parser()
        cv.WaitKey(100) # try every 0.1 secs
        
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
