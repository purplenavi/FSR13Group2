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
        self.tilt_camera()
        self.last_pcl = None
        self.activate_node()
        
    def activate_node(self):
        print self.tilt_camera()
        print 'Activating camera nodes'
        self.point_cloud2_subscribe = rospy.Subscriber('/points_transformed', PointCloud2, self.pcl2_callback)
        rospy.sleep(1.0)
        #self.image_subscribe = rospy.Subscriber('/camera/rgb/image_mono', Image, self.img_cb)
        self.pirate_coordinates = None
        self.dead_pirate_coordinates = None

    def tilt_camera(self,x=0.0,y=115.0,z=90.0):
        self.camera_publisher.publish(Vector3(x, y, z))

    def deactivate_node(self):
        print 'Deactivating camera nodes'
        #self.image_subscribe.unregister()
        self.point_cloud2_subscribe.unregister()
        self.image_subscribe = None
        self.point_cloud2_subscribe = None
        
    def pcl2_callback(self, data):
        print data.header.frame_id
        print 'PCL callback gotten'
        tmp = pointcloud2_to_array(data)
        tmp = self.downsample_pointcloud(tmp)
        cloud = array_to_pointcloud2(tmp)
        cloud.header.frame_id = '/base_link'
        self.pcpub.publish(cloud)
        pirates = self.look_for_pirates(tmp)
        dead = self.look_for_dead_pirates(tmp)
        self.publish_pirates(pirates)
        self.publish_dead(dead)
        self.last_pcl = data
        
    def remove_floor_and_segment(self, pointcloud_array):
        #TODO: FIX THE METHOD!
        tmp = pointcloud_array
        for y in range(len(tmp['x'])):
            for x in range(len(tmp['x'][0])):
                #If y value is close to zero, we can assume it's floor and set all values to zero :)
                #If z value is over 2, we remove it as we don't need to see that far
                if abs(tmp['z'][y][x]) < 0.005 or abs(tmp['x'][y][x]) > 2.0:
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

    def transform_pointcloud(self, cloud):
        listener = tf.TransformListener()
        print dir(cloud)
        return [listener.transformPoint('/map', PointStamped(cloud.header, p).point) for p in cloud.points]
        
    def publish_pirates(self, pirates):
        self.pirate_publisher.publish(pirates)
        
    def publish_dead(self, dead):
        self.dead_pirate_publisher.publish(dead)
        
    def look_for_dead_pirates(self, pointcloud_array, offset=0.02):
        tmp = pointcloud_array
        #Look for something like 1x1 objects from the cloud like 1-2 cm above the floor (floor currently at ~-0.69)
        dead = []
        for y in range(len(tmp['x'])):
            for x in range(len(tmp['x'][0])):
                if tmp['z'][y][x] > -0.68 and tmp['z'][y][x] < -0.665 and tmp['x'][y][x] < 1.8:
                    if x > 2:
                        y1 = abs(tmp['y'][y][x])
                        y2 = abs(tmp['y'][y][x + 2])
                        if abs(y1 - y2) > offset:
                            accept = True
                            point = [tmp['x'][y][x], tmp['y'][y][x], tmp['z'][y][x]]
                            for d in dead:
                                accept = self.distance(d, point)
                                if not accept:
                                    break
                            if accept:
                                dead.append(point)
        path1 = Path()
        camerapoint = PoseStamped()
        for d in dead:
            quaternion = quaternion_from_euler(0, 0, math.atan2(d[1]-0, d[0]-0))
            camerapoint.header.frame_id = 'map'
            camerapoint.header.stamp = rospy.Time.now()
            camerapoint.pose.position.x = d[0]
            camerapoint.pose.position.y = d[1]
            camerapoint.pose.orientation.w = quaternion[3]
            camerapoint.pose.orientation.z = quaternion[2]
            path1.poses.append(camerapoint)
        return path1

    def look_for_pirates(self, pointcloud_array, offset=0.02):
        tmp = pointcloud_array
        #Let's look something like 1X3 objects from the cloud, they should be pirates?
        #should look from up the ground or something
        pirates = []
        for y in range(len(tmp['x'])):
            if y > 7:
                for x in range(len(tmp['x'][0])):
                    if tmp['z'][y][x] > -0.68 and tmp['z'][y][x] < -0.65 and tmp['x'][y][x] < 1.8:
                        x1 = abs(tmp['x'][y][x])
                        x2 = abs(tmp['x'][y - 1][x])
                        x3 = abs(tmp['x'][y - 2][x])
                        x7 = abs(tmp['x'][y - 6][x])
                        if abs(x1 - x2) < offset and abs(x1 - x3) < offset and abs(x1 - x7) > offset:
                            accept = True
                            point = [tmp['x'][y][x], tmp['y'][y][x], tmp['z'][y][x]]
                            for pirate in pirates:
                                accept = self.distance(pirate, point)
                                if not accept:
                                    break
                            if accept:
                                pirates.append(point)
        path1 = Path()
        camerapoint = PoseStamped()
        for p in pirates:
            quaternion = quaternion_from_euler(0, 0, math.atan2(p[1]-0, p[0]-0))
            camerapoint.header.frame_id = 'map'
            camerapoint.header.stamp = rospy.Time.now()
            camerapoint.pose.position.x = p[0]
            camerapoint.pose.position.y = p[1]
            camerapoint.pose.orientation.w = quaternion[3]
            camerapoint.pose.orientation.z = quaternion[2]
            path1.poses.append(camerapoint)
        return path1
        
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
