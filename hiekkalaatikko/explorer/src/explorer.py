#!/usr/bin/env python
import roslib
roslib.load_manifest('explorer')
import rospy
from nav_msgs.msg import MapMetaData,OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import math

"""
MUST check is it 0 or 100 the value given from laser when contains something.
"""

# Explorer map
# 0 = unchecked
# 1 = checked and clear
# 2 = wall
class Explorer:

    # Actual initialization happens at first laser callback where the current pose, map size and resolution are defined.
    def __init__(self):
        self.angle = 55 # angle in where camera can detect pirates ni degrees
        self.min_distance = 0.2 # minimum distance where camera can detect pirates in meters
        self.max_distance = 1.5 # maximum distance where camera can detect pirates in meters
        self.resolution = None # resolution in m/cell
        self.map = None
        self.pose = None
        # Subscribers to deal with incoming data
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.laser_callback)
        self.camera_subscriber = rospy.Subscriber('/camera/rgb/image_mono', Image, self.detector_callback)
        # Publish goals as PoseStamped, subscriber at task planner ?
        self.goal_pub = rospy.Publisher('explore_point', PoseStamped)
        self.get_goal_sub = rospy.Subscriber('/explore_next_point', String, self.get_next_point)

    # Method to update map with laser callback data
    def laser_callback(self,msg):
        self.pose = msg.info.origin
        self.resolution = msg.info.resolution
        if self.map is None:
            self.map = np.zeros((msg.info.height, msg.info.width))
            print 'Explorer initialized'
        reshaped_data = np.array(msg.data, dtype=np.uint8, copy=False, order='C')
        reshaped_data = reshaped_data.reshape((msg.info.height, msg.info.width))
        # Convert reshaped values gotten from laser to binary
        laser_data = np.array(reshaped_data > 50, dtype=int)
        # Points to be cleared to make sure points contain value 2
        clearpoints = laser_data & np.array(self.map > 0, dtype=int)
        # Delete existing values from map in poins new laser data's going
        self.map -= np.multiply(clearpoints,self.map)
        # Add new laser data points to map as 2
        self.map += 2 * laser_data

    # Method for detector callbacks (pcl)
    def detector_callback(self, data):
        if self.map is None or self.resolution is None or self.pose is None:
            print 'Cannot use camera data, explorer not initialized yet'
            return
        # Robot coordinates on map
        robotx = self.pose.position.x #150
        roboty = self.pose.position.y #150

        (roll, pitch, robotangle) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]) #40

        beamangle = robotangle - (self.angle / 2)
        step = 0.5

        # Mark area checked
        for a in xrange(self.angle / step):
            targetangle = math.radians(beamangle + a * step)

            # Mark both ends of line
            x0 = robotx + self.min_distance * self.resolution * math.cos(targetangle)
            y0 = roboty + self.min_distance * self.resolution * math.sin(targetangle)

            x1 = robotx + self.max_distance * self.resolution * math.cos(targetangle)
            y1 = roboty + self.max_distance * self.resolution * math.sin(targetangle)

            # Line drawing algorithm
            dx = math.abs(x1 - x0)
            dy = math.abs(y1 - y0)
            sx = -1
            sy = -1
            if x0 < x1:
                sx = 1

            if y0 < y1:
                sy = 1

            err = dx - dy

            i = 0
            while i < 1000:
                i = i + 1

                if not self.plot(x0, y0):
                    break

                if x0 == x1 and y0 == y1:
                    break
                      
                e2 = 2 * err
                if e2 > -dy: 
                    err = err - dy
                    x0 = x0 + sx

                if x0 = x1 and y0 = y1:
                    self.plot(x0,y0)
                    break

                if e2 < dx:
                    err = err + dx
                    y0 = y0 + sy 

    def plot(self, x, y):
        if self.map[y][x] == 2:
            # Collided a wall, end drawing
            return False

        # Draw
        self.map[y][x] = 1
        return True


    def get_next_point(self, msg):
        if self.map is None or self.pose is None or self.resolution is None:
            print 'Cannot get next point, Explorer not initialized yet'
            return
        time_start = rospy.get_time()
        # There's no reason for me to do anything with the msg, wadap...
        print 'Wow! I just got a message from task planner: '+msg
        unexplored = np.where(self.map == 0) # Giving the indexes of map containing the zeros
        if len(unexplored) == 0:
            print 'Whole map is already checked out so the exploring is done!';
            return None
        weightmap = np.zeros((len(self.map),len(self.map[0])))
        # Check smallest distance from walls, obstacles or prechecked point for each zero point
        for i in range(len(unexplored[0])):
            x = unexplored[0][i]
            y = unexplored[1][i]
            it = 1
            mindist = len(weightmap)
            while y+it < len(weightmap):
                if self.map[y+it][x] > 0:
                    break
                it += 1
            if mindist > it:
                mindist = it
            it = 1
            while y > 0 and it < mindist and y-it >= 0:
                if self.map[y-it][x] > 0:
                    break
                it += 1
            if mindist > it:
                mindist = it
            it = 1
            while it < mindist and x+it < len(weightmap[0]):
                if self.map[y][x+it] > 0:
                    break
                it += 1
            if mindist > it:
                mindist = it
            it = 1
            while it < mindist and x-it >= 0:
                if self.map[y][x-it] > 0:
                    break
                it += 1
            if mindist < it:
                mindist = it
            weightmap[y][x] = mindist
        # Get maximum valued point that ain't behind wall (no 2 in map in direct path)
        new_points = np.where(weightmap==weightmap.max()):
        # x and y are cell indexes here
        x = None
        y = None
        while weightmap.max() > 0 and len(new_points) > 0:
            for i in range(len(new_points[0]))
                distance = math.sqrt(math.pow(new_points[1][i]-self.pose.position.x,2)+math.pow(new_points[0][i]-self.pose.position.y,2))
                angle = math.atan2(new_points[0][i]-self.pose.position.y,new_points[1][i]-self.pose.position.x)
                x = new_points[0][i]
                y = new_points[1][i]
                for j in range(math.ceil(distance)) # loop through every cell in the way
                    point = (self.pose.position.x + j*math.cos(angle), self.pose.position.y + j*math.sin(angle))
                    # Check points rounded around this point ain't an obstacle
                    if self.map[math.floor(point[1])][math.floor(point[2])] == 2:
                        x = None
                        y = None
                        break
                    if self.map[math.floor(point[1])][math.ceil(point[2])] == 2:
                        x = None
                        y = None
                        break
                    if self.map[math.ceil(point[1])][math.floor(point[2])] == 2:
                        x = None
                        y = None
                        break
                    if self.map[math.ceil(point[1])][math.ceil(point[2])] == 2:
                        x = None
                        y = None
                        break
                if x is not None and y is not None:
                    break
                else:
                    weightmap[0][i] = 0
                    weightmap[1][i] = 0
                    new_points = np.where(weightmap==weightmap.max())
            if x is not None and y is not None:
                # There's no walls in the way, no need to check more points from weightmap
                break
        if x is None or y is None:
            print 'Iz gone bad! No new point found but behind walls even though it should exist'
            # Needs some recovery behavior, 'cause it might end up in here eventually
            return None
        else:
            # Convert cell indexes to coordinates
            x *= self.resolution
            y *= self.resolution
            # Create MoveBaseGoal and return it
            goal = PoseStamped()
            quaternion = quaternion_from_euler(0, 0, math.atan2(y-self.pose.position.y, x-self.pose.position.x))
            goal.header.frame_id = 'map'
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y 
            goal.pose.orientation.w = self.pose.orientation.w
            goal.pose.orientation.z = self.pose.orientation.z
            print 'Next unexplored goal publish at ('+str(x)+', '+str(y)+'), calculation took '+str(rospy.get_time()-time_start)+' seconds..'
            self.goal_pub.publish(goal)
