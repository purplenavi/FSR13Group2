#!/usr/bin/env python
import roslib
roslib.load_manifest('explorer')
import rospy
from nav_msgs.msg import MapMetaData,OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import numpy as np
import math
import random

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
        self.viewpoints = []
        self.view_xcount = 5
        self.view_ycount = 2
        self.weightLimit = 1500
        self.directions = [(0, 0), (0, 1), (1, 0), (0, -1), (-1, 0)]
        self.poselist = []
        self.poseindex = 0
        self.firstcall = True
        self.pos_x = None
        self.pos_y = None
        # Subscribers to deal with incoming data
        #self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.laser_callback)
        #self.camera_subscriber = rospy.Subscriber('/camera/rgb/image_mono', Image, self.detector_callback)
        # Publish goals as PoseStamped, subscriber at task planner ?
        #self.goal_pub = rospy.Publisher('explore_point', PoseStamped)
        #self.get_goal_sub = rospy.Subscriber('/explore_next_point', String, self.next_pose_callback)

    # Method to update map with laser callback data
    def laser_callback(self,msg):
        self.pose = msg.info.origin
        self.pos_x = self.pose.position.x
        self.pos_y = self.pose.position.y
        (roll, pitch, robotangle) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        self.angle = robotangle
        
        self.resolution = msg.info.resolution
        if self.map is None:
            self.map = np.zeros((msg.info.height, msg.info.width))
            self.width = msg.info.width
            self.height = msg.info.height
            print 'Explorer initialized'
        # Convert values gotten from laser to int
        reshaped_data = np.array(msg.data, dtype=int, copy=False, order='C')
        # Convert to binary 
        reshaped_data = np.array(reshaped_data > 50, dtype=int)
        # Reshape the match map size
        reshaped_data = reshaped_data.reshape((msg.info.height, msg.info.width))
        # Points to be cleared to make sure points contain value 7
        clearpoints = reshaped_data & np.array(self.map > 0, dtype=int)
        # Delete existing values from map in poins new laser data's going
        self.map -= np.multiply(clearpoints,self.map)
        # Add new laser data points to map as 7
        self.map += 7 * reshaped_data

    # Method for detector callbacks (pcl)
    def detector_callback(self, data):
        if self.map is None or self.resolution is None or self.pose is None:
            print 'Cannot use camera data, explorer not initialized yet'
            return
        # Robot coordinates on map
        robotx = int(len(self.map[0])/2)+self.pose.position.x #150
        roboty = int(len(self.map)/2)+self.pose.position.y #150

        (roll, pitch, robotangle) = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]) #40

        beamangle = robotangle - (self.angle / 2)
        step = 0.5

        # Search end points
        self.ends = []
        
        # Mark area checked
        for a in xrange(int(self.angle / step)):
            targetangle = math.radians(beamangle + a * step)

            # Mark both ends of line
            x0 = robotx
            y0 = roboty
            
            xm = robotx + self.min_distance * self.resolution * math.cos(targetangle)
            ym = roboty + self.min_distance * self.resolution * math.sin(targetangle)

            x1 = robotx + self.max_distance * self.resolution * math.cos(targetangle)
            y1 = roboty + self.max_distance * self.resolution * math.sin(targetangle)

            # Line drawing algorithm
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
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
                    self.plot(x0,y0,True)
                    break
                  
                e2 = 2 * err
                if e2 > -dy: 
                    err = err - dy
                    x0 = x0 + sx

                if x0 == x1 and y0 == y1:
                    self.plot(x0,y0,True)
                    break

                if e2 < dx:
                    err = err + dx
                    y0 = y0 + sy 
                    
        # Mark interest points
        anglechange = self.angle / (self.view_xcount + 1)
        range = 0.8 * self.max_distance
    
        for i in xrange(1, self.view_xcount + 1):
            targetangle = math.radians(beamangle + (anglechange * i))
            xp = range * math.cos(targetangle)
            yp = range * math.sin(targetangle)
            
            xp = robotx + math.floor(xp / self.resolution)
            yp = roboty + math.floor(yp / self.resolution)
            
            print "Interest point in %d, %d, angle %.1f" % (xp, i, targetangle)
            
            if not self.markViewPoint(xp, yp):
                print "failed"
                
        # Final point in close
        targetangle = math.radians(robotangle)
        range = self.min_distance + 0.2 * (self.max_distance - self.min_distance)

        xp = range * math.cos(targetangle)
        yp = range * math.sin(targetangle)
            
        xp = robotx + math.floor(xp / self.resolution)
        yp = roboty + math.floor(yp / self.resolution)
        self.markViewPoint(xp, yp)
        
        # Draw map
        self.drawTargetMap(self.map)


    def markViewPoint(self, x, y):
        for i in xrange(0, 4):
            point = self.directions[i]
            xp = x + point[0]
            yp = y + point[1]
            if self.vp(xp, yp):
                return True
        
        return False

    def vp(self, x, y):
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False

        if self.map[y][x] > 0 and self.map[y][x] < 4:
            self.map[y][x] = 2
            self.viewpoints.append((x, y, 0))
            return True
        return False
    
    def reorder_interestpoints(self):
        for i in xrange(len(self.viewpoints)):
            # Count distance to current position
            x = self.viewpoints[i][0]
            y = self.viewpoints[i][1]
            
            dist = math.sqrt(math.pow(self.pos_x - x,2) + math.pow(self.pos_y - y,2))
            self.viewpoints[i] = (x, y, dist)
            
        self.viewpoints = sorted(self.viewpoints, key=lambda point: point[2])
    


    def plot(self, x, y, end=False):
        
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False
    
        if self.map[y][x] == 7:
            # Collided a wall, end drawing
            return False

        # Draw
        if end:
            # Mark scan end
            self.map[y][x] = 3
        
            # Append end coordinates to array
            coords = (x, y)
            if coords not in self.ends:
                self.ends.append(coords)
        
            return True
    
        if self.map[y][x] == 0:
            self.map[y][x] = 1
        return True
        

    def calculate_weightmap(self):
        if self.map is None or self.pose is None or self.resolution is None:
            print 'Cannot get next point, Explorer not initialized yet'
            return

        unexplored = np.where(self.map == 0) # Giving the indexes of map containing the zeros
        if len(unexplored) == 0:
            print 'Whole map is already checked out so the exploring is done!';
            return None
        
        self.weightmap = np.zeros((len(self.map),len(self.map[0])))
        
        searchdistance = 8
        
        # Check smallest distance from walls, obstacles or prechecked point for each zero point
        print 'Starting weightmap calculation, unexplored: ' + str(len(unexplored[0]))
        for i in range(len(unexplored[0])):
            x = unexplored[1][i]
            y = unexplored[0][i]
            it = 1
            mindist = len(self.weightmap)
            while y+it < len(self.weightmap) and it < searchdistance:
                if self.map[y+it][x] > 0:
                    break
                it += 1
            if mindist > it:
                mindist = it
            it = 1
            while it < mindist and y-it >= 0 and it < searchdistance:
                if self.map[y-it][x] > 0:
                    break
                it += 1
            if mindist > it:
                mindist = it
            it = 1
            while it < mindist and x+it < len(self.weightmap[0]) and it < searchdistance:
                if self.map[y][x+it] > 0:
                    break
                it += 1
            if mindist > it:
                mindist = it
            it = 1
            while it < mindist and x-it >= 0 and it < searchdistance:
                if self.map[y][x-it] > 0:
                    break
                it += 1
            if mindist > it:
                mindist = it
            self.weightmap[y][x] = mindist
            if self.weightmap[y][x] >= searchdistance:
                continue
        print 'Weightmap calculated'
 
     
    def direction_from_point(self, x, y):
        radius = self.max_distance / self.resolution
    
        # Clamp area
        xMin = int(max(0, x - radius))
        yMin = int(max(0, y - radius))
        xMax = int(min(self.width - 1, x + radius))
        yMax = int(min(self.height - 1, y + radius))    	
    
        tempMap = np.zeros((yMax - yMin, xMax - xMin))
    
        step = 0.5
    
        for a in xrange(int(360 / step)):
            targetangle = math.radians(a * step)
            
            # Mark both ends of line
            x0 = x
            y0 = y

            x1 = int(x + self.max_distance / self.resolution * math.cos(targetangle))
            y1 = int(y + self.max_distance / self.resolution * math.sin(targetangle))
            
            # Line drawing algorithm
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
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

                if not self.isClear(x0, y0):
                    break
                
                #self.plotMap(x0 - xMin,y0 - yMin, self.weightmap[y0][x0], tempMap)

                if x0 == x1 and y0 == y1:
                    #self.plotMap(x0 - xMin,y0 - yMin, self.weightmap[y0][x0], tempMap)
                    break
                      
                e2 = 2 * err
                if e2 > -dy: 
                    err = err - dy
                    x0 = x0 + sx

                if x0 == x1 and y0 == y1:
                    #self.plotMap(x0 - xMin,y0 - yMin, self.weightmap[y0][x0], tempMap)
                    break

                if e2 < dx:
                    err = err + dx
                    y0 = y0 + sy
                    
            
        #self.drawTargetMap(tempMap)
        
        # Use the mask to calculate direction
        
        xWeight = 0
        yWeight = 0
        fullweight = 0
        xMid = x - xMin
        yMid = y - yMin
    
        for xv in xrange(0, len(tempMap[0])):
            for yv in xrange(0, len(tempMap)):
                val = tempMap[yv][xv]
                xWeight += (xv - xMid) * val
                yWeight += (yv - yMid) * val
                fullweight += val

        """if fullweight > self.weightLimit:
            self.drawTargetMap(tempMap)
            #print "direction (%.2f %.2f)" % (xWeight / max(xWeight, yWeight), yWeight / max(xWeight, yWeight))
            print "Weights x: %d y: %d" % (xWeight, yWeight)
        """
        
        #print "Weights x: %d y: %d" % (xWeight, yWeight)
    
        return (math.atan2(yWeight, xWeight), fullweight)

    def isClear(self, x, y):
    
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False

        if self.map[y][x] == 7:
            # Collided a wall, end drawing
            return False

        return True

    def drawTargetMap(self, map):
        mapString = ''
        for y in xrange(len(map)):
            for x in xrange(len(map[0])):
                mapString += '{:1.0f}'.format(map[y][x])
            mapString += '\n'

        print mapString

    def get_next_point(self, algo=1):

        if algo == 1:
            if not self.ends or len(self.ends) == 0:
                return (-1, -1)

            # Go through vision endpoints and take the middle one

            index = int(len(self.ends) / 2)
            point = self.ends[index]

            return point

        elif algo == 2:
            # Get a random point from searched area
            p = (-1, -1)

            value = 0
            
            cont = False

            while not cont: #p[0] < 0 and (value <= 0 or value > 6):
                x = random.randrange(len(self.map[0]))
                y = random.randrange(len(self.map))

                p = (x, y)
                
                value = self.map[p[1]][p[0]]

                if value > 0 and value < 7:
                    cont = True

            #print "Random value: %d" % value
            return p

        return (-1, -1)
        
    
    def get_next_pose(self):
    
        # Get a random point from searched area
        p = (-1, -1, 0)

        value = 0
        
        self.reorder_interestpoints()
        
        # First get closest interestpoints
        for point in list(self.viewpoints):
            x = point[0]
            y = point[1]
        
            print "Interestpoint %d, %d (%.2f)" % (x, y, point[2])
            
            value = self.map[y][x]
        
            if value <= 0 or value > 6:
                continue
        
            dir = self.direction_from_point(x, y)
        
            # Check for weight map usability
            if dir[1] > self.weightLimit:
                p = (x, y, dir[0])
                self.viewpoints.remove(point)
                return p

        cont = 0
        
        while cont <= 500:
            x = random.randrange(len(self.map[0]))
            y = random.randrange(len(self.map))
        
            value = self.map[y][x]
        
            if value <= 0 or value > 6:
                continue
        
            dir = self.direction_from_point(x, y)
        
            # Check for weight map usability
            if dir[1] > self.weightLimit or cont >= 500:
                p = (x, y, dir[0])
                return p

        return p
        
    def get_pose_list(self, pose):
        x = pose[0]
        y = pose[1]
        angle = pose[2]
        usableAngle = self.angle * self.angle_usability
        
        # Get all smart image angles taken from this position
        count = (360) / (usableAngle)
        
        poses = []
        
        print "Position: (%d, %d)" % (x, y)
        
        poses.append(pose)
        
        for i in xrange(int(math.ceil(count))):
            a = angle + i * usableAngle

            poses.append((x, y, a))

                
        return poses


    def next_pose_callback(self,data):
        print 'Got called for next point with msg: '+str(data)
        if self.map is None or self.resolution is None:
            print 'Explorer not initialized yet!'
            return
        p = self.get_next_pose()
        quaternion = quaternion_from_euler(0,0,p[2])
        next_pose = PoseStamped()
        next_pose.header.frame_id = 'map'
        next_pose.header.stamp = rospy.Time.now()
        next_pose.pose.position.x = p[0]
        next_pose.pose.position.y = p[1]
        next_pose.pose.orientation.w = quaternion[3]
        next_pose.pose.orientation.z = quaternion[2]
        print 'Next point to explore: ('+str(p[0])+', '+str(p[1])+') with orientation '+str(p[2])
        self.goal_pub.publish(next_pose)
    
    def explore(self):
                
        tst = self.calculate_weightmap()
        if tst is None:
            # All done, can exit
            return None
        cont = True
        
        if self.firstcall:
            pose = (self.pos_x, self.pos_y, self.angle)
            self.poselist = self.get_pose_list(pose)
            self.poseindex = 0
            self.firstcall = False
        
        if len(self.poselist) == 0:
            pose = self.get_next_pose()
            poseList = self.get_pose_list(pose)
            self.poseindex = 0
        
        # Iterate through poses
        p = self.poselist[self.poseindex]
        
        #self.pos_x = p[0]
        #self.pos_y = p[1]
        #self.target_angle = math.degrees(p[2])
        self.poseindex = self.poseindex + 1
        
        
        if poseindex >= len(self.poselist):
            self.poselist = []
            cont = False
        
        #self.detector_callback()
        
        # Return next pose and wish to take more images
        return (p[0], p[1], p[2], cont)
        
        
        
        
            

if __name__ == "__main__":
    rospy.init_node('explorer')
    rospy.sleep(1.0)
    expl = Explorer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down pd node.'

