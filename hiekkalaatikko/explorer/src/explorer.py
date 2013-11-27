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
        self.ends = []
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
            x0 = robotx + self.min_distance * self.resolution * math.cos(targetangle)
            y0 = roboty + self.min_distance * self.resolution * math.sin(targetangle)

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


    def calculate_weightmap(self, msg):
        if self.map is None or self.pose is None or self.resolution is None:
            print 'Cannot get next point, Explorer not initialized yet'
            return
        time_start = rospy.get_time()
        # There's no reason for me to do anything with the msg, wadap...
        print 'Wow! I just got a message from task planner: '+str(msg)
        unexplored = np.where(self.map == 0) # Giving the indexes of map containing the zeros
        if len(unexplored) == 0:
            print 'Whole map is already checked out so the exploring is done!';
            return None
        weightmap = np.zeros((len(self.map),len(self.map[0])))
        # Check smallest distance from walls, obstacles or prechecked point for each zero point
        print 'Starting weightmap calculation, unexplored: ' + str(len(unexplored[0]))
        
        searchdistance = 8
        
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
                
                self.plotMap(x0 - xMin,y0 - yMin, self.weightmap[y0][x0], tempMap)

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
    	
    	for xv in xrange(0, len(tempMap[0])):
    		for yv in xrange(0, len(tempMap)):
    			val = tempMap[yv][xv]
    			xWeight += (xv - x) * val
    			yWeight += (yv - y) * val
    	
    	#print "Weights x: %d y: %d" % (xWeight, yWeight)
    	
    	return math.atan2(yWeight, xWeight)
    	
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

    def get_next_point(self):
    	
    	if not self.ends or len(self.ends) == 0:
    		return (0, 0)
    	
    	# Go through vision endpoints and take the middle one
    	
    	index = int(len(self.ends) / 2)
    	point = self.ends[index]
    	
    	return point




if __name__ == "__main__":
    rospy.init_node('explorer')
    rospy.sleep(1.0)
    expl = Explorer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down pd node.'

