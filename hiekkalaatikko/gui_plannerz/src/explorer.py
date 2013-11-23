"""
MUST check is it 0 or 100 the value given from laser when contains something.
"""

from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseStamped
from operator import xor
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseGoal
import math

# Explorer map
# 0 = unchecked
# 1 = checked and clear
# 2 = wall
class Explorer:

    def __init__(self, parent, mapinfo):
        self.parent = parent
        self.angle = 55 # angle in where camera can detect pirates ni degrees
        self.min_distance = 0.2 # minimum distance where camera can detect pirates in meters
        self.max_distance = 1.5 # maximum distance where camera can detect pirates in meters
        self.resolution = mapinfo.resolution # resolution gotten from MapMetaData (m/cell)
        self.map = np.zeros((mapinfo.height,mapinfo.width)) # same size as map
        self.pose = mapinfo.origin # current pose (position and orientation)

    # Current pose has to be updated on every scan too
    def update_info(self, mapinfo):
        self.resolution = mapinfo.resolution # scan resolution gotten from MapMetaData (m/cell)
        self.pose = mapinfo.origin # current pose (position and orientation)

    # Method to update map with laser callback data
    def laser_callback(self,reshaped_data):
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
        
        # Robot coordinates on map
        robotx = self.pose.position.x #150
        roboty = self.pose.position.y #150

        robotangle = euler_from_quaternion(self.pose.orientation.x,self.pose.orientation.y) #40
        
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
     		 
     		 	if not plot(x0, y0):
     		 		break
     		 
     		 	if x0 == x1 and y0 == y1:
     		 		break
     		 		
     		 	e2 = 2 * err
     			if e2 > -dy: 
       				err = err - dy
       				x0 = x0 + sx

     			if x0 = x1 and y0 = y1:
       				plot(x0,y0)
       				break

     			if e2 < dx:
       				err = err + dx
       				y0 = y0 + sy 

	def plot(x, y):
		if self.map[y][x] == 2:
         	# Collided a wall, end drawing
         	return False
         			
         # Draw
         self.map[y][x] = 1
         return True


    def get_next_point(self):
        unexplored = np.where(self.map == 0) # Giving the indexes of map containing the zeros
        if len(unexplored) == 0:
            print 'Whole map is checked out so the exploring is done!';
            return None
        weightmap = np.zeros((len(self.map),len(self.map[0])))
        # Check smallest distance from walls, obstacles or prechecked point
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
            while it < mindist and y-it > 0:
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
            while it < mindist and x-it > 0:
                if self.map[y][x-it] > 0:
                    break
                it += 1
            if mindist < it:
                mindist = it
            weightmap[y][x] = mindist
        # Get maximum valued point that ain't behind wall (no 2 in map in the path)
        new_points = np.where(weightmap==weightmap.max()):
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
        if x is None or y is None:
            print 'Iz gone bad! No new point found but behind walls even though it should exist'
            return None
        else:
            # Create MoveBaseGoal and return it
            goal = PoseStamped()
            quaternion = quaternion_from_euler(0, 0, math.atan2(y-self.pose.position.y, x-self.pose.position.x))
            goal.header.frame_id = 'map'
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = self.pose.orientation.w
            goal.pose.orientation.z = self.pose.orientation.z
            goal = MoveBaseGoal(target_pose=goal)
            print 'Next unexplored goal given at ('+str(x)+', '+str(y)+')'
            return goal
