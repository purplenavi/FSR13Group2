"""
MUST check is it 0 or 100 the value given from laser when contains something.
"""

from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from operator import xor
from tf.transformations import euler_from_quaternion

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
        self.resolution = mapinfo.  resolution # resolution gotten from MapMetaData (m/cell)
        self.map = np.zeros((mapinfo.height,mapinfo.width)) # same size as map
        self.pose = mapinfo.origin # current pose (has point and orientation)

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
        robotx = self.pose.point.x #150
        roboty = self.pose.point.y #150

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
        unexplored = np.where(A == 0) # Giving the indexes of map containing the zeros
        if len(unexplored) == 0:
            print 'Whole map is checked out so the exploring is done!';
            return None
        # Then should somehow find by some logic the next "smart" point to go. Ideas?
