"""
MUST check is it 0 or 100 the value given from laser when contains something.
"""

# Explorer map
# 0 = unchecked
# 1 = checked and clear
# 2 = wall
class Explorer:

    def __init__(self, parent):
        self.parent = parent
        self.angle = 55 # angle in where camera can detect pirates ni degrees
        self.min_distance = 0.2 # minimum distance where camera can detect pirates in meters
        self.max_distance = 1.5 # maximum distance where camera can detect pirates in meters
        self.resolution = 10 # TODO: resolution (how long is a pixel)?
        self.map = np.zeros((self.parent.h,self.parent.w)) # same size as map
        # Origin of the robot is self.parent.origin straighly (x,y)

    # Method to update map with laser callback data
    def laser_callback(self,reshaped_data):
        # self.map += reshaped_data # Add or values gotten and reshaped from laser
        # self.map = np.array(self.map > 50, dtype=int) # Convert to binary
        
        # Update map?
        for y in xrange(parent.h):
        	for x in xrange(parent.w):
        		if reshaped_data[y][x] > 50:
        			self.map[y][x] = 2
        		elif reshaped_data[y][x] < 50 and self.map[y][x] == 2:
        			self.map[y][x] = 0
        

    # Method for detector callbacks (pcl)
    def detector_callback(self, data):
        # TODO: get position on map
        
        # Robot coordinates on map
        robotx = 150
        roboty = 150
        
        robotangle = 40
        
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
