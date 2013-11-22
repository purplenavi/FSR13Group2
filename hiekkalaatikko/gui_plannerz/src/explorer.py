"""
MUST check is it 0 or 100 the value given from laser when contains something.
"""

# Explorer contains a binary array (1 = checked point) of the same size than map servers map.
class Explorer:

    def __init__(self, parent):
        self.parent = parent
        self.angle = 55 # angle in where camera can detect pirates ni degrees
        self.min_distance = 0.2 # minimum distance where camera can detect pirates in meters
        self.max_distance = 1.5 # maximum distance where camera can detect pirates in meters
        self.map = np.zeros((self.parent.h,self.parent.w)) # same size as map
        # Origin of the robot is self.parent.origin straighly (x,y)

    # Method to update map with laser callback data
    def laser_callback(self,reshaped_data):
        self.map += reshaped_data # Add or values gotten and reshaped from laser
        self.map = np.array(self.map > 50, dtype=int) # Convert to binary

    # Method for detector callbacks (pcl)
    def detector_callback(self, data):
        # Jussi's job
	pass

    def get_next_point(self):
        unexplored = np.where(A == 0) # Giving the indexes of map containing the zeros
        if len(unexplored) == 0:
            print 'Whole map is checked out so the exploring is done!';
            return None
        # Then should somehow find by some logic the next "smart" point to go. Ideas?

