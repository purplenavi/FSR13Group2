"""
MUST be imported to mcgui or added imports etc here

TODO:
- Add Explorer initialization to first laser callback with whole messages given as parameter.
- Give data gotten from 
"""
class Explorer:

    def __init__(self,laserinfo):
        self.angle = 55 # angle in where camera can detect pirates ni degrees
        self.distance = 1.5 # distance where camera can detect pirates in meters
        self.map = np.zeros(laserinfo.info.height,laserinfo.info.width) # same size as map
        self.laser_callback(laserinfo)

    def laser_callback(self,reshaped_data):
        self.map = self.map + reshaped_data
        self.map = np.array(self.map > 50, dtype=int)

    def get_point(self):
        # How to find a large area of zeros from self.map?
