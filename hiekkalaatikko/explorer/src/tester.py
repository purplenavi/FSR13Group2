import math
import numpy as np
import random
import scipy
import matplotlib.pyplot as plt

class Explorer:

    # Actual initialization happens at first laser callback where the current pose, map size and resolution are defined.
    def __init__(self):
        self.resolution = 0.04
        self.width = 100
        self.height = 50
        self.angle = 55
        self.angle_usability = 0.9
        self.min_distance = 0.2
        self.max_distance = 1.4
        self.map = np.zeros((self.height, self.width))
        self.weighting_distance = 2.0 # Radius in meters
        self.ends = []
        self.viewpoints = []
        self.view_xcount = 5
        self.view_ycount = 2
        self.weightLimit = 1000
        self.directions = [(0, 0), (0, 1), (1, 0), (0, -1), (-1, 0)]
        self.poselist = []
        self.poseindex = 0
        self.firstcall = True
        
        self.pos_x = 80
        self.pos_y = 30
        self.target_angle = 210
        self.firstcall = True
        
        # Draw some test walls
        for x in xrange(60):
        	self.map[20][x+10] = 7
        	self.map[19][x+10] = 7
        
    def detector_callback(self):

        # Robot coordinates on map
        robotx = self.pos_x
        roboty = self.pos_y

        robotangle = self.target_angle
        
        self.ends = []

        beamangle = robotangle - (self.angle / 2)
        step = 0.5

        # Mark area checked
        for a in xrange(int(self.angle / step)):
            targetangle = math.radians(beamangle + a * step)

            # Mark both ends of line
            x0 = robotx
            y0 = roboty
            
            xm = robotx + self.min_distance * self.resolution * math.cos(targetangle)
            ym = roboty + self.min_distance * self.resolution * math.sin(targetangle)

            x1 = int(robotx + self.max_distance / self.resolution * math.cos(targetangle))
            y1 = int(roboty + self.max_distance / self.resolution * math.sin(targetangle))
            
            #print "Line at (%d, %d)" % (x0, y0)
            
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
                    self.plot(x0,y0, True)
                    break
                  
                e2 = 2 * err
                if e2 > -dy: 
                    err = err - dy
                    x0 = x0 + sx

                if x0 == x1 and y0 == y1:
                    self.plot(x0,y0, True)
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
        
        self.drawTargetMap(self.map)
        #plt.imsave('map2.png', self.map)


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
            coords = (x, y)
            if coords not in self.ends:
                #print "End at (%d, %d)" % (x, y)
                self.ends.append(coords)
            return True
    
        if self.map[y][x] == 0:
            self.map[y][x] = 1
        return True
        
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
    
    def drawMap(self):
        mapString = ''
        for y in xrange(self.height):
            for x in xrange(self.width):
                mapString += '{:1.0f}'.format(self.map[y][x])
            mapString += '\n'
    
        print mapString

    def drawWeightMap(self):
        mapString = ''
        for y in xrange(self.height):
            for x in xrange(self.width):
                mapString += '{:1.0f}'.format(self.weightmap[y][x])
            mapString += '\n'
    
        print mapString

    def calculate_weightmap(self):

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

        #scipy.misc.imsave('outfile.jpg', self.weightmap)
        
        #plt.imshow(self.weightmap, filterrad=1.0) #Needs to be in row,col order
        #plt.savefig('weight.png')
        #plt.imsave('we.png', self.weightmap)

        """# Get maximum valued point that ain't behind wall (no 2 in map in direct path)
        new_points = np.where(weightmap==weightmap.max())
        # x and y are cell indexes here
        x = None
        y = None
        while weightmap.max() > 0 and len(new_points) > 0:
            for i in range(len(new_points[0])):
                distance = math.sqrt(math.pow(new_points[1][i]-self.pose.position.x,2)+math.pow(new_points[0][i]-self.pose.position.y,2))
                angle = math.atan2(new_points[0][i]-self.pose.position.y,new_points[1][i]-self.pose.position.x)
                x = new_points[0][i]
                y = new_points[1][i]
                for j in range(int(math.ceil(distance))): # loop through every cell in the way
                    point = (self.pose.position.x + j*math.cos(angle), self.pose.position.y + j*math.sin(angle))
                    y1 = int(math.floor(point[0]))
                    y2 = int(math.ceil(point[0]))
                    x1 = int(math.floor(point[1]))
                    x2 = int(math.ceil(point[1]))
                    # Check points rounded around this point ain't an obstacle
                    if self.map[y1][x1] == 2:
                        x = None
                        y = None
                        break
                    if self.map[y1][x2] == 2:
                        x = None
                        y = None
                        break
                    if self.map[y2][x1] == 2:
                        x = None
                        y = None
                        break
                    if self.map[y2][x2] == 2:
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
            x += int(len(self.map[0])/2)
            y += int(len(self.map)/2)
            x *= self.resolution
            y *= self.resolution
            # Create PoseStamped and return it
            goal = PoseStamped()
            quaternion = quaternion_from_euler(0, 0, math.atan2(y-self.pose.position.y, x-self.pose.position.x))
            goal.header.frame_id = 'map'
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y 
            goal.pose.orientation.w = quaternion[3]
            goal.pose.orientation.z = quaternion[2]
            print 'Next unexplored goal publish at ('+str(x)+', '+str(y)+'), calculation took '+str(rospy.get_time()-time_start)+' seconds..'
            self.goal_pub.publish(goal)"""

    """def get_next_point(self, algo=1):

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

            print "Random value: %d" % value
            return p

        return (-1, -1)"""
        
    
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

    def reorder_interestpoints(self):
        for i in xrange(len(self.viewpoints)):
            # Count distance to current position
            x = self.viewpoints[i][0]
            y = self.viewpoints[i][1]
            
            dist = math.sqrt(math.pow(self.pos_x - x,2) + math.pow(self.pos_y - y,2))
            self.viewpoints[i] = (x, y, dist)
            
        self.viewpoints = sorted(self.viewpoints, key=lambda point: point[2])
    
    def direction_from_point(self, x, y, targetAngle=0, angle=360):
        radius = self.max_distance / self.resolution
    
        # Clamp area
        xMin = int(max(0, x - radius))
        yMin = int(max(0, y - radius))
        xMax = int(min(self.width - 1, x + radius))
        yMax = int(min(self.height - 1, y + radius))
    
        tempMap = np.zeros((yMax - yMin, xMax - xMin))
    
        step = 0.5
        beamangle = targetAngle - (self.angle / 2)
    
        for a in xrange(int(angle / step)):
            targetangle = math.radians(beamangle + a * step)
            
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
        #plt.imsave('tempmap.png', tempMap)

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
                        
            #weight = self.direction_from_point(x, y, a, self.angle)
            
            #s = ""
            
            #if (weight[1] > 10):
                # This angle is usable, add to return value
            poses.append((x, y, a))
            #    s = "OK"
            
            #print "Angle: %.2f %.2f" % (a, weight[1])

                
        return poses

    def test(self):

        #print "Take image from (%d, %d) to direction %d" % (self.pos_x, self.pos_y, self.target_angle)

        #self.detector_callback()

        #expl.drawMap()

        #expl.drawWeightMap()

        #point = self.get_next_point(2)
    
        #print "Next point: (%d, %d)" % (point[0], point[1])

        #if (point[0] == -1 or point[1] == -1):
        #    return False

        self.calculate_weightmap()
    
        pose = self.get_next_pose()
        
        poseList = self.get_pose_list(pose)
        for p in poseList:
            # Take images to all poses
            self.pos_x = p[0]
            self.pos_y = p[1]
            self.target_angle = math.degrees(p[2])
            
            self.detector_callback()
            
            self.drawTargetMap(self.map)
            
            raw_input("Image")
            
    
        #dir = expl.direction_from_point(point[0], point[1])
        #print "Full weight: %d" % dir[1]

        self.pos_x = pose[0]
        self.pos_y = pose[1]
        self.target_angle = math.degrees(pose[2])

        return True
        
    def explore(self):
        
        self.calculate_weightmap()
        cont = True
        
        if self.firstcall:
            pose = (self.pos_x, self.pos_y, self.angle)
            self.poselist = self.get_pose_list(pose)
            self.poseindex = 0
            self.firstcall = False
        
        if len(self.poselist) == 0:
            print "New pose list"
            pose = self.get_next_pose()
            self.poselist = self.get_pose_list(pose)
            self.poseindex = 0
        
        # Iterate through poses
        p = self.poselist[self.poseindex]
        
        #self.pos_x = p[0]
        #self.pos_y = p[1]
        #self.target_angle = math.degrees(p[2])
        self.poseindex = self.poseindex + 1
        
        
        if self.poseindex >= len(self.poselist):
            self.poselist = []
            self.poseindex = 0
            cont = False
        
        #self.detector_callback()
        
        # Return next pose and wish to take more images
        return (p[0], p[1], p[2], cont)


    def plotMap(self, x, y, value, targetMap):

        if x < 0 or y < 0 or x >= len(targetMap[0]) or y >= len(targetMap):
            return

        if targetMap[y][x] == 0:
            targetMap[y][x] = value


    
if __name__ == "__main__":
    expl = Explorer()

    # Take initial image
    expl.detector_callback()
    
    while True:
        pose = expl.explore()
        expl.pos_x = pose[0]
        expl.pos_y = pose[1]
        expl.target_angle = math.degrees(pose[2])
        if pose[3]:
            print "Wanna roll!!"
        expl.detector_callback()
    
    """while expl.test():
        expl.drawTargetMap(expl.map)
        raw_input("Wait")

    expl.drawTargetMap(expl.map)

    #for x in xrange(100):
    	#expl.detector_callback()"""
    
    """expl.detector_callback()
        
    expl.drawMap()
    
    expl.calculate_weightmap()
    
    expl.drawWeightMap()
    
    point = expl.get_next_point()
    
    dir = expl.direction_from_point(point[0], point[1])
    
    print point
    
    expl.pos_x = point[0]
    expl.pos_y = point[1]
    expl.target_angle = math.degrees(dir)
    
    expl.detector_callback()
    
    expl.drawMap()"""
    
    #print expl.ends

    
    
    
