# This file generates somewhat evenly distributed points in a given polygon. 
# The polygon can take on any shape. The algorithm works by accepting the 
# points of the polygon and the number of traps to be placed down. 

import shapely 
import math
import random

# The TrapZone class represents the area that traps need to be distributed
# in. It is assumed that the entire TrapZone is a fly-zone. 
class TrapZone(object):
    
    # zone - The polygon that specifies the area for placing traps
    # num_traps - the number of traps to be distributed
    # bufAmt - specifies how much to erode the polygon. Basically, it answers
    #         ...the question of how safe you want to be regarding how close
    #         ... the traps get to the no-fly zone.
    # erode - a boolean specifying whether the polygon should shrink a bit
    #         ...to ensure that traps aren't place close to the no-fly zones.
    def __init__(self, zone, obstacles, num_traps):
        self.zone = zone
        self.obstacles = obstacles
        self.num_traps = num_traps
        
        
    # function that generates the locations of the traps.
    # returns a list of xy pairs for each trap. The algorithm finds a 
    # rectangle that surrounds the polygon. Then it evenly distributes 
    # points inside that rectangle. Then, the program iterates through
    # all of the points and only keeps the ones that are contained inside
    # the polygon. If the number of traps that are kept is less than the 
    # number of traps specified by the user, then the process repeats with 
    # a denser distribution of traps in the rectangle. If there are more 
    # traps in the polygon than the user specified, then the extras are 
    # randomly removed. 
    def genPoints(self):
        bounds = self.zone.bounds # get the corners of the rectangle that contain the Polygon
        width = math.ceil(bounds[2]-bounds[0])
        height = math.ceil(bounds[3]-bounds[1])
        fullTraps = False # Distribute the traps in the rectangle 
        num_points = self.num_traps
        while not fullTraps:
            # Generate coordinates in a square
            points = self.genCoords(num_points,width,height)
            points = [[x[0]+bounds[0],x[1]+bounds[1]] for x in points]
            points = self.pointsInZone(points)
            
            # Check how many points are located in the zone
            inZone = len(points[0])
            if inZone < self.num_traps:
                # Growth equation for point generation in rectangle (modify)
                num_points = num_points + self.num_traps
            else:
                fullTraps = True  # There are enough traps in the zone
                random.seed() # Initialize a random seed
                while inZone != self.num_traps:
                    index = random.randint(0,inZone-1)
                    points[0] = points[0][:index] + points[0][index+1 :]
                    inZone = inZone - 1
        return points
                
    def genCoords(self, elements, width, height):
        # Compute some intermediate numbers
        py = (height/(height+width))*elements
        ratio = ((elements*height)/(width*(py**2)))**.5
        ry = ratio*py # Number of points on y axis (height)
        inc = height/ry # increment for x and y to make points evenly spaced
        points = []
        w = 0
        while w < (width-inc*.25):
            h = 0
            while h < (height-inc*.25):
                points = points + [(w,h)]
                h = h + inc
            w = w + inc
        # Center the points in the rectangle
        x_shift = (width-points[len(points)-1][0])/2
        y_shift = (height-points[len(points)-1][1])/2
        points = [[x[0]+x_shift,x[1]+y_shift] for x in points]
        return points
    
    def pointsInZone(self, points):
        resultList = [[],[]]
        for x in points:
            if self.zone.contains(shapely.geometry.Point(x)):
                resultList[0] = resultList[0] + [x]
            else: 
                resultList[1] = resultList[1] + [x]
        return resultList



def drawPoints(canvas, points, size, color):
    for x in points:
        canvas.create_oval(x[0]-size,x[1]+size,x[0]+size,x[1]-size,fill=color)
        
        