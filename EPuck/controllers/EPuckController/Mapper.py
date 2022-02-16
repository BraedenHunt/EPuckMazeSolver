import math


class Mapper:

    '''
    ? - unknown value
    # - wall
    + - pathway
    '''
    def __init__(self, rows, columns, grid_spacing):

        self.rows = rows
        self.columns = columns
        self.grid_spacing = grid_spacing
        self.generateMap()
        self.max_heading_error = 1

    def get_grid_pos(self, pose):
        x = round((self.rows / 2) - (-pose[0] - self.grid_spacing/2.0) / self.grid_spacing)
        y = round((self.columns / 2) - (pose[1] - self.grid_spacing/2.0) / self.grid_spacing)
        return [x, y]

    def generateMap(self):
        self.map = [[char for char in ('#' + '?' * self.columns + '#')] for _ in range(0, self.rows + 2)]
        self.map[0] = ['#'] * (self.columns + 2)
        self.map[-1] = ['#'] * (self.columns + 2)

    def translateSensors(self, heading, sensorValues): #
        '''
        0 - F
        1 - R
        2 - B
        3 - L
        '''
        if abs(heading) < self.max_heading_error: # Facing North
            #print("facing north")
            return sensorValues # No need to translate
        elif abs(90-heading) < self.max_heading_error: # Facing West
                    # North         # East          # South          # West
            #print("facing west")
            return [sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[0]]
        elif abs(-90-heading) < self.max_heading_error: # Facing East
                    # North         # East          # South          # West
            return [sensorValues[3], sensorValues[0], sensorValues[1], sensorValues[2]]
            #print("facing east")

        elif abs(180-heading) < self.max_heading_error or abs(-180-heading) < self.max_heading_error: # Facing South
            #print("facing south")
                    # North         # East          # South          # West
            return [sensorValues[2], sensorValues[3], sensorValues[0], sensorValues[1]]
        else:
            return [False] * 4
    def updateGridWalls(self, pose, heading, sensors):
        #print("Heading: {}".format(heading))
        translatedSensors = self.translateSensors(heading, sensors)
        gridPose = self.get_grid_pos(pose)
        returnVal = False
        if translatedSensors[0]: # North
            returnVal = returnVal or self.setWall(gridPose[0], gridPose[1] - 1, True)
        if translatedSensors[3]: # West
            returnVal = returnVal or self.setWall(gridPose[0]-1, gridPose[1], True)
        if translatedSensors[2]: # South
            returnVal = returnVal or self.setWall(gridPose[0], gridPose[1] + 1, True)
        if translatedSensors[1]: # East
            returnVal = returnVal or self.setWall(gridPose[0]+1, gridPose[1], True)
        returnVal = returnVal or self.setWall(gridPose[0], gridPose[1], False)
        return returnVal

    def setWall(self, x, y, wall):
        #print("setting {} @ {}, {}".format('#' if wall else '+', x, y))
        #print("row len: {} col len: {}".format(len(self.map), len(self.map[y])))
        if self.getPos(x, y) == '?':
            self.map[y][x] = '#' if wall else '+'
            return True
        return False

    def prettyPrintMap(self, pose=[-100,-100]):
        print("  ", end='')
        for i in range(len(self.map)):
            print("{} ".format(str(i).zfill(2)), end='')
        print()
        for row in range(len(self.map)):
            print(str(row).zfill(2), end='')
            for point in range(len(self.map[row])):
                if point == pose[0] and row == pose[1]:
                    print(" * ", end="")
                else:
                    print(" {} ".format(self.map[row][point]), end="")
            print()
        print('-------------------------------')

    def getPos(self, x, y):
        return self.map[y][x]

    def getPosFromPose(self, pose):
        return self.map[pose[1]][pose[0]]

    '''
    returns a list of coordinates around the robot relative to the robot heading
    '''
    def getRobotRelativeAdjPoses(self, heading, pose):
        '''
        0 - F
        1 - R
        2 - B
        3 - L
        '''
        x = pose[0]
        y = pose[1]
        if abs(heading) < self.max_heading_error:  # Facing North
            return [[x, y-1], [x+1, y], [x, y+1], [x-1, y]]
        elif abs(90 - heading) < self.max_heading_error:  # Facing West
            return [[x-1, y], [x, y-1], [x+1, y], [x, y+1]]
        elif abs(-90 - heading) < self.max_heading_error:  # Facing East
            return [[x+1, y], [x, y+1], [x-1, y], [x, y-1]]
        elif abs(180 - heading) < self.max_heading_error or abs(
                -180 - heading) < self.max_heading_error:  # Facing South
            return [[x, y + 1], [x - 1, y], [x, y - 1], [x + 1, y]]
