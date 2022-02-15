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
            return sensorValues # No need to translate
        elif abs(90-heading) < self.max_heading_error: # Facing West
                    # North         # West          # South          # East
            return [sensorValues[1], sensorValues[0], sensorValues[3], sensorValues[2]]
        elif abs(-90-heading) < self.max_heading_error: # Facing East
                    # North         # West          # South          # East
            return [sensorValues[3], sensorValues[2], sensorValues[1], sensorValues[0]]
        elif abs(180-heading) < self.max_heading_error or abs(-180-heading) < self.max_heading_error: # Facing South
                    # North         # West          # South          # East
            return [sensorValues[2], sensorValues[1], sensorValues[0], sensorValues[3]]
        else:
            return [False] * 4
    def updateGridWalls(self, pose, heading, sensors):
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
        if self.map[y][x] == '?':
            self.map[y][x] = '#' if wall else '+'
            return True
        return False

    def prettyPrintMap(self):
        print("  ", end='')
        for i in range(len(self.map)):
            print("{} ".format(str(i).zfill(2)), end='')
        print()
        for row in range(len(self.map)):
            print(str(row).zfill(2), end='')
            for point in range(len(self.map[row])):
                print(" {} ".format(self.map[row][point]), end="")
            print()
        print('-------------------------------')