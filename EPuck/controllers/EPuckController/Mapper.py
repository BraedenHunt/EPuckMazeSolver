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

        self.map = [['?'] * columns] * rows

    def get_grid_pos(self, pose):
        x = round((self.rows / 2) - 1 - (-pose[0] - self.grid_spacing/2.0) / self.grid_spacing)
        y = round((self.columns / 2) - 1 - (pose[1] - self.grid_spacing/2.0) / self.grid_spacing)
        return [x,y]

    def translateSensors(self, heading, sensorValues):
        if -1 <= heading and heading <= 1: # Facing North
            return sensorValues # No need to translate
        elif 89 <= heading and heading <= 91: # Facing West
                    # North         # West          # South          # East
            return [sensorValues[1], sensorValues[0], sensorValues[3], sensorValues[2]]
        elif -89 >= heading and heading >= -91: # Facing East
                    # North         # West          # South          # East
            return [sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[0]]
        elif -179 <= heading or heading >= 179 : # Facing South
                    # North         # West          # South          # East
            return [sensorValues[2], sensorValues[3], sensorValues[0], sensorValues[1]]
