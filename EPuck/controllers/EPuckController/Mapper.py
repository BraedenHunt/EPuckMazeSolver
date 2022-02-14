import math


class Mapper:

    def __init__(self, rows, columns, grid_spacing):

        self.rows = rows
        self.columns = columns
        self.grid_spacing = grid_spacing

        self.map = [['?'] * columns] * rows

    def get_grid_pos(self, pose):
        x = round((self.rows / 2) - 1 - (-pose[0] - self.grid_spacing/2.0) / self.grid_spacing)
        y = round((self.columns / 2) - 1 - (pose[1] - self.grid_spacing/2.0) / self.grid_spacing)
        return [x,y]
