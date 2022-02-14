from controller import Robot, DistanceSensor

class SonicSensors:
    DISTANCE_THRESHOLD = 80

    device_names = ['ps0', 'ps1','ps2','ps3','ps4','ps5','ps6','ps7']
    sensors = []

    def __init__(self, robot, timestep = 64):
        self.robot = robot
        self.timestep = timestep

        for i in range(len(self.device_names)):
            self.sensors.insert(i, robot.getDevice(self.device_names[i]))
            self.sensors[i].enable(self.timestep)


    def update(self):
        '''for i in [2]:#range(len(self.sensors)):
            print("{}: {}".format(self.device_names[i], self.sensors[i].getValue()))
        '''
        self.printGrid(self.get_grid())

    def get_grid(self):
        return [self.is_wall(self.sensors[0]) or self.is_wall(self.sensors[7]), # Front
                self.is_wall(self.sensors[5]), # Right
                self.is_wall(self.sensors[3]) or self.is_wall(self.sensors[4]), # Back
                self.is_wall(self.sensors[2])] # Left

    @classmethod
    def is_wall(cls, sensor):
        return sensor.getValue() > cls.DISTANCE_THRESHOLD

    @classmethod
    def printGrid(cls, grid):
        print()
        print("?{}?\n{}-{}\n?{}?".format(cls.printGridHelper(grid[0]), cls.printGridHelper(grid[1]), cls.printGridHelper(grid[3]), cls.printGridHelper(grid[2])))
        print()

    def printSensorValues(self):
        print()
        print("?{}?\n{}-{}\n?{}?".format(max(self.sensors[0].getValue(), self.sensors[7].getValue()), self.sensors[5].getValue(),
                                         self.sensors[2].getValue(), max(self.sensors[3].getValue(), self.sensors[4].getValue())))
        print()
    @classmethod
    def printGridHelper(cls, bool):
        return '#' if bool else '0'