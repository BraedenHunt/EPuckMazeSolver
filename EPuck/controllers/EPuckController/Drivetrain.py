import math
from math import atan2

from controller import Robot, Motor, Compass

class Drivetrain:

    def __init__(self, robot: Robot, max_speed):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.MAX_SPEED = max_speed
        self.right_motor: Motor = robot.getDevice('right wheel motor')
        self.left_motor: Motor = robot.getDevice('left wheel motor')

        self.compass: Compass = robot.getDevice('compass')
        self.compass.enable(self.timestep)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

    def drive(self, left: float, right: float):
        self.left_motor.setVelocity(self.bound(left, -1, 1) * self.MAX_SPEED)
        self.right_motor.setVelocity(self.bound(right, -1, 1) * self.MAX_SPEED)

    def get_heading(self):
        values = self.compass.getValues()
        #print(values)
        rad = atan2(values[0], values[1])
        bearing = (rad - 1.5708) / math.pi * 180.0
        if bearing < 0:
            bearing = bearing + 360.0
        return bearing

    @classmethod
    def bound(cls, value: float, min_value: float, max_value: float):
        return max(min(max_value, value), min_value)
