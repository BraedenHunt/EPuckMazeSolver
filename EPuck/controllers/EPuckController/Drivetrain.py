import math
from math import atan2

from controller import Robot, Motor, Compass, PositionSensor

class Drivetrain:
    WHEEL_DIAMETER = 0.041 # meters (41mm)

    def __init__(self, robot: Robot, max_speed, max_accel):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.MAX_SPEED = max_speed
        self.MAX_ACCELERATION = max_accel
        self.right_motor: Motor = robot.getDevice('right wheel motor')
        self.left_motor: Motor = robot.getDevice('left wheel motor')

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.left_encoder: PositionSensor = robot.getDevice('left wheel sensor')
        self.right_encoder: PositionSensor = robot.getDevice('right wheel sensor')

        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)

        self.left_encoder_offset = 0
        self.right_encoder_offset = 0


        self.compass: Compass = robot.getDevice('compass')
        self.compass.enable(self.timestep)

    def drive(self, left: float, right: float):
        self.left_motor.setVelocity(self.bound(left, -1, 1) * self.MAX_SPEED)
        self.right_motor.setVelocity(self.bound(right, -1, 1) * self.MAX_SPEED)

    def get_heading(self):
        values = self.compass.getValues()
        #print(values)
        rad = atan2(values[0], values[1])
        bearing = ((rad - 1.5708) / math.pi * 180.0) % 360
        if bearing > 180:
            bearing = bearing - 360.0
        return bearing

    @classmethod
    def bound(cls, value: float, min_value: float, max_value: float):
        return max(min(max_value, value), min_value)

    def get_wheel_poses(self):
        return [self.getLeftDistance(),
                self.getRightDistance()]

    def convert_rad_to_distance(self, radians):
        return radians * self.WHEEL_DIAMETER / 2.0

    def zero_encoders(self):
        self.left_encoder_offset += self.getLeftDistance()
        self.right_encoder_offset += self.getRightDistance()

    def getLeftDistance(self):
        return self.convert_rad_to_distance(self.left_encoder.getValue()) - self.left_encoder_offset

    def getRightDistance(self):
        return self.convert_rad_to_distance(self.right_encoder.getValue()) - self.right_encoder_offset
