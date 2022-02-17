import math
from math import atan2

from controller import Robot, Motor, Compass, PositionSensor
from Odometry import Odometry

class Drivetrain:
    WHEEL_DIAMETER = 0.041 * 0.97947900595 # meters (~41mm)

    def __init__(self, robot: Robot, max_speed, max_accel, timestep = 64):
        self.leftPower = 0
        self.rightPower = 0
        self.robot = robot
        self.timestep = timestep
        self.MAX_SPEED = max_speed
        self.MAX_ACCELERATION = max_accel
        self.right_motor: Motor = robot.getDevice('right wheel motor')
        self.left_motor: Motor = robot.getDevice('left wheel motor')

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        self.left_encoder: PositionSensor = robot.getDevice('left wheel sensor')
        self.right_encoder: PositionSensor = robot.getDevice('right wheel sensor')

        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)

        self.left_encoder_offset = 0
        self.right_encoder_offset = 0

        self.odometry = Odometry([-.06, -0.6])

        self.compass: Compass = robot.getDevice('compass')
        self.compass.enable(self.timestep)

        self.gryoOffset = 0

    def drive(self, left: float, right: float):
        self.leftPower = left
        self.rightPower = right

    def update(self):
        self.left_motor.setVelocity(self.bound(self.leftPower, -1, 1) * self.MAX_SPEED)
        self.right_motor.setVelocity(self.bound(self.rightPower, -1, 1) * self.MAX_SPEED)
        self.odometry.update(self.getLeftDistance(), self.getRightDistance(), self.get_heading())

        #print(self.odometry.getPose())

    def get_raw_heading(self):
        values = self.compass.getValues()
        #print(values)
        rad = atan2(values[0], values[1])
        bearing = ((rad - 1.5708) / math.pi * 180.0)
        return bearing

    def get_heading(self):
        bearing = self.get_raw_heading() % 360
        if bearing > 180:
            bearing = bearing - 360.0
        return bearing

    def getGryoAngle(self):
        bearing = self.get_raw_heading() - self.gryoOffset
        return bearing

    def resetGyro(self):
        self.gryoOffset += self.getGryoAngle()

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
        self.odometry = Odometry(self.odometry.getPose())

    def getLeftDistance(self):
        return self.convert_rad_to_distance(self.left_encoder.getValue()) - self.left_encoder_offset

    def getRightDistance(self):
        return self.convert_rad_to_distance(self.right_encoder.getValue()) - self.right_encoder_offset
