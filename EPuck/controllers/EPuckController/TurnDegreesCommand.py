from Drivetrain import Drivetrain
from Command import Command
from TrapezoidalMotionProfile import TrapezoidalMotionProfile

class TurnDegressCommand(Command):
    kP = 0.12
    kD = 0.06
    max_speed_prop = 1
    max_error = 0.075
    maxOmega = 120
    maxAlpha = 450

    def __init__(self, drivetrain: Drivetrain, angle: float):
        self.drivetrain = drivetrain
        self.target_heading = 0
        self.angleOffset = -angle
        self.initialTime = 0
        self.error = 0
        self.prevError = 0

    def update(self, time):
        delta_time = time - self.initialTime
        self.error = (self.profile.calculate(delta_time) - (self.getCurrentAngle()))
        rot = Drivetrain.bound(self.error * self.kP + self.kD * (self.prevError - self.error), -self.max_speed_prop, self.max_speed_prop)
        self.prevError = self.error
        #print("Current Angle: {}".format(self.getCurrentAngle()))
        #print("Target Angle: {}".format(self.profile.calculate(delta_time)))
        self.drivetrain.drive(-rot, rot)
        self.drivetrain.update()
        #print("error: {}".format(self.error))

    def initialize(self, time):
        self.initialTime = time
        self.drivetrain.resetGyro()
        self.profile = TrapezoidalMotionProfile(self.getCurrentAngle(), self.target_heading, self.maxOmega, self.maxAlpha)

    def getCurrentAngle(self):
        bearing = (self.drivetrain.getGryoAngle() + self.angleOffset)
        if bearing > 180:
            bearing = bearing - 360.0
        elif bearing < -180:
            bearing += 360.0
        return bearing

    def is_finished(self):
        return abs(self.target_heading - self.getCurrentAngle()) < self.max_error