from Drivetrain import Drivetrain
from Command import Command

class FaceHeadingCommand(Command):
    kP = 0.1
    max_speed_prop = .50
    max_error = 0.005

    def __init__(self, drivetrain: Drivetrain, heading: float, fast=False):
        self.drivetrain = drivetrain
        if heading > 180:
            heading = heading % -180
        elif heading < -180:
            heading = heading % 180
        self.target_heading = heading
        #print("turning to {}".format(heading))
        if fast:
            self.kP = 0.1
            self.max_error = .15

    def update(self, time):
        error = (self.target_heading - self.drivetrain.get_heading())
        if error < -180:
            error = error % 360
        if error > 180:
            error = error % -360
        #print("Heading: {} Target: {} Error: {}".format(self.drivetrain.get_heading(), self.target_heading, error))
        rot = Drivetrain.bound(error * self.kP, -self.max_speed_prop, self.max_speed_prop)
        #print("Rot: {}".format(rot))
        self.drivetrain.drive(-rot, rot)
        self.drivetrain.update()

    def is_finished(self):
        return abs(self.target_heading - self.drivetrain.get_heading()) < self.max_error