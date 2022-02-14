from Drivetrain import Drivetrain
from Command import Command

class FaceHeadingCommand(Command):
    kP = 0.05
    max_speed_prop = .5
    max_error = 0.075

    def __init__(self, drivetrain: Drivetrain, heading: float):
        self.drivetrain = drivetrain
        self.target_heading = heading

    def update(self, time):
        error = (self.target_heading - self.drivetrain.get_heading())
        rot = Drivetrain.bound(error * self.kP, -self.max_speed_prop, self.max_speed_prop)
        #print("Rot: {}".format(rot))
        self.drivetrain.drive(-rot, rot)
        self.drivetrain.update()

    def is_finished(self):
        return abs(self.target_heading - self.drivetrain.get_heading()) < self.max_error