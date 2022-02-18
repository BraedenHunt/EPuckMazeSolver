import math

from Drivetrain import Drivetrain
from Command import Command
from TrapezoidalMotionProfile import TrapezoidalMotionProfile

class TurnAndDriveCommand(Command):
    MAZE_GRID = 0.12 # meters

    kP = 45
    max_speed_prop = .83
    max_error = 0.015

    def __init__(self, drivetrain: Drivetrain, right = True):
        self.drivetrain = drivetrain
        self.currentPoses = [0, 0]
        self.right = right

    def update(self, time):
        delta_time = time - self.initial_time
        self.currentPoses = self.drivetrain.get_wheel_poses()
        left_error = self.left_trap_profile.calculate(delta_time) - self.drivetrain.getLeftDistance()
        right_error = self.right_trap_profile.calculate(delta_time) - self.drivetrain.getRightDistance()
        print("Left Pos: {} Right Pos: {}".format(self.drivetrain.getLeftDistance(), self.drivetrain.getRightDistance()))
        print("Left Error: {} Right Error {}".format(left_error, right_error))
        left_adj = Drivetrain.bound(left_error * self.kP, -self.max_speed_prop * self.leftMaxSpeed / self.drivetrain.MAX_SPEED, self.max_speed_prop * self.leftMaxSpeed / self.drivetrain.MAX_SPEED)
        right_adj = Drivetrain.bound(right_error * self.kP, -self.max_speed_prop * self.rightMaxSpeed / self.drivetrain.MAX_SPEED, self.max_speed_prop  * self.rightMaxSpeed / self.drivetrain.MAX_SPEED)
        self.drivetrain.drive(left_adj, right_adj)
        self.drivetrain.update()

        #print("Target = {}".format(self.left_trap_profile.calculate(100)))
        #print((self.drivetrain.getLeftDistance() + self.drivetrain.getRightDistance())/ 2.0)

    def initialize(self, time):
        self.initial_time = time
        self.initialized = True
        self.drivetrain.zero_encoders()
        self.set_target_poses()
        self.left_trap_profile = TrapezoidalMotionProfile(0, self.target_pos_left, self.leftMaxSpeed, self.leftMaxAccel)
        self.right_trap_profile = TrapezoidalMotionProfile(0, self.target_pos_right, self.rightMaxSpeed, self.rightMaxAccel)

    def set_target_poses(self):
        self.currentPoses = self.drivetrain.get_wheel_poses()
        d1 = (self.MAZE_GRID - self.drivetrain.TRACK_WIDTH) * math.pi / 4.0
        d2 = (self.MAZE_GRID + self.drivetrain.TRACK_WIDTH) * math.pi / 4.0
        if self.right:
            self.target_pos_left = d2
            self.target_pos_right = d1

            self.leftMaxSpeed = self.drivetrain.MAX_SPEED * self.max_speed_prop
            self.rightMaxSpeed = d1 / d2 * self.drivetrain.MAX_SPEED * self.max_speed_prop

            self.leftMaxAccel = self.drivetrain.MAX_ACCELERATION
            self.rightMaxAccel = d1/d2 * self.drivetrain.MAX_ACCELERATION
        else:
            self.target_pos_left = d1
            self.target_pos_right = d2

            self.leftMaxSpeed = d1/d2 * self.drivetrain.MAX_SPEED * self.max_speed_prop
            self.rightMaxSpeed = self.drivetrain.MAX_SPEED * self.max_speed_prop

            self.leftMaxAccel = d1/d2 * self.drivetrain.MAX_ACCELERATION
            self.rightMaxAccel = self.drivetrain.MAX_ACCELERATION

    def is_finished(self):
        left_error = self.target_pos_left - self.currentPoses[0]
        right_error = self.target_pos_right - self.currentPoses[1]
        return abs(left_error) < self.max_error and abs(right_error) < self.max_error