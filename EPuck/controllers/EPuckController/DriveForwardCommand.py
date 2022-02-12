from Drivetrain import Drivetrain
from Command import Command
from TrapezoidalMotionProfile import TrapezoidalMotionProfile

class DriveForwardCommand(Command):
    MAZE_GRID = 0.12 # meters
    kP = 20
    max_speed_prop = 1
    max_error = 0.01

    def __init__(self, drivetrain: Drivetrain, spaces: int = 1):
        self.drivetrain = drivetrain
        self.spaces = spaces

    def update(self, time):
        delta_time = time - self.initial_time
        #self.update_target_poses()
        self.currentPoses = self.drivetrain.get_wheel_poses()
        left_error = self.left_trap_profile.calculate(delta_time) - self.currentPoses[0]
        print("Left Error: {}".format(left_error))
        right_error = self.left_trap_profile.calculate(delta_time) - self.currentPoses[1]
        print("Right Error: {}".format(right_error))

        left_adj = Drivetrain.bound(left_error * self.kP, -self.max_speed_prop, self.max_speed_prop)
        right_adj = Drivetrain.bound(right_error * self.kP, -self.max_speed_prop, self.max_speed_prop)
        #print("Adj: {}, {}".format(left_adj, right_adj))
        self.drivetrain.drive(left_adj, right_adj)

    def initialize(self, time):
        self.initial_time = time
        self.set_target_poses()
        self.left_trap_profile = TrapezoidalMotionProfile(self.currentPoses[0], self.target_pos_left, self.drivetrain.MAX_SPEED, self.drivetrain.MAX_ACCELERATION)
        self.right_trap_profile = TrapezoidalMotionProfile(self.currentPoses[1], self.target_pos_left, self.drivetrain.MAX_SPEED, self.drivetrain.MAX_ACCELERATION)

    def set_target_poses(self):
        self.currentPoses = self.drivetrain.get_wheel_poses()
        self.target_pos_left = self.MAZE_GRID * self.spaces + self.currentPoses[0]
        self.target_pos_right = self.MAZE_GRID * self.spaces + self.currentPoses[1]

    def is_finished(self):
        left_error = self.target_pos_left - self.currentPoses[0]
        right_error = self.target_pos_right - self.currentPoses[1]
        return abs(left_error) < self.max_error and abs(right_error) < self.max_error