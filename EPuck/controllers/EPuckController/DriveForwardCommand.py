from Drivetrain import Drivetrain
from Command import Command
from TrapezoidalMotionProfile import TrapezoidalMotionProfile

class DriveForwardCommand(Command):
    MAZE_GRID = 0.12 # meters
    kP = 200
    max_speed_prop = 1
    max_error = 0.005

    def __init__(self, drivetrain: Drivetrain, spaces = 1):
        self.drivetrain = drivetrain
        self.spaces = spaces
        self.currentPoses = [0, 0]

    def update(self, time):
        delta_time = time - self.initial_time
        self.currentPoses = self.drivetrain.get_wheel_poses()
        left_error = self.left_trap_profile.calculate(delta_time) - self.drivetrain.getLeftDistance()
        right_error = self.left_trap_profile.calculate(delta_time) - self.drivetrain.getRightDistance()
        #print("Left Pos: {} Right Pos: {}".format(self.drivetrain.getLeftDistance(), self.drivetrain.getRightDistance()))
        #print("Left Error: {} Right Error {}".format(left_error, right_error))
        left_adj = Drivetrain.bound(left_error * self.kP, -self.max_speed_prop, self.max_speed_prop)
        right_adj = Drivetrain.bound(right_error * self.kP, -self.max_speed_prop, self.max_speed_prop)
        self.drivetrain.drive(left_adj, right_adj)
        self.drivetrain.update()

        print("Target = {}".format(self.left_trap_profile.calculate(100)))
        print((self.drivetrain.getLeftDistance() + self.drivetrain.getRightDistance())/ 2.0)

    def initialize(self, time):
        self.initial_time = time
        self.drivetrain.zero_encoders()
        self.set_target_poses()
        self.left_trap_profile = TrapezoidalMotionProfile(0, self.target_pos_left, self.drivetrain.MAX_SPEED, self.drivetrain.MAX_ACCELERATION)
        self.right_trap_profile = TrapezoidalMotionProfile(0, self.target_pos_left, self.drivetrain.MAX_SPEED, self.drivetrain.MAX_ACCELERATION)

    def set_target_poses(self):
        self.currentPoses = self.drivetrain.get_wheel_poses()
        self.target_pos_left = self.MAZE_GRID * self.spaces
        self.target_pos_right = self.MAZE_GRID * self.spaces

    def is_finished(self):
        left_error = self.target_pos_left - self.currentPoses[0]
        right_error = self.target_pos_right - self.currentPoses[1]
        return abs(left_error) < self.max_error and abs(right_error) < self.max_error