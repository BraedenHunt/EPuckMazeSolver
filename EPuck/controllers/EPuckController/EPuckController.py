from controller import Robot, Motor, TouchSensor
from Drivetrain import Drivetrain
from FaceHeadingCommand import FaceHeadingCommand
from DriveForwardCommand import DriveForwardCommand

MAX_SPEED = 3.14
MAX_ACCELERATION = 6.28

# create the Robot instance.
robot = Robot()
timestep = int(robot.getBasicTimeStep())
# get a handler to the motors and set target position to infinity (speed control)

touchSensor = robot.getDevice('touch sensor')
touchSensor.enable(timestep)

# set up the motor speeds at 10% of the MAX_SPEED.
drivetrain = Drivetrain(robot, MAX_SPEED, MAX_ACCELERATION)
#drivetrain.drive(.05, -0.05)

#currentCommand = FaceHeadingCommand(drivetrain, 90)
currentCommand = DriveForwardCommand(drivetrain, 1)

robot.step(timestep)
currentCommand.initialize(robot.getTime())

while robot.step(timestep) != -1:
    poses = drivetrain.get_wheel_poses()
    #print("Left Encoder: {} Right Encoder: {}".format(poses[0], poses[1]))
    if not currentCommand.is_finished():
        currentCommand.update(robot.getTime())
    else:
        print("Final Distance")
        print(drivetrain.get_wheel_poses()[0])
        break
    '''
    print(drivetrain.get_heading())
    #drivetrain.get_heading()
    if touchSensor.getValue() > 0:
        print("TOUCHED!")
        break'''
