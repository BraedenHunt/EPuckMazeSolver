from controller import Robot, Motor, TouchSensor
from Drivetrain import Drivetrain
from FaceHeadingCommand import FaceHeadingCommand
from DriveForwardCommand import DriveForwardCommand

MAX_SPEED = 6.28
MAX_ACCELERATION = 4
TIMESTEP = 64

# create the Robot instance.
robot = Robot()

# get a handler to the motors and set target position to infinity (speed control)

touchSensor = robot.getDevice('touch sensor')
touchSensor.enable(TIMESTEP)

# set up the motor speeds at 10% of the MAX_SPEED.
drivetrain = Drivetrain(robot, MAX_SPEED, MAX_ACCELERATION)
#drivetrain.drive(.05, -0.05)

#currentCommand = FaceHeadingCommand(drivetrain, 90)
currentCommand = DriveForwardCommand(drivetrain, 6)

robot.step(TIMESTEP)
commands = [DriveForwardCommand(drivetrain, 2.5), FaceHeadingCommand(drivetrain, 90), DriveForwardCommand(drivetrain, 5),
            FaceHeadingCommand(drivetrain, 0), DriveForwardCommand(drivetrain, 8), FaceHeadingCommand(drivetrain, -90),
            DriveForwardCommand(drivetrain, 2)]
index = 0
max_index = 10
commands[index].initialize(robot.getTime())
while robot.step(TIMESTEP) != -1 and index < len(commands) and index <= max_index:
    '''
    if not commands[index].is_finished():
        commands[index].update(robot.getTime())
    else:
        print("Ended Command")
        index += 1
        commands[index].initialize(robot.getTime())

    if touchSensor.getValue() > 0:
        print("TOUCHED!")
        break
    '''

