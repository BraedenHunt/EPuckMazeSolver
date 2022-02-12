from controller import Robot, Motor, TouchSensor
from Drivetrain import Drivetrain


MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()
timestep = int(robot.getBasicTimeStep())
# get a handler to the motors and set target position to infinity (speed control)

touchSensor = robot.getDevice('touch sensor')
touchSensor.enable(timestep)

# set up the motor speeds at 10% of the MAX_SPEED.
drivetrain = Drivetrain(robot, MAX_SPEED)
drivetrain.drive(.05, -0.05)

while robot.step(timestep) != -1:

    print(drivetrain.get_heading())
    #drivetrain.get_heading()
    if touchSensor.getValue() > 0:
        print("TOUCHED!")
        break
