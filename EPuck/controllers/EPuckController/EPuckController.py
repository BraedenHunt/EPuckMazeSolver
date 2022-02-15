from controller import Robot, Motor, TouchSensor, DistanceSensor
from Drivetrain import Drivetrain
from FaceHeadingCommand import FaceHeadingCommand
from TurnDegreesCommand import TurnDegressCommand
from DriveForwardCommand import DriveForwardCommand
from SonicSensors import SonicSensors
from Mapper import Mapper

MAX_SPEED = 6.28
MAX_ACCELERATION = 200
TIMESTEP = 64

# create the Robot instance.
robot = Robot()

# get a handler to the motors and set target position to infinity (speed control)

touchSensor = robot.getDevice('touch sensor')
touchSensor.enable(TIMESTEP)

# set up the motor speeds at 10% of the MAX_SPEED.
drivetrain = Drivetrain(robot, MAX_SPEED, MAX_ACCELERATION)
sonicSensors = SonicSensors(robot)
drivetrain.drive(0.0, 0.0)

mapper = Mapper(12, 12, .12)

robot.step(TIMESTEP)
widePath = [DriveForwardCommand(drivetrain, 2.5), FaceHeadingCommand(drivetrain, 90), DriveForwardCommand(drivetrain, 5),
            FaceHeadingCommand(drivetrain, 0), DriveForwardCommand(drivetrain, 8), FaceHeadingCommand(drivetrain, -90),
            DriveForwardCommand(drivetrain, 2)]

shortPath = [DriveForwardCommand(drivetrain, 7.5), TurnDegressCommand(drivetrain, 90), DriveForwardCommand(drivetrain, 2),
            TurnDegressCommand(drivetrain, -90), DriveForwardCommand(drivetrain, 3), TurnDegressCommand(drivetrain, 90), DriveForwardCommand(drivetrain, 2)]

testTurns = [TurnDegressCommand(drivetrain, 90), TurnDegressCommand(drivetrain, -90), TurnDegressCommand(drivetrain, 180)]

testTurns2 = [FaceHeadingCommand(drivetrain, 180)]
commands = shortPath
runCommands = True
index = 0
max_index = 10
commands[index].initialize(robot.getTime())
while robot.step(TIMESTEP) != -1 and index < len(commands) and index <= max_index:
    if runCommands:
        if not commands[index].is_finished():
            commands[index].update(robot.getTime())
        else:
            print("Ended Command")
            index += 1
            if index < len(commands):
                commands[index].initialize(robot.getTime())

        if touchSensor.getValue() > 0:
            print("TOUCHED!")
            break
        #print('Heading: {}'.format(drivetrain.get_heading()))
        print("right sensor: {}".format(sonicSensors.sensors[2].getValue()))
        if type(commands[index]) == DriveForwardCommand:
            if mapper.updateGridWalls(drivetrain.odometry.getPose(), drivetrain.get_heading(), sonicSensors.get_grid()):
                print("  ", end='')
                for i in range(len(mapper.map)):
                    print("{} ".format(str(i).zfill(2)), end='')
                print()
                for row in range(len(mapper.map)):
                    print(str(row).zfill(2), end='')
                    for point in range(len(mapper.map[row])):
                        print(" {} ".format(mapper.map[row][point]), end="")
                    print()
                print('-------------------------------')
    #sonicSensors.printSensorValues()


