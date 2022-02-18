from controller import Robot
from Drivetrain import Drivetrain
from FaceHeadingCommand import FaceHeadingCommand
from TurnDegreesCommand import TurnDegressCommand
from DriveForwardCommand import DriveForwardCommand
from TurnAndDriveCommand import TurnAndDriveCommand
from RHExploreCommand import RHExploreCommand
from LHExploreCommand import LHExploreCommand
from SonicSensors import SonicSensors
from Mapper import Mapper
from mazeRead import MazeReader
import json

def main():
    MAX_SPEED = 6.28
    MAX_ACCELERATION = 10
    TIMESTEP = 64

    data = {"run": 1}
    try:
        with open("data.json") as file:
            data = json.load(file)
    except OSError:
        print("File not found, running Run 1")

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
    if 'maze' in data:
        mapper.map = data['maze']

    robot.step(TIMESTEP)
    widePath = [DriveForwardCommand(drivetrain, 2.5, fast=True), FaceHeadingCommand(drivetrain, 90, fast=True),
                DriveForwardCommand(drivetrain, 5, fast=True),
                FaceHeadingCommand(drivetrain, 0, fast=True), DriveForwardCommand(drivetrain, 8, fast=True), FaceHeadingCommand(drivetrain, -90, fast=True),
                DriveForwardCommand(drivetrain, 2, fast=True)]

    shortPath = [DriveForwardCommand(drivetrain, 7.5, fast=True), FaceHeadingCommand(drivetrain, 90, fast=True),
                 DriveForwardCommand(drivetrain, 2, fast=True),
                 FaceHeadingCommand(drivetrain, 0, fast=True), DriveForwardCommand(drivetrain, 3, fast=True),
                 FaceHeadingCommand(drivetrain, 90, fast=True), DriveForwardCommand(drivetrain, 2, fast=True)]

    shortPathTightTurns = [DriveForwardCommand(drivetrain, 7, fast=True), TurnAndDriveCommand(drivetrain, right=False),
                 DriveForwardCommand(drivetrain, 1, fast=True),
                 TurnAndDriveCommand(drivetrain, right=True), DriveForwardCommand(drivetrain, 2, fast=True),
                 TurnAndDriveCommand(drivetrain, right=False), DriveForwardCommand(drivetrain, 2.5, fast=True)]

    testTurns = [TurnDegressCommand(drivetrain, 90), TurnDegressCommand(drivetrain, -90),
                 TurnDegressCommand(drivetrain, 180)]

    testTurns2 = [FaceHeadingCommand(drivetrain, 180)]

    lhExplore = [DriveForwardCommand(drivetrain, 0.5), LHExploreCommand(drivetrain, mapper, sonicSensors)]
    rhExplore = [DriveForwardCommand(drivetrain, 0.5), RHExploreCommand(drivetrain, mapper, sonicSensors)]

    if data['run'] == 1:
        commands = rhExplore
    elif data['run'] == 2:
        commands = lhExplore
    else:
        commands = generatePathFromMaze(data['maze'], drivetrain)

    commands = shortPathTightTurns

    runCommands = True
    index = 0
    max_index = 100000
    commands[index].initialize(robot.getTime())
    while robot.step(TIMESTEP) != -1 and index < len(commands) and index <= max_index:
        if runCommands:
            if not commands[index].initialized:
                commands[index].initialize(robot.getTime())
            if not commands[index].is_finished():
                commands[index].update(robot.getTime())
            else:
                print("Ended Command")
                index += 1
                if index < len(commands):
                    commands[index].initialize(robot.getTime())

            if touchSensor.getValue() > 0:
                mapper.setTrophy(drivetrain.odometry.getPose())
                print("WIN!")
                break
        if mapper.updateGridWalls(drivetrain.odometry.getPose(), drivetrain.get_heading(), sonicSensors.get_grid()):
            mapper.prettyPrintMap()
        #print("Heading {}".format(drivetrain.get_heading()))
        # print([sensor.getValue() for sensor in sonicSensors.sensors])
    if data['run'] != 3:
        data['run'] += 1
        data['maze'] = mapper.map
        with open("data.json", 'w') as file:
            json.dump(data, file)

def generatePathFromMaze(maze, drivetrain):
    mazeReader = MazeReader()
    directions = mazeReader.GiveDirections(maze)
    print(directions)
    headings = {"left": 90, "up": 0, "down": 180, "right": -90}
    commands = [DriveForwardCommand(drivetrain, directions[0][1] + 0.5, fast=True)]
    for i in range(1, len(directions)-1):
        commands.append(FaceHeadingCommand(drivetrain, headings[directions[i][0]], fast=True))
        commands.append(DriveForwardCommand(drivetrain, directions[i][1], fast=True))
    commands.append(FaceHeadingCommand(drivetrain, headings[directions[-1][0]], fast=True))
    commands.append(DriveForwardCommand(drivetrain, directions[i][1]+2, fast=True))
    return commands


if __name__ == "__main__":
    main()