from Command import Command
from Drivetrain import Drivetrain
from Mapper import Mapper
from SonicSensors import SonicSensors
from queue import Queue
from Command import Command
from DriveForwardCommand import DriveForwardCommand
from TurnDegreesCommand import TurnDegressCommand
from FaceHeadingCommand import FaceHeadingCommand


class LHExploreCommand(Command):

    def __init__(self, drivetrain: Drivetrain, mapper: Mapper, sonicSensors: SonicSensors):
        self.drivetrain = drivetrain
        self.mapper = mapper
        self.sonicSensors = sonicSensors

        self.initialTime = 0
        self.commandQueue = Queue()
        self.currentCommand: Command = None

    def initialize(self, time):
        self.initialized = True
        self.initialTime = time

    def update(self, time):
        delta_time = time - self.initialTime
        #self.mapper.updateGridWalls(self.drivetrain.odometry.getPose(), self.drivetrain.get_heading(), self.sonicSensors.get_grid())
        if self.currentCommand is None:  # If there isn't a current command
            if not self.commandQueue.empty():  # If the queue isn't empty, the next command is the next queue command
                self.currentCommand = self.commandQueue.get()
                self.currentCommand.initialize(time)
            else:  # If there isn't a current command, and the queue is empty, then determine next step
                robotPose = self.drivetrain.odometry.getPose()
                gridPose = self.mapper.get_grid_pos(robotPose)
                adjCoords = self.mapper.getRobotRelativeAdjPoses(self.drivetrain.get_heading(), gridPose)
                rightSpace = self.mapper.getPosFromPose(adjCoords[1])
                forwardSpace = self.mapper.getPosFromPose(adjCoords[0])
                leftSpace = self.mapper.getPosFromPose(adjCoords[3])

                if leftSpace == '?':
                    print("Turning left: {} heading".format(90 * round((self.drivetrain.get_heading() + 90) / 90)))
                    # turn left to the next cardinal direction.
                    self.currentCommand = FaceHeadingCommand(self.drivetrain,
                                                             90 * round((self.drivetrain.get_heading() + 90) / 90))
                else:
                    if forwardSpace == '#':
                        if leftSpace == '+':
                            # turn left to the next cardinal direction.
                            self.currentCommand = FaceHeadingCommand(self.drivetrain,
                                                                     90 * round(
                                                                         (self.drivetrain.get_heading() + 90) / 90))
                        else:
                            print("Turning right")
                            self.currentCommand = FaceHeadingCommand(self.drivetrain,
                                                                     90 * round((self.drivetrain.get_heading() - 90) / 90))
                    else:
                        self.currentCommand = DriveForwardCommand(self.drivetrain, 1)
                        print("Driving Forward")

        if not self.currentCommand.initialized:
            self.currentCommand.initialize(time)
        self.currentCommand.update(time)
        if self.currentCommand.is_finished():
            self.currentCommand.cancel()
            self.currentCommand = None if self.commandQueue.empty() else self.commandQueue.get()  # If there is another command on the queue, use that, otherwise None

    def cancel(self):
        if not self.currentCommand is None:
            self.currentCommand.cancel()

    def is_finished(self):
        return False  # always keep exploring
