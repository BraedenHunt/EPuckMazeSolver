import math


class Odometry:

    def __init__(self, initialPose):
        self.pose = initialPose
        self.prevLeftDistance = 0
        self.prevRightDistance = 0

    def update(self, leftDistance, rightDistance, compassHeading):
        deltaLeft = leftDistance - self.prevLeftDistance
        deltaRight = rightDistance - self.prevRightDistance

        self.prevLeftDistance = leftDistance
        self.prevRightDistance = rightDistance

        avgDist = (deltaLeft + deltaRight) / 2.0

        radians = math.radians(compassHeading)

        self.pose[0] += avgDist * (-math.sin(radians))
        self.pose[1] += avgDist * (math.cos(radians))

    def getPose(self):
        return self.pose

