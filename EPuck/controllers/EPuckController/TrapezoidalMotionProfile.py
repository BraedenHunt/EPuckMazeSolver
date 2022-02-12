class TrapezoidalMotionProfile:

    def __init__(self, initialState, goalState, maxVelocity, maxAcceleration):
        self.initialState = initialState
        self.goalState = goalState
        self.maxVelocity = maxVelocity
        self.maxAcceleration = maxAcceleration

        self.dX = self.goalState - self.initialState

        self.endAccelTime = self.maxVelocity / self.maxAcceleration
        self.endAccelDist = self.maxVelocity / 2 * self.endAccelTime

        self.fullSpeedDist = self.dX - (self.endAccelDist * 2)

        if self.fullSpeedDist < 0:
            # TODO
            pass
        self.endFullSpeedTime = self.fullSpeedDist / self.maxVelocity + self.endAccelTime

        self.totalTime = self.endFullSpeedTime + self.endAccelTime

    def calculate(self, time):
        if time < self.endAccelTime:  # speeding up
            return self.initialState + self.maxAcceleration * time ** 2 / 2.0

        elif time < self.endFullSpeedTime:  # at max speed
            return self.initialState + self.endAccelDist + self.maxVelocity * (time - self.endAccelTime)

        elif time < self.totalTime:  # decelerating
            decel_time = time - self.endFullSpeedTime
            return self.initialState + self.endAccelDist + self.fullSpeedDist + (
                        self.maxVelocity * decel_time - 0.5 * self.maxAcceleration * decel_time ** 2)
        else:  # over time limit so should be at target state
            return self.goalState
