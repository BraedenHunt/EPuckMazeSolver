class TrapezoidalMotionProfile:

    def __init__(self, initialState, goalState, maxVelocity, maxAcceleration):
        self.initialState = initialState
        self.goalState = goalState
        self.maxVelocity = maxVelocity
        self.maxAcceleration = maxAcceleration

        self.dX = self.goalState - self.initialState
        self.sign = abs(self.dX) / self.dX

        self.endAccelTime = self.maxVelocity / self.maxAcceleration
        self.endAccelDist = self.sign * self.maxVelocity / 2 * self.endAccelTime

        self.fullSpeedDist = self.dX - (self.endAccelDist * 2)

        if self.sign * self.fullSpeedDist < 0:
            self.endAccelTime = (abs(self.dX) / self.maxAcceleration)**0.5
            self.endAccelDist = self.sign * 0.5 * self.maxAcceleration * self.endAccelTime**2
            self.endFullSpeedTime = self.endAccelTime
            self.fullSpeedDist = 0
            self.maxVelocity = self.maxAcceleration * self.endAccelTime
        else:
            self.endFullSpeedTime = abs(self.fullSpeedDist) / self.maxVelocity + self.endAccelTime

        self.totalTime = self.endFullSpeedTime + self.endAccelTime

    def calculate(self, time):
        if time < self.endAccelTime:  # speeding up
            #print('{}: speeding up'.format(time))
            return self.initialState + self.sign * self.maxAcceleration * time ** 2 / 2.0

        elif time < self.endFullSpeedTime:  # at max speed
            #print('{}: full speed'.format(time))
            return self.initialState + self.endAccelDist + self.sign * self.maxVelocity * (time - self.endAccelTime)

        elif time < self.totalTime:  # decelerating
            decel_time = time - self.endFullSpeedTime
            #print('{}: slowing down'.format(time))
            return self.initialState + self.endAccelDist + self.fullSpeedDist + self.sign * (
                        self.maxVelocity * decel_time - 0.5 * self.maxAcceleration * decel_time ** 2)
        else:  # over time limit so should be at target state
            #print('{}: over time'.format(time))
            return self.goalState
