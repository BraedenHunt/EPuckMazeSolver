from TrapezoidalMotionProfile import TrapezoidalMotionProfile

t = 0
max_time = 10
time_step = 1/64.0

profile = TrapezoidalMotionProfile(0, 5, 1, 1)

while t <= max_time:
    #print("t: {} Goal: {}".format(t, profile.calculate(t)))
    profile.calculate(t)
    t += time_step