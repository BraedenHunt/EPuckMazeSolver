from TrapezoidalMotionProfile import TrapezoidalMotionProfile

t = 0
max_time = 20
time_step = 1/10

profile = TrapezoidalMotionProfile(90, 0, 10, 10)

while t <= max_time:
    print("t: {} Goal: {}".format(t, profile.calculate(t)))
    profile.calculate(t)
    t += time_step