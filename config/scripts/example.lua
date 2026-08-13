local home = {0.0, -0.5, 0.8, 0.0, 0.5, 0.0, 0.0}

robot.SetEnabled()
robot.MoveJ(home, 1.0, 2.0, 10.0)
robot.SetDisabled()
