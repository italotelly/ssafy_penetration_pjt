from robot.robot import dobot

robot = dobot('COM6')
robot.connect()
robot.w_home()

pos = [10, -100, 20, 0]

robot.moveJ(*pos)
while True:
    if robot.is_reached(pos):
        print(robot.get_pose())
        break
# robot.moveJ(*pos)
# robot.w_home()

robot.disconnect()