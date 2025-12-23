from robot.robot import dobot
import time

# ===============================
# CONFIG
# ===============================
PORT = "COM6"
LOOP_DELAY = 0.1   # 루프 안정화
RUN_TIME_SEC = 20 * 60  # 20분

HOME_POS = [209.75, 0, 99.96, 0]

PICK_POS = [-17.0, -227.0, 23.0, 0.0]
PICK_WAY = [-17.0, -227.0, 73.0, 0.0]   # Z + 50

PLACE_POS = [217.09, 6.20, -27.86, 0]
PLACE_WAY = [217.09, 6.20, 22.14, 0.0]  # Z + 50

# ===============================
# INIT
# ===============================
robot = dobot(PORT)
robot.connect()
print("[INFO] Dobot connected")

robot.w_home()
print("[INFO] Homing completed")

while True:
    # Home
    robot.moveJ(*HOME_POS)
    while not robot.is_reached(HOME_POS):
        time.sleep(LOOP_DELAY)

    # Pick Way
    robot.moveJ(*PICK_WAY)
    while not robot.is_reached(PICK_WAY):
        time.sleep(LOOP_DELAY)

    # Pick
    robot.moveL(*PICK_POS)
    while not robot.is_reached(PICK_POS):
        time.sleep(LOOP_DELAY)

    # Suction ON
    robot.suction(1)
    time.sleep(1.0)

    # Pick Way
    robot.moveL(*PICK_WAY)
    while not robot.is_reached(PICK_WAY):
        time.sleep(LOOP_DELAY)

    # Home
    robot.moveJ(*HOME_POS)
    while not robot.is_reached(HOME_POS):
        time.sleep(LOOP_DELAY)

    # Place Way
    robot.moveJ(*PLACE_WAY)
    while not robot.is_reached(PLACE_WAY):
        time.sleep(LOOP_DELAY)

    # Place
    robot.moveL(*PLACE_POS)
    while not robot.is_reached(PLACE_POS):
        time.sleep(LOOP_DELAY)

    # Suction OFF
    robot.suction(0)
    time.sleep(1.0)

    # Place Way
    robot.moveL(*PLACE_WAY)
    while not robot.is_reached(PLACE_WAY):
        time.sleep(LOOP_DELAY)
