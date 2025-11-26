import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import file_A

WORK_ROUTE = {
    "work_target": [1.51, -0.298, -0.00143, 1.0],
    "work_init": [-1.82, -2.15, 0.00247, 1.0],
}

def go_to(navigator, pose):
    navigator.goToPose(pose)
    while not navigator.isTaskComplete():
        pass

def make_pose(x, y, w, navigator):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = w
    return goal

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    print("Nav2 is Ready!")

    while True:
        time.sleep(1)
        file_A.count += 1

        if file_A.count >= 5:
            print("Box is full. Dropping mini boxes.")

            x, y, _, w = WORK_ROUTE["work_target"]
            target_pose = make_pose(x, y, w, navigator)
            go_to(navigator, target_pose)

            print("Arrived. Preparing to unload all mini boxes.")
            time.sleep(30)

            print("Resuming mission. Heading back to collect boxes.")

            x, y, _, w = WORK_ROUTE["work_init"]
            init_pose = make_pose(x, y, w, navigator)
            go_to(navigator, init_pose)

            print("Waiting until box count reaches 5.")
            file_A.count = 0

        else:
            continue

    exit(0)

if __name__ == '__main__':
    main()
