from niryo_one_python_api.niryo_one_api import *
from utils.movement import *


def load_cup():
    # grab cup, place under machine
    pass


def load_capsule():
    # grab capsule, drop into machine
    pass


def serve_cup():
    # grab cup from machine, place on pedestal
    pass


def main():
    zero()
    home()
    move_waypoints([
        (0.226, 0.159, 0.268, 0, 0, 0),
        (0.138, 0.229, 0.243, 0, 0, 0),
        (0.079, 0.242, 0.202, 0, 0, 0),
        (-0.02, 0.201, 0.131, 0, 0, 0),
    ])

def capsule_cycle(Niryo):
    Niryo.change_tool(TOOL_GRIPPER_1_ID)
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    # start of sequence of serve capsule (start from zero())
    move_joints(-0.362, -0.742, 0.056, -0.590, 0.754, 0.450)
    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)
    shift_z(0.1)
    move_joints(-0.346, -0.476, 0.212, -0.158, 0.144, 0.213)
    move_joints(-0.941, -0.211, 0.387, -0.233, -1.201, 0.081)
    move_joints(-1.562, -0.808, 0.907, 0.028, -1.554, -0.035)
    move_joints(-1.555, -0.830, 0.952, 0.020, -1.476, -0.015)
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    rospy.sleep(1)

if __name__ == '__main__':
    # n.calibrate_manual()
    n.change_tool(TOOL_GRIPPER_1_ID)
    n.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    # start of sequence of serve capsule (start from zero())
    move_joints(-0.362, -0.742, 0.056, -0.590, 0.754, 0.450)
    n.close_gripper(TOOL_GRIPPER_1_ID, 1000)
    shift_z(0.1)
    move_joints(-0.346, -0.476, 0.212, -0.158, 0.144, 0.213)
    move_joints(-0.941, -0.211, 0.387, -0.233, -1.201, 0.081)
    move_joints(-1.562, -0.808, 0.907, 0.028, -1.554, -0.035)
    move_joints(-1.555, -0.830, 0.952, 0.020, -1.476, -0.015)
    n.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    rospy.sleep(1)
