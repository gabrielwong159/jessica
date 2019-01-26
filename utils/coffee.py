#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from tf.transformations import quaternion_from_euler
from niryo_one_python_api.niryo_one_api import *

print 'Init node'
rospy.init_node('move_group_python', anonymous=True)

print 'Init moveit'
group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)

def move_to_cup(Niryo):
    rospy.sleep(0.1)
    # home()
    zero()
    # grab a cup
    #move_pose((-0.005, 0.24, 0.295), (0, 0, 1.735))
    rospy.sleep(0.2)
    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)
    move_pose((0.014, 0.239, 0.202), (0, 0, 1.735))
    return
    shift_z(-0.15)
    # return cup
    home()
    move_joint(1.176, 0.029, -0.641, 1.345, 1.295, -0.901)
    move_pose((0.297, 0.011, 0.285), (0, 0, 0))
    shift_z(-0.015)
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    zero()
    home()

def move_to_coffee(Niryo):


if __name__ == '__main__':
    # Niryo.calibrate_manual()
    n = NiryoOne()
    n.change_tool(TOOL_GRIPPER_1_ID)
    coffee(n)
