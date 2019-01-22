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

n = NiryoOne()
SENSOR = GPIO_1A
LED = GPIO_2A


def move_joint(x, y, z, roll, pitch, yaw):
    joint_goal = [x, y, z, roll, pitch, yaw]
    group.go(joint_goal, wait=True)
    group.stop()
    return joint_goal


def move_pose(position, orientation):
    """ 
    position: (x, y, z)
    orientation: (x, y, z)
    """
    pose_goal = geometry_msgs.msg.Pose()

    x, y, z = position
    pose_goal.position.x = x 
    pose_goal.position.y = y 
    pose_goal.position.z = z 

    r, p, y = orientation
    x, y, z, w = quaternion_from_euler(r, p, y)
    pose_goal.orientation.x = x 
    pose_goal.orientation.y = y 
    pose_goal.orientation.z = z 
    pose_goal.orientation.w = w 

    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return pose_goal


def shift_z(dz):
    pose = group.get_current_pose().pose
    pose.position.z += dz

    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


def home():
    move_joint(0, 0, -1.162, 0.144, 1.157, 0)


def zero():
    move_joint(0, 0, 0, 0, 0, 0)


def coffee():
    rospy.sleep(1)
    # home()
    zero()
    # grab a cup
    move_pose((-0.005, 0.24, 0.295), (0, 0, 1.735))
    rospy.sleep(0.5)
    n.close_gripper(TOOL_GRIPPER_1_ID, 1000)
    move_pose((0.036, 0.217, 0.192), (0, 0, 1.735))
    return
    shift_z(-0.15)
    # return cup
    home()
    move_joint(1.176, 0.029, -0.641, 1.345, 1.295, -0.901)
    move_pose((0.297, 0.011, 0.285), (0, 0, 0))
    shift_z(-0.015)
    n.open_gripper(TOOL_GRIPPER_1_ID, 1000)

    zero()
    home()


if __name__ == '__main__':
    # n.calibrate_manual()
    n.change_tool(TOOL_GRIPPER_1_ID)
    coffee()
