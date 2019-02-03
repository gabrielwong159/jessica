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


def move_joints(x, y, z, roll, pitch, yaw):
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


def change_position(x=None, y=None, z=None):
    # change only (x, y, z) while maintaining tool orientation
    pose = group.get_pose().pose

    if x is not None:
        pose.position.x = x
    if y is not None:
        pose.position.y = y
    if z is not None:
        pose.position.z = z
    
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return pose


def change_orientation(roll, pitch, yaw):
    # change only (roll, pitch, yaw) while maintaining object position
    x, y, z, w = quaternion_from_euler(roll, pitch, yaw)
    
    pose = group.get_pose().pose
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w

    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return pose_goal


def shift_z(dz):
    # vertical movement only, by delta instead
    pose = group.get_current_pose().pose
    pose.position.z += dz

    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


def move_waypoints(waypoints):
    # ref: http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html#cartesian-paths
    # i guess send waypoints as a list of (x, y, z, roll, pitch, yaw) tuples?
    goal = []
    for x, y, z, roll, pitch, yaw in waypoints:
        pose = geometry_msgs.msg.Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        x, y, z, w = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        goal.append(pose)
    
    plan, _ = group.compute_cartesian_path(goal, 0.01, 0.0)
    group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()
    

def home():
    move_joints(0, 0, -1.162, 0.144, 1.157, 0)


def zero():
    move_joints(0, 0, 0, 0, 0, 0)


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