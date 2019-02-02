#!/usr/bin/env python

# Wait for capsule
# Move capsule into coffee machine
# Move to cup prime
# Dispense cup
# Move cup to coffee machine
# Activate electromagnet
# Move cup to serving platform
# Drop cup

import time
import sys

from niryo_one_python_api.niryo_one_api import *
from activate_coffee_machine import start_coffee
# from capsule_process import *
# from cup_movements import *

import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from tf.transformations import quaternion_from_euler

print 'Init node'
rospy.init_node('move_group_python', anonymous=True, disable_signals=True)

print 'Init moveit'
group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)

n = NiryoOne()
SENSOR = GPIO_1A
MOTOR = GPIO_2A
ELECTROMAGNET = SW_2

# MAIN FUNCTION
def coffee_cycle(niryo_one):
    capsule_cycle(niryo_one)
    move_to_cup(niryo_one)
    dispense_cup(niryo_one)
    move_cup_to_machine(niryo_one)
    # prepare_coffee(niryo_one)
    deliver_coffee(niryo_one)


# MOVEMENT FUNCTIONS
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
    goal = [group.get_current_pose().pose]
    for x, y, z, roll, pitch, yaw in waypoints:
        pose = group.get_current_pose().pose

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
    # group.stop()
    # group.clear_pose_targets()


def start():
    move_joints(0, 0.428, -1.237, 0.012, 0.814, -0.046)

def home():
    move_joints(0, 0, -1.162, 0.144, 1.157, 0)

def zero():
    move_pose((0.238, 0.000, 0.417), (0.041, 0.014, 0.000))

def check_joints(Niryo, joints, tolerance=0.05):
    actual_joints = Niryo.get_joints()
    for i in range(6):
        if abs(joints[i] - actual_joints[i]) > tolerance:
            print('Joint {:d} failed: ({:.3f}, {:.3f})'.format(i, joints[i], actual_joints[i]))
            return False
    return True


def move_to_cup(Niryo):
    rospy.sleep(0.2)
    # move_pose((0.014, 0.239, 0.202), (-0.049, -0.117, 1.735))
    move_joints(1.490, -0.368, -0.659, -0.252, 1.146, 0.056)
    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)


def dispense_cup(niryo_one):
    niryo_one.pin_mode(MOTOR,0)
    niryo_one.digital_write(MOTOR, 1)
    time.sleep(0.1)
    niryo_one.digital_write(MOTOR, 0)
    time.sleep(0.1)


def move_cup_to_machine(niryo_one):
    rospy.sleep(0.2)
    home()
    move_pose((0.054, -0.196, 0.077), (-0.048, 0.027, -2.101))


def deliver_coffee(Niryo):
    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)
    move_pose((0.130, -0.127, 0.074), (-0.087, 0.086, -1.362))
    move_joints(-0.685, -0.080, -0.466, 0.637, 0.577, -0.633)

    move_joints(0.049, -0.275, -0.346, -0.015, 0.630, 0.000)

    move_joints(0.060, -0.445, -0.298, 0.103, 0.733, -0.121)
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    start()


def prepare_coffee(Niryo):
    n.digital_write(ELECTROMAGNET, True)
    time.sleep(45)
    n.digital_write(ELECTROMAGNET, False)


def capsule_cycle(Niryo):
    Niryo.change_tool(TOOL_GRIPPER_1_ID)
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    # start of sequence of serve capsule (start from zero())
    # move_joints(-0.319, -0.720, 0.005, -0.353, 0.679, 0.309)
    move_joints(-0.246, -0.711, 0.013, 0.260, 0.645, -0.177)

    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)
    shift_z(0.1)
    move_joints(-1.516, 0.102, -0.265, -0.025, -1.069, -0.020)
    move_joints(-1.536, -0.843, 0.881, 0.005, -1.460, -0.015)
    
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    move_joints(-1.555, -0.630, 0.952, 0.020, -1.476, -0.015)


if __name__ == "__main__":
    arg = sys.argv[1]
    SENSOR = GPIO_1A
    NIRYO = NiryoOne()
    NIRYO.pin_mode(SENSOR, 1)
    NIRYO.change_tool(TOOL_GRIPPER_1_ID)
    if arg == "main":
        while True:
            if not check_joints(NIRYO, [0, 0.428, -1.237, 0.012, 0.814, -0.046]):
                start()
            # Takes 2 seconds of the sensor being activated
            # Checks 20 times over the next 2 seconds
            trigger_time = 2
            count = 0
            check_ticks = 20
    
            # Sensor output is inverted so we have to NOT all of them(?)
            trigger = not NIRYO.digital_read(SENSOR)
            # print(trigger)
            if trigger:
                while count < trigger_time:
                    trigger = not NIRYO.digital_read(SENSOR)
                    # print(trigger)
                    # print(count, trigger_time)
                    time.sleep(trigger_time/float(check_ticks))
                    count += trigger_time/float(check_ticks)
                    if NIRYO.digital_read(SENSOR):
                        trigger = False
                        break
            # print("FINAL TRIGGER: ", trigger)
            if trigger:
                start()
                coffee_cycle(NIRYO)
    elif arg=="capsule":
        start()
        capsule_cycle(NIRYO)
    elif arg=="coffee":
        prepare_coffee(NIRYO)
    elif arg=="deliver":
        home()
        move_pose((0.064, -0.196, 0.077), (-0.048, 0.027, -2.101))
        deliver_coffee(NIRYO)

