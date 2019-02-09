#!/usr/bin/env python

# Wait for capsule
# Move capsule into coffee machine
# Move to cup prime
# Dispense cup
# Move cup to coffee machine
# Activate electromagnet
# Move cup to serving platform
# Drop cup
from __future__ import print_function
import time
import sys
import datetime

import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from tf.transformations import quaternion_from_euler

import running
from niryo_one_python_api.niryo_one_api import *

group_name='arm'
group = moveit_commander.MoveGroupCommander(group_name)
SENSOR = GPIO_1A
MOTOR = GPIO_2A
ELECTROMAGNET = SW_1

CROUCHINGTIGER=[0, 0.428, -1.237, 0.012, 0.814, -0.046]
NIRYO = NiryoOne()
NIRYO.pin_mode(SENSOR, 1)
NIRYO.pin_mode(MOTOR,0)
NIRYO.digital_write(MOTOR, 1)
NIRYO.change_tool(TOOL_GRIPPER_1_ID)

# MAIN FUNCTION
def coffee_cycle(niryo_one):
    start()
    capsule_cycle(niryo_one)
    move_to_cup(niryo_one)
    dispense_cup(niryo_one)
    move_cup_to_machine(niryo_one)
    prepare_coffee(niryo_one)
    deliver_coffee(niryo_one)


# MOVEMENT FUNCTIONS
def move_joints(joint_goal):
    global group
    group.go(joint_goal, wait=True)

    while not check_joints(joint_goal):
#        move_pose((0.238, 0, 0.418), (0, 0, 0))
        try:
            start(niryo_api = True)
        except NiryoOneException as e:
            try:
                start(niryo_api = False)
            except NiryoOneException as e:
                try:
                    home(niryo_api = True)
                except NiryoOneException as e:
                    home(niryo_api = False)


	print("new move joint done")
        group.go(joint_goal, wait=True)
    return joint_goal


def move_pose(position, orientation):
    """
    position: (x, y, z)
    orientation: (x, y, z)
    """
    global group
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
    global group
    pose = group.get_current_pose().pose
    pose.position.z += dz

    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return pose


def move_waypoints(waypoints):
    global group
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
    group.stop()
    group.clear_pose_targets()
    return goal


def home(niryo_api=False):
    if niryo_api:
        NIRYO.move_joints([0, 0, -1.162, 0.144, 1.157, 0])
    else:
        move_joints([0, 0, -1.162, 0.144, 1.157, 0])

def start(niryo_api=False):
    if niryo_api:
        NIRYO.move_joints([0, 0.428, -1.237, 0.012, 0.814, -0.046])
    else:
        move_joints([0, 0.428, -1.237, 0.012, 0.814, -0.046])

def zero():
    move_joints([0, 0, 0, 0, 0, 0])

def check_joints(joints, tolerance=0.085):
    global group
    rospy.sleep(0.01)
    actual_joints = group.get_current_joint_values()
    for i in range(6):
        if abs(joints[i] - actual_joints[i]) > tolerance:
            print('Joint {:d} failed: ({:.3f}, {:.3f})'.format(i, joints[i], actual_joints[i]))
            return False
    return True


def capsule_cycle(Niryo):
    print('Capsule')
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    # start of sequence of serve capsule (start from zero())
    move_joints([-0.246, -0.711, 0.013, 0.260, 0.645, -0.177])

    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)
    shift_z(0.1)
    move_joints([-1.516, 0.102, -0.265, -0.025, -1.069, -0.020])
    move_joints([-1.536, -0.843, 0.881, 0.005, -1.460, -0.015])
    
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    move_joints([-1.555, -0.630, 0.952, 0.020, -1.476, -0.015])


def move_to_cup(Niryo):
    print('move to cup')
    move_joints([1.490, -0.368, -0.659, -0.252, 1.146, 0.056])
    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)


def dispense_cup(Niryo):
    print('dispense cup')
    Niryo.digital_write(MOTOR, 0)
    time.sleep(0.1)
    Niryo.digital_write(MOTOR, 1)
    time.sleep(0.1)


def move_cup_to_machine(Niryo):
    print('move cup to machine')
    move_joints([0, 0, -1.162, 0.144, 1.157, 0])
    move_joints([-1.199, -0.82, -0.795, 0.828, 1.57, -0.01])


def prepare_coffee(Niryo):
    print("Electromagnet On")
    NIRYO.digital_write(ELECTROMAGNET, True)
    time.sleep(45)
    NIRYO.digital_write(ELECTROMAGNET, False)
    print("Electromagnet Off")


def deliver_coffee(Niryo):
    print('deliver coffee')
    move_joints([-0.637, -0.769, -0.913, 0.622, 1.634, 0.03])
    move_joints([-0.685, -0.080, -0.466, 0.637, 0.577, -0.633])
    move_joints([0.047, -0.184, -0.313, 0.0, 0.525, -.046])
    move_joints([0.060, -0.445, -0.298, 0.103, 0.733, -0.121])
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    start()

def short_activate():
    """ 
    Quick slideover of magnet (for 1 second). Necessary to activate cleaning function
    """
    NIRYO.digital_write(ELECTROMAGNET, True)
    time.sleep(0.8)
    NIRYO.digital_write(ELECTROMAGNET, False)
    return True


def arm_script():
    latest_time = datetime.datetime.now()
    print('Start loop')
    start()
    while running.bool:
        # Takes 2 seconds of the sensor being activated
        # Checks 20 times over the next 2 seconds
        trigger_time = 2
        count = 0
        check_ticks = 20

        # Sensor output is inverted so we have to NOT all of them(?)
        trigger = not NIRYO.digital_read(SENSOR)
        if trigger:
            while count < trigger_time:
                trigger = not NIRYO.digital_read(SENSOR)
                print(str(count)+ '/' + str(trigger_time))
                time.sleep(trigger_time/float(check_ticks))
                count += trigger_time/float(check_ticks)
                if NIRYO.digital_read(SENSOR):
                    trigger = False
                    break
        time_diff = datetime.datetime.now() - latest_time
        print(str(time_diff.seconds) + " seconds since last run")
        if time_diff.seconds/60. >= 0.5:
            NIRYO.activate_learning_mode(1)
        else:
            if not check_joints(CROUCHINGTIGER):
                start()
        if time_diff.seconds/60. > 20:
            short_activate()
	    latest_time = datetime.datetime.now()
        if trigger:
            coffee_cycle(NIRYO)
            latest_time = datetime.datetime.now()


if __name__ == "__main__":
    print('Init node')
    rospy.init_node('move_group_python', anonymous=True, disable_signals=True)
    print('Init moveit')
    group_name = 'arm'
    group = moveit_commander.MoveGroupCommander(group_name)

    SENSOR = GPIO_1A
    MOTOR = GPIO_2A
    ELECTROMAGNET = SW_1

    CROUCHINGTIGER=[0, 0.428, -1.237, 0.012, 0.814, -0.046]
    arg = sys.argv[1]
    NIRYO = NiryoOne()
    NIRYO.pin_mode(SENSOR, 1)
    NIRYO.pin_mode(MOTOR,0)
    NIRYO.digital_write(MOTOR, 1)
    NIRYO.change_tool(TOOL_GRIPPER_1_ID)
    latest_time = datetime.datetime.now()
    if arg == "main":
        print('Start loop')
        start()
        while True:
            # Takes 2 seconds of the sensor being activated
            # Checks 20 times over the next 2 seconds
            trigger_time = 2
            count = 0
            check_ticks = 20
    
            # Sensor output is inverted so we have to NOT all of them(?)
            trigger = not NIRYO.digital_read(SENSOR)
            if trigger:
                while count < trigger_time:
                    trigger = not NIRYO.digital_read(SENSOR)
                    print(str(count)+ '/' + str(trigger_time))
                    time.sleep(trigger_time/float(check_ticks))
                    count += trigger_time/float(check_ticks)
                    if NIRYO.digital_read(SENSOR):
                        trigger = False
                        break
            # print("FINAL TRIGGER: ", trigger)
            time_diff = datetime.datetime.now() - latest_time
            if time_diff.seconds/60. >= 1:
                NIRYO.activate_learning_mode(1)
            else:
                if not check_joints(CROUCHINGTIGER):
                    start()
            if time_diff.seconds/60. > 20:
		short_activate()
		latest_time = datetime.datetime.now()
            if trigger:
                NIRYO.activate_learning_mode(0)
                coffee_cycle(NIRYO)
                latest_time = datetime.datetime.now()
    elif arg=="capsule":
        start()
        capsule_cycle(NIRYO)
    elif arg=="coffee":
        prepare_coffee(NIRYO)
    elif arg=="deliver":
        start()
        move_pose((0.064, -0.196, 0.077), (-0.048, 0.027, -2.101))
        deliver_coffee(NIRYO)
    elif arg == 'start':
        start()
    elif arg == 'zero':
        NIRYO.move_joints([0]*6)
        rospy.sleep(2)
