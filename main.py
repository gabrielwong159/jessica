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

# MAIN FUNCTION
def coffee_cycle(niryo_one):
    capsule_cycle(niryo_one)
    move_to_cup(niryo_one)
    dispense_cup(niryo_one)
    move_cup_to_machine(niryo_one)
    prepare_coffee(niryo_one)
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

from niryo_one_python_api.niryo_one_api import *

def move_to_cup(Niryo):
    rospy.sleep(0.2)
    # home()
    zero()
    # grab a cup
    #move_pose((-0.005, 0.24, 0.295), (0, 0, 1.735))
    rospy.sleep(0.2)
    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)
    move_pose((0.014, 0.239, 0.202), (0, 0, 1.735))
    return

def dispense_cup(niryo_one):
    MOTOR = GPIO_2A
    niryo_one.pin_mode(MOTOR,0)
    niryo_one.digital_write(MOTOR, 1)
    niryo_one.pin_mode(MOTOR2,0)
    niryo_one.digital_write(MOTOR2, 1)
    niryo_one.pin_mode(MOTOR3,0)
    niryo_one.digital_write(MOTOR3, 1)
    time.sleep(0.1)
    niryo_one.digital_write(MOTOR, 0)
    niryo_one.digital_write(MOTOR2, 0)
    niryo_one.digital_write(MOTOR3, 0)
    time.sleep(0.1)

def move_cup_to_machine(Niryo):
    shift_z(-0.15)
    # return cup
    home()
    move_joint(1.176, 0.029, -0.641, 1.345, 1.295, -0.901)
    move_pose((0.297, 0.011, 0.285), (0, 0, 0))
    shift_z(-0.015)
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    zero()
    home()

def prepare_coffee(Niryo):
    ELECTROMAGNET = SW_1
    n.digital_write(ELECTROMAGNET, True)
    time.sleep(30)
    n.digital_write(ELECTROMAGNET, False)
    return True

def deliver_coffee(Niryo):
    return True

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

if __name__ == "__main__":
    #!/usr/bin/env python
    SENSOR = GPIO_1A
    NIRYO = NiryoOne()
    NIRYO.pin_mode(SENSOR, 1)
    while True:
        # Takes 2 seconds of the sensor being activated
        # Checks 20 times over the next 2 seconds
        trigger_time = 2
        count = 0
        check_ticks = 20

        # Sensor output is inverted so we have to NOT all of them(?)
        trigger = not NIRYO.digital_read(SENSOR)
        print(trigger)
        while count < trigger_time:
            time.sleep(trigger_time/check_ticks)
            count += trigger_time/check_ticks;
            if NIRYO.digital_read(SENSOR):
                trigger = False
                break
        if trigger:
            coffee_cycle()
