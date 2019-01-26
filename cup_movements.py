#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from tf.transformations import quaternion_from_euler
from niryo_one_python_api.niryo_one_api import *
from utils.movement import *

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

if __name__ == '__main__':
    # n.calibrate_manual()
    n.change_tool(TOOL_GRIPPER_1_ID)
    move_to_cup()
