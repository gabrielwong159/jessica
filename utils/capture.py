#!/usr/bin/env python
from __future__ import print_function

import rospy
from niryo_one_python_api.niryo_one_api import *

rospy.init_node('niryo')
n = NiryoOne()


def write(s, filename='a.txt'):
    with open(filename, 'a') as f:
        f.write(s)
    print(s)


def capture_joints():
    joints = n.get_joints()
    s = 'move_joints(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)\n' % \
        tuple(joints)
    write(s)


def capture_pose():
    pose = n.get_arm_pose()
    s = 'move_pose((%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f))\n' % \
            (pose.position.x, pose.position.y, pose.position.z,
             pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw)
    write(s)


def main():
    print('Start (j, p, q)')
    while True:
        print('>', end=' ')
        c = raw_input()
        if c == 'q':
            return

        if c == 'j':
            capture_joints()
        elif c == 'p':
            capture_pose()


if __name__ == '__main__':
    main()

