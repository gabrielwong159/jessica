#!/usr/bin/env python
from __future__ import print_function
import rospy
from niryo_one_python_api.niryo_one_api import *
from bot import send_message

rospy.init_node('niryo_one_temp_monitoring')
n = NiryoOne()

THRESHOLD = 10


def check_temp():
    temp = n.get_hardware_status().temperatures
    targets = [(i, t) for i, t in enumerate(temp) if t >= THRESHOLD]
    return targets


def main():
    targets = check_temp()
    for i, t in targets:
        message = 'Motor %d temperature: %d' % (i, t)
        send_message(message)


if __name__ == '__main__':
    while True:
        main()
        rospy.sleep(60 * 5)
