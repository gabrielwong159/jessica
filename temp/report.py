#!/usr/bin/env python
from __future__ import print_function
import rospy
from niryo_one_python_api.niryo_one_api import *
from bot import send_message

rospy.init_node('niryo_one_temp_monitoring')
n = NiryoOne()

def check_temp():
    return n.get_hardware_status().temperatures


if __name__ == '__main__':
    while True:
        send_message(str(check_temp()))
        rospy.sleep(60)
