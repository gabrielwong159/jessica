#!/usr/bin/env python
import rospy
import time
from niryo_one_python_api.niryo_one_api import *

MOTOR = GPIO_2A
MOTOR2 = GPIO_2B
MOTOR3 = GPIO_2C
n = NiryoOne()

if __name__ == '__main__':
    n.pin_mode(MOTOR,0) 
    n.digital_write(MOTOR, 1)
    n.pin_mode(MOTOR2,0) 
    n.digital_write(MOTOR2, 1)
    n.pin_mode(MOTOR3,0) 
    n.digital_write(MOTOR3, 1)
    time.sleep(0.1)
    n.digital_write(MOTOR, 0)
    n.digital_write(MOTOR2, 0)
    n.digital_write(MOTOR3, 0)
    time.sleep(1)
