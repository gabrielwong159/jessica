#!/usr/bin/env python
import time
from niryo_one_python_api.niryo_one_api import *

SENSOR = GPIO_1A
LED = GPIO_2A

n = NiryoOne()


if __name__ == '__main__':
    n.pin_mode(SENSOR, 1)
    n.pin_mode(LED, 0)

    while True:
        val = n.digital_read(SENSOR)
        n.digital_write(LED, val)
        time.sleep(0.1)
        print(val)

