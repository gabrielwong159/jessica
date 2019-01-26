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
from capsule_process import *
from cup_movements import *

def coffee_cycle(niryo_one):
    capsule_cycle(niryo_one)
    move_to_cup(niryo_one)
    dispense_cup(niryo_one)
    move_cup_to_machine(niryo_one)
    prepare_coffee(niryo_one)
    deliver_coffee(niryo_one)

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
        while count < trigger_time:
            print(trigger)
            time.sleep(trigger_time/check_ticks)
            count += trigger_time/check_ticks;
            if NIRYO.digital_read(SENSOR):
                trigger = False
                break
        if trigger:
            coffee_cycle()
