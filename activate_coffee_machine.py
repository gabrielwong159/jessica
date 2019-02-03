#!/usr/bin/env python
import time
from niryo_one_python_api.niryo_one_api import *

ELECTROMAGNET = SW_1

n = NiryoOne()

def short_activate(n):
    """
    Quick slideover of magnet (for 1 second). Necessary to activate cleaning function
    """
    n.digital_write(ELECTROMAGNET, True)
    time.sleep(0.8)
    n.digital_write(ELECTROMAGNET, False)
    return True


def start_coffee(n):
    """
    Starts coffee making process. Timed to be about 27 seconds; use 30s to be safe
    """
    n.digital_write(ELECTROMAGNET, True)
    time.sleep(30)
    n.digital_write(ELECTROMAGNET, False)
    return True


if __name__ == '__main__':
    n = NiryoOne()
    short_activate(n)
