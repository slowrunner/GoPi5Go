#!/usr/bin/env python3

# FILE:  mydock.py
#
# SPECIAL - using slower SPI max transfer rate


"""
   USAGE:  Place Dave in front of dock, Execute ./mydock.py
"""


import sys
sys.path.append('/home/pi/GoPi5Go/plib')
from myeasygopigo3 import EasyGoPiGo3
import mydocking as docking
from time import sleep

NUM_OF_DOCKING_TESTS = 5


def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    try:
            docking.dock(egpg)
    except KeyboardInterrupt:
        egpg.stop()
        print("\nTest Terminated")
    finally:
        egpg.stop()

if __name__ == '__main__':
    main()
