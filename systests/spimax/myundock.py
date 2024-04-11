#!/usr/bin/env python3

# FILE:  myundock.py
#
# SPECIAL - using slower SPI max transfer rate


"""
   USAGE:  Set NUM_OF_DOCKING_TESTS and then execute ./test_docking.py
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
            docking.undock(egpg)
    except KeyboardInterrupt:
        egpg.stop()
        print("\nTest Terminated")
    finally:
        sleep(5)
        print("Outta Here...")

if __name__ == '__main__':
    main()
