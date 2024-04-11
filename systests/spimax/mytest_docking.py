#!/usr/bin/env python3

# FILE:  mytest_docking.py

"""
   USAGE:  Set NUM_OF_DOCKING_TESTS and then execute ./test_docking.py
"""


import sys
sys.path.append('/home/pi/GoPi5Go/plib')
# from noinit_easygopigo3 import EasyGoPiGo3
from myeasygopigo3 import EasyGoPiGo3
import mydocking as docking
from time import sleep

NUM_OF_DOCKING_TESTS = 15


def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    try:
        for test in range(NUM_OF_DOCKING_TESTS):
            print("test_docking.main(): Test: ",test)
            docking.undock(egpg)
            sleep(5)
            docking.dock(egpg)
            if ((test+1) < NUM_OF_DOCKING_TESTS):  sleep(5)
        print("docking.main(): TEST COMPLETE")
    except KeyboardInterrupt:
        egpg.stop()
        print("\nTest Terminated")
    finally:
        egpg.stop()

if __name__ == '__main__':
    main()
