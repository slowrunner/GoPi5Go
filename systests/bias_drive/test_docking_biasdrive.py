#!/usr/bin/env python3

# FILE:  test_biasdrive.py

"""
   USAGE:  Set NUM_OF_DOCKING_TESTS and then execute ./test_biasdrive.py
"""


import sys
sys.path.append('/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
import biasdrive
from time import sleep

NUM_OF_DOCKING_TESTS = 25
UNDOCKED_SLEEP = 10
DOCKED_SLEEP = 30
SPEED = 0.05  # m/s
DOCKING_BIAS = 0.01  # m/s  add angular to make drive straight
UNDOCKING_BIAS = 0.03  # m/s  add angular to make drive straight
undocking_dist_cm = 17.0  # 100
docking_dist_cm = 17.4

def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    try:
        for test in range(NUM_OF_DOCKING_TESTS):
            print("\n*** test_docking.main(): TEST: ",test)
            biasdrive.drive_cm_bias(undocking_dist_cm, UNDOCKING_BIAS, SPEED, egpg)
            sleep(UNDOCKED_SLEEP)
            biasdrive.drive_cm_bias(-docking_dist_cm, -DOCKING_BIAS, -SPEED, egpg)
            if ((test+1) < NUM_OF_DOCKING_TESTS):  sleep(DOCKED_SLEEP)
        print("docking.main(): TEST COMPLETE")
    except KeyboardInterrupt:
        egpg.stop()
        print("\nTest Terminated")
    finally:
        egpg.stop()

if __name__ == '__main__':
    main()
