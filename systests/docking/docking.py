#!/usr/bin/env python3

# FILE:  docking.py

"""
   USAGE:
       import sys
       sys.path.insert(1,'/home/pi/GoPi5Go/plib')
       import docking
       from noinit_easygopigo3 import EasyGoPiGo3

       egpg = EasyGoPiGo3(use_mutex=True, noinit=True)
       docking.undock(egpg)
       docking.dock(egpg)

   ABOUT:
       These functions use drive_cm() with values 
       from empirical testing.

       The docking simply drives backward 17.4 cm, nothing more.
       The undocking simply drives forward 17.0 cm.
"""


import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
import time
# import biasdrive

NUM_OF_DOCKING_TESTS = 10
UNDOCKED_SLEEP = 10
DOCKED_SLEEP = 30

SPEED_MPS = 0.05  # m/s
DOCKING_SPEED_DPI = 50
# DOCKING_BIAS = -0.01  # m/s  add angular to make drive straight
# DOCKING_BIAS = -0.00  # m/s  add angular to make drive straight
DOCKING_DIST_CM = -17.4  # cm
# UNDOCKING_BIAS = 0.03  # m/s  add angular to make drive straight
# UNDOCKING_BIAS = 0.00  # m/s  add angular to make drive straight
UNDOCKING_DIST_CM = 17.0  # cm


def undock(egpg):
    tnow = time.strftime("%Y-%m-%d %H:%M:%S")
    # print("\n{:s} Undocking Begins - dist:{:.1f} bias:{:.2f} speed:{:.2f}".format(tnow,UNDOCKING_DIST_CM, UNDOCKING_BIAS, SPEED_MPS))
    print("\n{:s} Undocking Begins - dist:{:.1f}".format(tnow,UNDOCKING_DIST_CM))
    # biasdrive.drive_cm_bias(UNDOCKING_DIST_CM,UNDOCKING_BIAS,SPEED_MPS,egpg)
    egpg.set_speed(DOCKING_SPEED_DPI)
    egpg.drive_cm(UNDOCKING_DIST_CM)
    tnow = time.strftime("%Y-%m-%d %H:%M:%S")
    print("{:s} Undocking Complete".format(tnow))

def dock(egpg):
    tnow = time.strftime("%Y-%m-%d %H:%M:%S")
    # print("\n{:s} Docking Begins - dist:{:.1f} bias:{:.2f} speed:{:.2f}".format(tnow,DOCKING_DIST_CM,DOCKING_BIAS,-SPEED_MPS))
    print("\n{:s} Docking Begins - dist:{:.1f}".format(tnow,DOCKING_DIST_CM))
    # biasdrive.drive_cm_bias(DOCKING_DIST_CM,DOCKING_BIAS,-SPEED_MPS,egpg)
    egpg.set_speed(DOCKING_SPEED_DPI)
    egpg.drive_cm(DOCKING_DIST_CM)
    tnow = time.strftime("%Y-%m-%d %H:%M:%S")
    print("{:s} Docking Complete".format(tnow))


def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    try:
        for test in range(NUM_OF_DOCKING_TESTS):

            tnow = time.strftime("%Y-%m-%d %H:%M:%S")
            print("\n{:s} **** docking.main(): TEST {:d} ".format(tnow,test))

            undock(egpg)
            time.sleep(UNDOCKED_SLEEP)

            dock(egpg)
            time.sleep(DOCKED_SLEEP)

        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        print("{:s} docking.main(): TEST COMPLETE".format(tnow))
    except KeyboardInterrupt:
        egpg.stop()
        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        print("\n{:s} Test Stopped Early".format(tnow))
if __name__ == '__main__':
    main()
