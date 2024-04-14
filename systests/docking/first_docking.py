#!/usr/bin/env python3

# FILE:  docking.py

"""
   USAGE:
       import docking
       from noinit_easygopigo3 import EasyGoPiGo3

       egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

       docking.undock(egpg)
       docking.dock(egpg)
"""


import sys
sys.path.append('/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
# import battery
import time
import math
import statistics
import biasdrive

NUM_OF_DOCKING_TESTS = 10
UNDOCKED_SLEEP = 10
DOCKED_SLEEP = 30

SPEED_MPS = 0.05  # m/s
DOCKING_BIAS = -0.01  # m/s  add angular to make drive straight
DOCKING_DIST_CM = -17.4  # cm
UNDOCKING_BIAS = 0.03  # m/s  add angular to make drive straight
UNDOCKING_DIST_CM = 17.0  # cm

"""
def aveBatteryV(egpg,vlist):
    for i in range(3):
        vBatt = battery.vBatt_vReading(egpg)[0]
        # if vBatt is obviously out of range, read again
        if (vBatt < 9):  
            time.sleep(0.01)
            vBatt = battery.vBatt_vReading(egpg)[0]
        vlist += [vBatt]
        time.sleep(0.01)  # cannot be faster than 0.005
    if ( len(vlist)>9 ):
       # print("vlist before shortening: ",vlist)
       vlist = vlist[-9:]
       # print("vlist after shortening: ",vlist)
    return statistics.mean(vlist[-3:]),vlist

def delta_ave_vBatt(vlist):
    # print("vlist ", vlist)
    # print("vlist[-3:] ",vlist[-3:], "vlist[-9:-6] ", vlist[-9:-6])
    res = statistics.mean(vlist[-3:]) - statistics.mean(vlist[-9:-6])
    return res

def ave_vBatt_str(vBatt,deltavBatt,maxDeltavBatt):
    return "vbatt: {:>5.2f}  d(vBatt): {:>6.3f}  max d(vBatt): {:>6.3f}".format(vBatt,deltavBatt,maxDeltavBatt)

def motors_running(egpg):
    left_dps = egpg.get_motor_status(egpg.MOTOR_LEFT)[3]
    right_dps = egpg.get_motor_status(egpg.MOTOR_RIGHT)[3]
    if (left_dps == 0) and (right_dps == 0):
        return False
    else:
        return True
"""

def undock(egpg):
    tnow = time.strftime("%Y-%m-%d %H:%M:%S")
    print("\n{:s} Undocking Begins - dist:{:.1f} bias:{:.2f} speed:{:.2f}".format(tnow,UNDOCKING_DIST_CM, UNDOCKING_BIAS, SPEED_MPS))
    biasdrive.drive_cm_bias(UNDOCKING_DIST_CM,UNDOCKING_BIAS,SPEED_MPS,egpg)
    tnow = time.strftime("%Y-%m-%d %H:%M:%S")
    print("{:s} Undocking Complete".format(tnow))

def dock(egpg):
    tnow = time.strftime("%Y-%m-%d %H:%M:%S")
    print("\n{:s} Docking Begins - dist:{:.1f} bias:{:.2f} speed:{:.2f}".format(tnow,DOCKING_DIST_CM,DOCKING_BIAS,-SPEED_MPS))
    biasdrive.drive_cm_bias(DOCKING_DIST_CM,DOCKING_BIAS,-SPEED_MPS,egpg)
    tnow = time.strftime("%Y-%m-%d %H:%M:%S")
    print("{:s} Docking Complete".format(tnow))

"""
def olddock(egpg):
    vlist = []
    vBatt,vlist = aveBatteryV(egpg,vlist)
    vBatt,vlist = aveBatteryV(egpg,vlist)
    vBatt,vlist = aveBatteryV(egpg,vlist)
    davevBatt = delta_ave_vBatt(vlist)
    max_d_ave_vBatt = delta_ave_vBatt(vlist)
    notDocked = True
    egpg.set_speed(DOCKING_SPEED)
    try:
        print("Docking Begins")
        egpg.drive_cm(-1.0 * DOCKING_DISTANCE,blocking=False)
        time.sleep(0.1)  # allow time to start motors running
        while notDocked and motors_running(egpg):
            vBatt,vlist = aveBatteryV(egpg,vlist)
            davevBatt = delta_ave_vBatt(vlist)
            max_d_ave_vBatt = max(max_d_ave_vBatt,davevBatt)
            if (max_d_ave_vBatt > 1.0):
                print("resetting max_d_ave_vBatt: {:.3f}".format(max_d_ave_vBatt))
                max_d_ave_vBatt = 0
            print(ave_vBatt_str(vBatt,davevBatt,max_d_ave_vBatt)+"             ",end = "\r")
            if (max_d_ave_vBatt > 0.1):
                 print("\nDocking: Success")
                 notDocked = False
                 time.sleep(2)  # allow drive_cm() to complete even if charging contact has been noted
        if notDocked:
            print("\nDocking Failure")

    except KeyboardInterrupt:
        egpg.stop()

    except Exception as e:
        print(e)

    finally:
        print("\n")

"""

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
