#!/usr/bin/env python3

# FILE:  mydocking.py
#
# SPECIAL - using slower SPI max transfer rate


"""
   USAGE:
       import mydocking as docking
       from myeasygopigo3 import EasyGoPiGo3

       egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

       docking.undock(egpg)
       docking.dock(egpg)
"""


import sys
sys.path.append('/home/pi/GoPi5Go/plib')
# from noinit_easygopigo3 import EasyGoPiGo3
# import battery
from myeasygopigo3 import EasyGoPiGo3
import mybattery as battery
from time import sleep
import math
import statistics

DOCKING_SPEED = 60
NUM_OF_DOCKING_TESTS = 1
UNDOCK_DISTANCE = 17.0  #cm
DOCKING_DISTANCE = UNDOCK_DISTANCE + 0.2  # in cm - add x millimeters to compensate for slip getting onto dock

def aveBatteryV(egpg,vlist):
    for i in range(3):
        vBatt = battery.vBatt_vReading(egpg)[0]
        # if vBatt is obviously out of range, read again
        if (vBatt < 9):  
            sleep(0.01)
            vBatt = battery.vBatt_vReading(egpg)[0]
        vlist += [vBatt]
        sleep(0.01)  # cannot be faster than 0.005
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

def undock(egpg):
    print("Undocking Begins")
    egpg.set_speed(DOCKING_SPEED)
    egpg.drive_cm(UNDOCK_DISTANCE)
    print("Undocking Complete")

def dock(egpg):
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
        sleep(0.1)  # allow time to start motors running
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
                 sleep(2)  # allow drive_cm() to complete even if charging contact has been noted
        if notDocked:
            print("\nDocking Failure")

    except KeyboardInterrupt:
        egpg.stop()

    except Exception as e:
        print(e)

    finally:
        print("\n")

def main():
    global egpg

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)


    for test in range(NUM_OF_DOCKING_TESTS):
        print("docking.main(): Test: ",test)
        undock(egpg)
        sleep(5)
        dock(egpg)
        sleep(10)
    print("docking.main(): TEST COMPLETE")

if __name__ == '__main__':
    main()
