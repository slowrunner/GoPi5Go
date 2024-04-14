#!/usr/bin/env python3

# FILE:  test_docking.py

"""
   USAGE:  Set NUM_OF_DOCKING_TESTS and then execute ./test_docking.py
"""


import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
import docking
import time
import battery
import lifeLog

NUM_OF_DOCKING_TESTS = 10
UNDOCKED_SLEEP = 10
DOCKED_SLEEP = 30

SPEED_MPS = 0.05  # m/s
DOCKING_BIAS = -0.01  # m/s  add angular to make drive straight
DOCKING_DIST_CM = -17.4  # cm
UNDOCKING_BIAS = 0.03  # m/s  add angular to make drive straight
UNDOCKING_DIST_CM = 17.0  # cm
UNDOCK_PCT = 0.90
DOCK_PCT = 0.10




def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)

    try:
        for test in range(NUM_OF_DOCKING_TESTS):

            tnow = time.strftime("%Y-%m-%d %H:%M:%S")
            print("\n{:s} **** test_docking.main(): TEST {:d} ".format(tnow,test))
            batt_pct = battery.pctRemaining(egpg)
            while (batt_pct < UNDOCK_PCT):
                tnow = time.strftime("%Y-%m-%d %H:%M:%S")
                print("{:s} Battery at {:.0f}%, Waiting for battery > {:.0f}%".format(tnow,batt_pct*100,UNDOCK_PCT*100),end="\r")
                time.sleep(6)
                batt_pct = battery.pctRemaining(egpg)
            tnow = time.strftime("%Y-%m-%d %H:%M:%S")
            print("\n{:s} Battery at {:.0f}%, UNDOCKING".format(tnow,batt_pct*100))
            str_to_log = "Battery at {:.0f}%, undocking".format(batt_pct*100)
            lifeLog.logger.info(str_to_log)
            docking.undock(egpg)
            # time.sleep(UNDOCKED_SLEEP)
            batt_pct = battery.pctRemaining(egpg)
            while (batt_pct > DOCK_PCT):
                tnow = time.strftime("%Y-%m-%d %H:%M:%S")
                print("{:s} Battery at {:.0f}%, Waiting for battery < {:.0f}%".format(tnow,batt_pct*100,DOCK_PCT*100),end="\r")
                time.sleep(6)
                batt_pct = battery.pctRemaining(egpg)
            tnow = time.strftime("%Y-%m-%d %H:%M:%S")
            print("\n{:s} Battery at {:.0f}%, DOCKING".format(tnow,batt_pct*100))
            vBattB4, vReadingB4 = battery.vBatt_vReading(egpg)
            batt_pctB4 = battery.pctRemaining(egpg)   # battery before docking
            vBattAveB4 = battery.aveBatteryV(egpg)
            print("vBattB4: {:.2f}  vBattAveB4: {:.2f}  vReadingB4: {:.2f} volts Remaining: {:.0f}%".format(vBattB4, vBattAveB4, vReadingB4, batt_pctB4*100))
            docking.dock(egpg)
            vBattAveDocked = battery.aveBatteryV(egpg)
            vBattDocked, vReadingDocked = battery.vBatt_vReading(egpg)
            batt_pctDocked = battery.pctRemaining(egpg)   # battery remaining after docking
            print("vBattDocked: {:.2f}  vBattAveDocked: {:.2f}  vReadingDocked: {:.2f} volts Remaining: {:.0f}%".format(vBattDocked, vBattAveDocked, vReadingDocked, batt_pctDocked*100))
            str_to_log = "Battery at {:.1f}v {:.0f}%, Docking: success (assumed)".format(vBattAveB4,batt_pctB4*100)
            lifeLog.logger.info(str_to_log)

        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        print("{:s} test_docking.main(): TEST COMPLETE".format(tnow))
    except KeyboardInterrupt:
        egpg.stop()
        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        print("\n{:s} Test Stopped Early".format(tnow))

if __name__ == '__main__':
    main()
