#!/usr/bin/env python3

# FILE:  test_docking.py

"""
   USAGE:  ./test_docking.py

   PROCESS:  Monitor INA219 current and voltage sensor
             Undock when charging current falls below cutoff target (175mA)
             Dock when voltage reaches 9.9v

   RESULTS:

       time - charge - cutoff - estimated more playtime at 6.5W average load
       1.5h - 23.3Wh - 643mA =  (3h 6m)
       2.0h - 26.1Wh - 331mA      +21m
       2.5h - 27.5Wh - 176mA      +11m (xtra hour charge = 32m xtra playtime)
       3.5h - 28.5Wh - 20mA       + 8m (xtra hour charge =  8m xtra playtime)

       Charge till 175mA = 2.3h
       Playtime till 9.9v = 3.6-3.8h
"""


import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
import docking
import battery
import lifeLog
import odomLog
import daveDataJson
import speak
from easy_ina219 import EasyINA219
from ina219 import DeviceRangeError
# import biasdrive

import time
import logging
import datetime as dt

UNDOCK_CHARGING_CURRENT_mA = 175
DOCK_VOLTAGE = 9.90


SPEED_MPS = 0.05  # m/s
# DOCKING_BIAS = 0.0  # m/s  add angular to make drive straight
DOCKING_FIX_DIST_CM = -1.0  # cm


DT_FORMAT = "%Y-%m-%d %H:%M:%S"

def charging(eina):
       current_now = eina.ave_milliamps()
       if (current_now < 0):
            charging = True
       else:
            charging = False
       return charging

def docking_failure_fix(egpg):
    tnow = time.strftime(DT_FORMAT)
    print(tnow,"docking_failure_fix: backing {:.1f} cm".format(DOCKING_FIX_DIST_CM))
    # biasdrive.drive_cm_bias(DOCKING_FIX_DIST_CM,DOCKING_BIAS,-SPEED_MPS,egpg)
    egpg.drive_cm(DOCKING_FIX_DIST_CM)

def do_charging(eina,egpg):

            dtLastStartCharging = dt.datetime.strptime(daveDataJson.getData("lastDockingTime"),DT_FORMAT)
            dtLastStartPlaytime = dt.datetime.strptime(daveDataJson.getData("lastDismountTime"),DT_FORMAT)
            batt_pct = battery.pctRemaining(egpg)
            charging_current = -1 * eina.milliamps()  # mA
            charging_voltage = eina.volts()
            while (charging_current > UNDOCK_CHARGING_CURRENT_mA):
              try:
                tnow = time.strftime(DT_FORMAT)
                print("{:s} Charging at {:.2f}v {:.0f}mA, Waiting for current < {:.0f}mA      ".format(
                       tnow,charging_voltage,charging_current,UNDOCK_CHARGING_CURRENT_mA),end="\r")
                time.sleep(6)
                charging_current = -1 * eina.milliamps()  # mA
                charging_voltage = eina.volts()
              except DeviceRangeError as e:
                print("\n",e)
                pass
            print("\n")
            tnow = time.strftime(DT_FORMAT)
            dtLastStartPlaytime = dt.datetime.now()
            charging_voltage = eina.volts()
            lastChargeTimeInSeconds = (dtLastStartPlaytime - dtLastStartCharging).total_seconds()
            lastChargeTimeInDays = divmod(lastChargeTimeInSeconds, 86400)
            lastChargeTimeHours = round( (lastChargeTimeInDays[1] / 3600.0),1)
            # print("lastChargeTimeInSeconds: {:.0f} lastChargeTimeInDays: {} lastChargeTimeHours: {:.1f}".format(lastChargeTimeInSeconds, lastChargeTimeInDays, lastChargeTimeHours))
            str_to_log = "---- Undocking at Charge Current {:.0f} mA {:.2f}v after {:.1f} h charging".format(UNDOCK_CHARGING_CURRENT_mA,charging_voltage,lastChargeTimeHours)
            print("\n{:s} {:s} ".format(tnow,str_to_log))
            lifeLog.logger.info(str_to_log)
            speak.say("Full Charge.  Undocking now.")
            docking.undock(egpg)

            daveDataJson.saveData('lastDismount', str_to_log)
            daveDataJson.saveData('lastRechargeDuration', lastChargeTimeHours)
            daveDataJson.saveData('lastDismountTime', tnow)
            daveDataJson.saveData('dockingState',"undocked")
            daveDataJson.saveData('chargingState',"discharging")


def do_playtime(eina,egpg):

            dtLastStartCharging = dt.datetime.strptime(daveDataJson.getData("lastDockingTime"),DT_FORMAT)
            dtLastStartPlaytime = dt.datetime.strptime(daveDataJson.getData("lastDismountTime"),DT_FORMAT)
            daveDataJson.saveData('dockingState',"undocked")
            daveDataJson.saveData('chargingState',"discharging")
            batt_pct = battery.pctRemaining(egpg)
            batt_voltage = eina.volts()
            while (batt_voltage > DOCK_VOLTAGE):
              try:
                tnow = time.strftime(DT_FORMAT)
                print("{:s} Battery at {:.0f}% {:.2f}v, Waiting for battery < {:.2f}v   ".format(
                       tnow,batt_pct*100,batt_voltage,DOCK_VOLTAGE),end="\r")
                time.sleep(6)
                batt_pct = battery.pctRemaining(egpg)
                batt_voltage = eina.volts()
              except DeviceRangeError as e:
                print("\n",e)
                pass
            print("\n")
            tnow = time.strftime(DT_FORMAT)
            vBattB4, vReadingB4 = battery.vBatt_vReading(egpg)
            batt_pctB4 = battery.pctRemaining(egpg)   # battery before docking
            vBattAveB4 = battery.aveBatteryV(egpg)
            # print("vBattB4: {:.2f}  vBattAveB4: {:.2f}  vReadingB4: {:.2f} volts Remaining: {:.0f}%".format(vBattB4, vBattAveB4, vReadingB4, batt_pctB4*100))
            speak.say("Battery at {:.1f} volts.  Docking now.".format(vBattAveB4))
            docking.dock(egpg)
            tnow = time.strftime(DT_FORMAT)
            daveDataJson.saveData('lastDockingTime', tnow)
            daveDataJson.saveData('dockingState',"docked")
            vBattAveDocked = battery.aveBatteryV(egpg)
            vBattDocked, vReadingDocked = battery.vBatt_vReading(egpg)
            batt_pctDocked = battery.pctRemaining(egpg)   # battery remaining after docking
            time.sleep(10)  # wait to see if charging starts
            if charging(eina):
                docking_success = True
                dtLastStartCharging = dt.datetime.now()
                try:
                    chargeCycles = int(daveDataJson.getData('chargeCycles'))
                    chargeCycles += 1
                except:
                    chargeCycles = 0

                lastPlaytimeInSeconds = (dtLastStartCharging - dtLastStartPlaytime).total_seconds()
                lastPlaytimeDays = divmod(lastPlaytimeInSeconds, 86400)
                lastPlaytimeHours = round( (lastPlaytimeDays[1] / 3600.0),1)
                # print("lastPlaytimeInSeconds: {:.0f} lastPlaytimeDays: {} lastPlaytimeHours: {:.1f}".format(lastChargeTimeInSeconds, lastChargeTimeInDays, lastChargeTimeHours))

                str_to_log = "---- Docking {} : success at {:.0f}% {:.1f}v after {:.1f} h playtime".format(chargeCycles,batt_pctB4,vBattAveB4,lastPlaytimeHours)
                daveDataJson.saveData('lastPlaytimeDuration', lastPlaytimeHours)
                daveDataJson.saveData('chargingState',"charging")
                daveDataJson.saveData('chargeCycles', chargeCycles)
                speak.say("Docking success after {:.1f} hours playtime".format(lastPlaytimeHours))
                str_to_odomlog = "***  Reset Encoders ***"
                egpg.reset_encoders(blocking=False)
                odomLog.logger.info(str_to_odomlog)
                speak.say("Reset Encoders To Zero")

            else:
                str_to_log = "Battery at {:.1f}v {:.0f}%, Docking: failure".format(vBattAveB4,batt_pctB4*100)
                docking_success = False
            lifeLog.logger.info(str_to_log)
            daveDataJson.saveData('lastDocking', str_to_log)
            tnow = time.strftime(DT_FORMAT)
            print(tnow,str_to_log)
            if (docking_success == False):
                daveDataJson.saveData('dockingState',"dockingfailure")
                print("\n",tnow,"Docking Failure. Attempting Fix.  Docking Failure.")
                speak.say("Docking Failure.  Attempting Fix.  Docking Failure.")
                docking_failure_fix(egpg)
                time.sleep(5)
                if charging(eina):
                    docking_success = True
                    dtLastStartCharging = dt.datetime.now()
                    chargeCycles = int(daveDataJson.getData('chargeCycles'))
                    chargeCycles += 1

                    lastPlaytimeInSeconds = (dtLastStartCharging - dtLastStartPlaytime).total_seconds()
                    lastPlaytimeDays = divmod(lastPlaytimeInSeconds, 86400)
                    lastPlaytimeHours = round( (lastPlaytimeDays[1] / 3600.0),1)

                    str_to_log = "---- Fix Docking {} : success at {:.0f}% {:.1f}v after {:.1f} h playtime".format(chargeCycles,batt_pctB4,vBattAveB4,lastPlaytimeHours)
                    lifeLog.logger.info(str_to_log)
                    daveDataJson.saveData('dockingState',"docked")
                    daveDataJson.saveData('lastDocking', str_to_log)
                    daveDataJson.saveData('lastPlaytimeDuration', lastPlaytimeHours)
                    daveDataJson.saveData('chargingState',"charging")
                    daveDataJson.saveData('chargeCycles', chargeCycles)
                    speak.say("Docking fix successful after {:.1f} hours playtime".format(lastPlaytimeHours))
                    str_to_odomlog = "***  Reset Encoders ***"
                    egpg.reset_encoders(blocking=False)
                    odomLog.logger.info(str_to_odomlog)
                    speak.say("Reset Encoders To Zero")

                else:
                    while True:
                        print("\n{:s} Docking Failure -  Test Stopped Early".format(tnow))
                        speak.say("Docking Failure.  Docking Failure Detected.  Test Stopped Early.  Docking Failure.")

                        try:
                            current_now = eina.ave_milliamps()
                            voltage_now = eina.ave_volts()
                            power_now =   eina.ave_watts()
                            tnow = time.strftime("%Y-%m-%d %H:%M:%S")
                            print("{} Docking Failed - Reading: {:.2f} V  {:.3f} A  {:.2f} W    ".format(tnow,voltage_now, current_now, power_now ))
                            time.sleep(10)
                        except DeviceRangeError as e:
                            print("\n",e)
                            pass



def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)
    eina = EasyINA219()
    test = 1

    try:
        while test > 0:
            tnow = time.strftime(DT_FORMAT)
            print("\n{:s} **** test_docking.main(): TEST {:d} ".format(tnow,test))
            if charging(eina):
                do_charging(eina,egpg)
            else:
                do_playtime(eina,egpg)
            time.sleep(10)  # Allow time for charging / discharge to settle

            if charging(eina):
                do_charging(eina,egpg)
            else:
                do_playtime(eina,egpg)
            time.sleep(10)  # allow time for charging / discharge to settle
            test += 1

    except KeyboardInterrupt:
        egpg.stop()
        tnow = time.strftime(DT_FORMAT)
        print("\n{:s} Test {:d} Stopped".format(tnow,test))

if __name__ == '__main__':
    main()
