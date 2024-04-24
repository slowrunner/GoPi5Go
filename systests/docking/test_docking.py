#!/usr/bin/env python3

# FILE:  test_docking.py

"""
   USAGE:  Set NUM_OF_DOCKING_TESTS and then execute ./test_docking.py

   PROCESS:  Monitor INA219 current and voltage sensor
             Undock when charging current falls below cutoff target
             Dock when voltage reaches 9.85v

   RESULTS:

       time - charge - cutoff - estimated more playtime at 6.5W average load
       1.5h - 23.3Wh - 643mA =  (3h 6m)
       2.0h - 26.1Wh - 331mA      +21m
       2.5h - 27.5Wh - 176mA      +11m (xtra hour charge = 32m xtra playtime)
       3.5h - 28.5Wh - 20mA       + 8m (xtra hour charge =  8m xtra playtime)
"""


import sys
sys.path.insert(1,'/home/pi/GoPi5Go/plib')
from noinit_easygopigo3 import EasyGoPiGo3
import docking
import time
import battery
import lifeLog
import  daveDataJson
import ina219
import logging
import datetime as dt

NUM_OF_DOCKING_TESTS = 10

SPEED_MPS = 0.05  # m/s
DOCKING_BIAS = -0.01  # m/s  add angular to make drive straight
DOCKING_DIST_CM = -17.4  # cm
UNDOCKING_BIAS = 0.03  # m/s  add angular to make drive straight
UNDOCKING_DIST_CM = 17.0  # cm

UNDOCK_CHARGING_CURRENT_mA = 175
DOCK_VOLTAGE = 9.85
DOCKING_SUCCESS_dvBatt = 0.1  # delta average battery voltage (rise) after successful docking

SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 2.0


def do_charging(ina,egpg):
            batt_pct = battery.pctRemaining(egpg)
            charging_current = -1 * ina.current()  # mA
            charging_voltage = ina.supply_voltage()
            while (charging_current > UNDOCK_CHARGING_CURRENT_mA):
                tnow = time.strftime("%Y-%m-%d %H:%M:%S")
                print("{:s} Charging at {:.2f}v {:.0f}mA, Waiting for current < {:.0f}mA      ".format(tnow,charging_voltage,charging_current,UNDOCK_CHARGING_CURRENT_mA),end="\r")
                time.sleep(6)
                charging_current = -1 * ina.current()  # mA
                charging_voltage = ina.supply_voltage()

            print("\n")
            tnow = time.strftime("%Y-%m-%d %H:%M:%S")
            dtLastStartPlaytime = dt.datetime.now()
            charging_voltage = ina.supply_voltage()
            lastChargeTimeInSeconds = (dtLastStartPlaytime - dtLastStartCharging).total_seconds()
            lastChargeTimeInDays = divmod(lastChargeTimeInSeconds, 86400)
            lastChargeTimeHours = round( (lastChargeTimeInDays[1] / 3600.0),1)
            # print("lastChargeTimeInSeconds: {:.0f} lastChargeTimeInDays: {} lastChargeTimeHours: {:.1f}".format(lastChargeTimeInSeconds, lastChargeTimeInDays, lastChargeTimeHours))
            str_to_log = "---- Undocking at Charge Current {:.0f} mA {:.2f}v after {:.1f} h charging".format(UNDOCK_CHARGING_CURRENT_mA,charging_voltage,lastChargeTimeHours)
            print("\n{:s} {:s} ".format(tnow,str_to_log))
            lifeLog.logger.info(str_to_log)
            docking.undock(egpg)

            daveDataJson.saveData('lastDismount', str_to_log)
            daveDataJson.saveData('lastRechargeDuration', lastChargeTimeHours)
            daveDataJson.saveData('lastDismountTime', tnow)
            daveDataJson.saveData('dockingState',"undocked")
            daveDataJson.saveData('chargingState',"discharging")


def do_playtime(ina,egpg):
            batt_pct = battery.pctRemaining(egpg)
            batt_voltage = ina.supply_voltage()
            while (batt_voltage > DOCK_VOLTAGE):
                tnow = time.strftime("%Y-%m-%d %H:%M:%S")
                print("{:s} Battery at {:.0f}% {:.2f}v, Waiting for battery < {:.2f}v".format(tnow,batt_pct*100,batt_voltage,DOCK_VOLTAGE),end="\r")
                time.sleep(6)
                batt_pct = battery.pctRemaining(egpg)
                batt_voltage = ina.supply_voltage()
            print("\n")
            tnow = time.strftime("%Y-%m-%d %H:%M:%S")
            vBattB4, vReadingB4 = battery.vBatt_vReading(egpg)
            batt_pctB4 = battery.pctRemaining(egpg)   # battery before docking
            vBattAveB4 = battery.aveBatteryV(egpg)
            # print("vBattB4: {:.2f}  vBattAveB4: {:.2f}  vReadingB4: {:.2f} volts Remaining: {:.0f}%".format(vBattB4, vBattAveB4, vReadingB4, batt_pctB4*100))
            docking.dock(egpg)
            vBattAveDocked = battery.aveBatteryV(egpg)
            vBattDocked, vReadingDocked = battery.vBatt_vReading(egpg)
            batt_pctDocked = battery.pctRemaining(egpg)   # battery remaining after docking
            # print("vBattDocked: {:.2f}  vBattAveDocked: {:.2f}  vReadingDocked: {:.2f} volts Remaining: {:.0f}%".format(vBattDocked, vBattAveDocked, vReadingDocked, batt_pctDocked*100))
            dvBatt = vBattAveDocked - vBattAveB4
            if (dvBatt > DOCKING_SUCCESS_dvBatt):
                tnow = time.strftime("%Y-%m-%d %H:%M:%S")
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
                daveDataJson.saveData('lastDockingTime', tnow)
                daveDataJson.saveData('dockingState',"docked")
                daveDataJson.saveData('chargingState',"charging")
                daveDataJson.saveData('chargeCycles', chargeCycles)


            else:
                str_to_log = "Battery at {:.1f}v {:.0f}%, Docking: failure".format(vBattAveB4,batt_pctB4*100)
                docking_success = False
            lifeLog.logger.info(str_to_log)
            daveDataJson.saveData('lastDocking', str_to_log)

            if (docking_success == False):
                daveDataJson.saveData('dockingState',"dockingfailure")
                tnow = time.strftime("%Y-%m-%d %H:%M:%S")
                print("\n{:s} Docking Failure (dvBatt: {:.2f}v) -  Test Stopped Early".format(tnow,dvBatt))
                egpg.stop()
                sys.exit(1)



def main():

    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)
    ina = ina219.INA219(SHUNT_OHMS,MAX_EXPECTED_AMPS, log_level=None)
    ina.configure(ina.RANGE_16V,bus_adc=ina.ADC_128SAMP,shunt_adc=ina.ADC_128SAMP)

    dtStart = dt.datetime.now()
    dtLastStartCharging = dtStart
    dtLastStartPlaytime = dtStart

    try:
        for test in range(NUM_OF_DOCKING_TESTS):
            tnow = time.strftime("%Y-%m-%d %H:%M:%S")
            print("\n{:s} **** test_docking.main(): TEST {:d} ".format(tnow,test))
            if battery.charging(ina):
                do_charging(ina,egpg)
            else:
                do_playtime(ina,egpg)

        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        print("{:s} test_docking.main(): TEST COMPLETE".format(tnow))
    except KeyboardInterrupt:
        egpg.stop()
        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        print("\n{:s} Test Stopped Early".format(tnow))

if __name__ == '__main__':
    main()
