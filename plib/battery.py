#!/usr/bin/python3

# FILE: battery.py

# PURPOSE:  Central facts and methods for the ModRobotics/TalentCell YB1203000 Li-Ion Battery Pack
#           12.6v to 9v 3A "3000mAh" Battery Pack
#
# Update:
#    2024-04: Added Battery class which uses INA219 current and voltage sensor

import sys
sys.path.insert(1,"/home/pi/GoPi5Go/plib/")
from noinit_easygopigo3 import EasyGoPiGo3
# from easygopigo3 import EasyGoPiGo3
import lifeLog
import numpy as np
import time
import statistics
import daveDataJson
import ina219
from threading import Thread

CHARGING_ADJUST_V = 0.2    # When charging the GoPiGo3 reading is closer to the actual battery voltage
REV_PROTECT_DIODE = 0.7    # The GoPiGo3 has a reverse polarity protection diode drop of 0.6v to 0.8v (n=2)
                            # This value results in average readings vs battery voltage error of +/- 0.03
SAFETY_SHUTDOWN_vBatt = 9.75   # Battery Discharge Protection Circuit allows down to 8.2v or so (9.75 leaves 5-12m reserve)
SAFETY_SHUTDOWN_vReading = SAFETY_SHUTDOWN_vBatt - REV_PROTECT_DIODE   # 8.5v EasyGoPiGo3.volt() reading
WARNING_LOW_vBatt = 9.9       # Give (~15 minutes) Advance Warning before safety shutdown
BATTERY_CLASS_RATE_PER_HOUR = 360     # update once every 10 seconds

# egpg.volt() Data points
# 2024-4-2 Data  4h19m 12.5v to 8.91v at cutoff - To Cutoff: 9.84v = 5m, 10.15v = 5% 13m, 10.29v = 10% 26m, 10.32v = 12% 20m
VOLTAGE_POINTS = \
    [8.5, 10.15, 10.29, 10.42, 10.56, 10.64,  10.69, 10.75,  10.81,  10.87,   10.9,  11.0,  11.14,  11.31,  11.42,  11.56,  11.68,  11.87,   12.0, 12.13,  12.36, 12.5]
BATTERY_REMAINING = \
    [0.0,   0.05,  0.10,  0.15,  0.20,  0.25,   0.30,  0.35,   0.40,   0.45,   0.50,  0.55,   0.60,   0.65,   0.70,   0.75,   0.80,   0.85,   0.90,  0.95,   0.99,  1.00]

def pctRemaining_from_vBatt(vBatt):
      FULL_CHARGE = 1.0
      PROTECTION_CUTOFF = 0.0
      pctRemaining = np.interp(vBatt,VOLTAGE_POINTS,BATTERY_REMAINING, right=FULL_CHARGE, left=PROTECTION_CUTOFF)
      return pctRemaining

def vBatt_vReading(egpg):
      vReading = 0
      try:
          vReading = egpg.volt()
      except Exception as e:
          str_to_log="Exception "+type(e).__name__+": "+str(e)+" vReading:{:.2f}".format(vReading)+" continuing"
          print(str_to_log)
          lifeLog.logger.info(str_to_log)
      chargingState = daveDataJson.getData('chargingState')
      if (chargingState == "charging"):
          # charging
          vBatt = vReading + REV_PROTECT_DIODE - CHARGING_ADJUST_V
          # print("charging")
      else:
          vBatt = vReading + REV_PROTECT_DIODE
      return vBatt,vReading

def voltages_string(egpg):
        vBatt, vReading = vBatt_vReading(egpg)
        pctRemaining = pctRemaining_from_vBatt(vBatt)*100.0
        return "Current Battery {:.1f}v  {:.1f}%  (Reading {:.2f}v)".format(vBatt,pctRemaining,vReading)

def too_low(egpg):
	vBatt, _ = vBatt_vReading(egpg)
	return vBatt < SAFETY_SHUTDOWN_vBatt

def on_last_leg(egpg):
	vBatt, _ = vBatt_vReading(egpg)
	return vBatt < WARNING_LOW_vBatt

def aveBatteryV(egpg):
    vlist = []
    for i in range(3):
        vBatt = vBatt_vReading(egpg)[0]
        # if vBatt is obviously out of range, read again
        if (vBatt < 8):
            time.sleep(0.01)
            vBatt = vBatt_vReading(egpg)[0]
        vlist += [vBatt]
        time.sleep(0.01)  # cannot be faster than 0.005
    return statistics.mean(vlist)

def pctRemaining(egpg):
    return(pctRemaining_from_vBatt(aveBatteryV(egpg)))

def charging(ina):
        clist = []
        for i in range(3):
            cBatt = ina.current()
            clist += [cBatt]
            time.sleep(0.02)  # cannot be faster than 0.02
        current = statistics.mean(clist)
        return (current < 0)

class Battery(Thread):
    """Class containing TalentCell YB1203000 parameters and methods"""
    ina = None   # INA219 class instance
    SHUNT_OHMS = 0.1
    MAX_EXPECTED_AMPS = 2.0

    power_meter = 0
    measurement_count = 0
    mAh_meter = 0
    rate_per_hour = BATTERY_CLASS_RATE_PER_HOUR
    charging = False
    last_charging = False
    log_to_console = False


    def __init__(self,log_to_console = False):
        super(Battery, self).__init__()       # init Battery thread
        self.daemon = True
        self.cancelled = False
        self.ina = ina219.INA219(self.SHUNT_OHMS, self.MAX_EXPECTED_AMPS,log_level=None)
        self.ina.configure(self.ina.RANGE_16V, bus_adc=self.ina.ADC_128SAMP,shunt_adc=self.ina.ADC_128SAMP)
        if (self.ave_current() < 0):
            self.charging = True
        else:
            self.charging = False
        self.last_charging = self.charging
        self.log_to_console = log_to_console

    def run(self):
       """Overloaded Thread.run, runs update method once every second"""
       while not self.cancelled:
           time.sleep(int(3600/self.rate_per_hour))
           self.updateBattery()

    def cancel(self):
        """End the battery update thread"""
        self.cancelled = True

    def updateBattery(self):
        """Update the battery power statistics"""
        current_now = self.ave_current()
        voltage_now = self.ave_voltage()
        power_now = self.ave_power()
        if (current_now < 0):
            self.charging = True
        else:
            self.charging = False

        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        if not (self.charging == self.last_charging):
            if self.log_to_console:
                if (self.last_charging == True):
                    print("{} Charge Complete: {:.0f} mAh  {:.1f}Wh \n".format(tnow,self.mAh_meter,self.power_meter))
                else:
                    print("{} Playtime Complete: {:.0f} mAh  {:.1f}Wh \n".format(tnow,self.mAh_meter,self.power_meter))
            self.mAh_meter = 0
            self.power_meter = 0
            self.last_charging = self.charging
        self.mAh_meter += current_now / self.rate_per_hour
        self.power_meter += power_now / self.rate_per_hour

        if self.log_to_console:
            print("{} Reading: {:.2f} V  {:.3f} A  {:.2f} W  {:.0f} mAh  {:.1f}Wh     ".format(
                  tnow,voltage_now, current_now/1000.0, power_now,self.mAh_meter,self.power_meter))



    def ave_voltage(self):
        vlist = []
        for i in range(3):
            vBatt = self.ina.voltage()
            vlist += [vBatt]
            time.sleep(0.02)  # cannot be faster than 0.02
        return statistics.mean(vlist)

    def ave_current(self):
        clist = []
        for i in range(3):
            cBatt = self.ina.current()
            clist += [cBatt]
            time.sleep(0.02)  # cannot be faster than 0.02
        return statistics.mean(clist)

    def ave_power(self):
        plist = []
        for i in range(3):
            pBatt = self.ina.power()
            plist += [pBatt]
            time.sleep(0.02)  # cannot be faster than 0.02
        return statistics.mean(plist)/1000.0

    def pctRemaining(self):
        # ina219.voltage() Data points
        # 2024-4-2 Data  4h19m 12.5v to 8.91v at cutoff - To Cutoff: 9.84v = 5m, 10.15v = 5% 13m, 10.29v = 10% 26m, 10.32v = 12% 20m
        VOLTAGE_POINTS = \
            [8.5,   9.74,  9.88, 10.04, 10.16, 10.22,  10.25, 10.26,  10.38,  10.43,  10.46, 11.53,  11.59,  10.69,  10.79,  10.89,  11.00,  11.17,  11.29, 11.44,  11.53, 11.55]
        BATTERY_REMAINING = \
            [0.0,   0.05,  0.10,  0.15,  0.20,  0.25,   0.30,  0.35,   0.40,   0.45,   0.50,  0.55,   0.60,   0.65,   0.70,   0.75,   0.80,   0.85,   0.90,  0.95,   0.99,  1.00]

        FULL_CHARGE = 1.0
        PROTECTION_CUTOFF = 0.0
        vBatt = self.ave_voltage()
        pctRemaining = np.interp(vBatt,VOLTAGE_POINTS,BATTERY_REMAINING, right=FULL_CHARGE, left=PROTECTION_CUTOFF) * 100
        return pctRemaining

    def too_low(self):
        vBatt = self.ave_voltage()
        return vBatt < SAFETY_SHUTDOWN_vBatt

    def on_last_leg(self):
        vBatt = self.ave_voltage()
        return vBatt < WARNING_LOW_vBatt


    def status_string(self):
        vBatt = self.ave_voltage()
        cBatt = self.ave_current()
        rBatt = self.pctRemaining()
        pBatt = self.ave_power()
        return "Current Battery {:.2f}v  {:.1f}% Load: {:.0f}mA {:.1f}W".format(vBatt,rBatt,cBatt,pBatt)






def testMain():
    egpg = EasyGoPiGo3(use_mutex=True, noinit=True)
    print(voltages_string(egpg))  
    print("Battery Remaining: {:.0f}% at 10.0v".format(pctRemaining_from_vBatt(10.0)*100 ))
    print("Battery Remaining: {:.0f}% at 9.75v".format(pctRemaining_from_vBatt(9.75)*100 ))
    print("Battery Percent Remaining now: {:.0f}%".format(pctRemaining(egpg)*100 ))

    batt = Battery(log_to_console=True)
    batt.start()
    print(batt.status_string())
    try:
        time.sleep(60)
    except KeyboardInterrupt:
        print("\n")
        pass
    finally:
        print("Closing Battery Thread")
        batt.cancel()
        time.sleep(1)
    print(batt.status_string())
    print("Done")


def main():
    batt = Battery(log_to_console=True)
    batt.start()
    print(batt.status_string())
    batt.cancel()


# if __name__ == '__main__': testMain()
if __name__ == '__main__': main()
