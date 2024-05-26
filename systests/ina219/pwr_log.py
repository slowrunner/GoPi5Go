#!/usr/bin/env python3

# FILE: pwr_log.py

# OPERATION:  prints date/time stamped supply_voltage, current, load, and cumulative capacity in mAh and Wh
#             prints charge / playtime complete summary when current changes sign
#
# NOTE: supply voltage measured to be around 50mV less than measured by AstroAI DT132A meter
#       (version of this program prior to 2024-04-23 used bus voltage which was around 120mV less than meter measurement)

from easy_ina219 import EasyINA219
import time
from ina219 import DeviceRangeError


eina = EasyINA219()
time.sleep(0.5)

power_meter = 0
measurement_count = 0
mAh_meter = 0

rate_per_hour = 120


try:
    current_now = eina.milliamps()
    if (current_now < 0):
        charging = True
    else:
        charging = False
    last_charging = charging

    while True:
      try:
        current_now = eina.ave_milliamps()
        voltage_now = eina.ave_volts()
        power_now = eina.ave_watts()
        if (current_now < 0):
            charging = True
        else:
            charging = False

        tnow = time.strftime("%Y-%m-%d %H:%M:%S")
        if not (charging == last_charging):
            if (last_charging == True):
                print("{} Charge Complete: {:.0f} mAh  {:.1f}Wh \n".format(tnow,mAh_meter,power_meter))
            else:
                print("{} Playtime Complete: {:.0f} mAh  {:.1f}Wh \n".format(tnow,mAh_meter,power_meter))
            mAh_meter = 0
            power_meter = 0
            last_charging = charging
        mAh_meter += current_now / rate_per_hour 
        power_meter += power_now / rate_per_hour

        print("{} Reading: {:.2f} V  {:.3f} A  {:.2f} W  {:.0f} mAh  {:.1f}Wh     ".format(tnow,eina.ave_volts(), eina.ave_milliamps()/1000.0, eina.ave_watts(),mAh_meter,power_meter))
        time.sleep(3600/rate_per_hour)
      except DeviceRangeError as e:
        print("ignoring: ",e)

except Exception as e:
    print(e)
